// SPDX-FileCopyrightText: 2002-2026 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

#pragma once

#include "common/Pcsx2Types.h"

#include <array>
#include <string>
#include <vector>

namespace Pif
{
	// The DECI2 manager of the TOOL's Linux board (dsnetm 1.2.0), between its TCP
	// clients (NETMP) and the target behind the PIF card. Only the routing; sockets
	// and queues belong to the caller (PifBridge.cpp). Follows the TOOL's own dsnetm
	// binary where it differs from the NETMP and DCMP specs, e.g. CONNECTR and
	// MESSAGER echo the request's body. Needs proper testing beyond the sessions it
	// was compared on (docs/05).
	//
	// Packets are whole DECI2 packets (8-byte header included). Not thread-safe:
	// call it from one thread.
	class Dsnetm
	{
	public:
		using Packet = std::vector<u8>;

		class Output
		{
		public:
			virtual ~Output() = default;
			virtual void ToClient(int client, Packet pkt) = 0;
			virtual void ToTarget(Packet pkt) = 0;
			// NETMP KILL: close this client. Dsnetm has already forgotten it.
			virtual void CloseClient(int client) = 0;
			// NETMP RESET: reset the target, then connect with these boot parameters (EE 8 bytes, IOP 8
			// bytes). Call TargetBooted once it has booted again.
			virtual void ResetTarget(const std::array<u8, 16>& boot_params) = 0;
			// A TTYP flush request: drop the packets of this protocol still queued for
			// every client (not one already being sent).
			virtual void FlushQueued(u16 proto) = 0;
			// An EMUP packet (PifEmup.h) from this client: the emulator answers it, not
			// the target. Only the built-in dsnetm has this; the TOOL's bounces it.
			virtual void Emulator(int client, Packet pkt) = 0;
		};

		static constexpr const char* VERSION = "1.2.0";
		// NETMP RESETR causes (NETMP spec; dsnetm passes mrp.o's RSTINFO).
		static constexpr int CAUSE_SOFTWARE = 0x00;
		static constexpr int CAUSE_BUTTON = 0x04;
		static constexpr int CAUSE_NETMP = 0x06;

		explicit Dsnetm(Output& out);

		// dsnetm -nci / -nce: don't send DCMP connects to the IOP or the EE.
		void SetConnectNodes(bool iop, bool ee);

		void ClientOpened(int client, u64 now_ms);
		void ClientClosed(int client);
		void FromClient(int client, Packet pkt, u64 now_ms);
		void FromTarget(Packet pkt);

		// The target has booted (DECI2DRP's handshake): what mrp.o's EPIPE and the
		// reconnect that follows do in dsnetm. Sends registered clients a NETMP RESETR
		// with this cause (none if negative), then DCMP connects to the IOP and the EE,
		// unless the caller already put them in front of the target (`connects_sent`).
		void TargetBooted(int cause, bool connects_sent = false);

		// dsnetm's DCMP connect to a node (I or E), with the boot parameters.
		static Packet ConnectPacket(u8 node, const std::array<u8, 16>& boot_params);

		// The TTY protocols dsnetm delivers to every client that registered them.
		static bool IsTtyp(u16 proto);

	private:
		static constexpr int TARGET = -1;

		struct Client
		{
			int id;
			u64 opened_ms;
			bool registered = false;
			// NETMP CONNECT entries as sent: priority, 0, protocol (LE).
			std::vector<std::array<u8, 4>> protos;
			// The owner string from NETMP MESSAGE, if one came (even an empty one).
			std::string message;
			bool hasMessage = false;
		};

		// A packet that found no client: its protocol gets a DCMP status (PROTO)
		// once a client registers it.
		struct Bounced
		{
			int owner;
			u16 proto;
			u8 node;
		};

		Client* FindClient(int id);
		// The client holding proto at the highest priority (the first one on a tie).
		Client* Holder(u16 proto);
		bool Busy(u8 prio, u16 proto) const;

		void Send(int owner, Packet pkt);
		// ds_send_net: proto 0 to every registered client, TTYP to each one
		// registered for it, anything else to the holder.
		void Deliver(u16 proto, const Packet& pkt);
		// send_net: deliver, or bounce with a DCMP error (NOPROTO).
		void Route(int owner, const Packet& pkt);
		void DcmpError(int owner, const Packet& pkt, u8 code);
		void SendConnect(u8 node);
		void ForgetNodes();

		void Netmp(Client& client, const Packet& pkt, u64 now_ms);
		Packet StatusReply(u64 now_ms) const;
		void DcmpConnect(const Packet& pkt);
		void DcmpErrorFromTarget(const Packet& pkt);
		void Drfp2(const Packet& pkt);

		Output& m_out;
		std::vector<Client> m_clients;
		std::vector<Bounced> m_bounced;
		bool m_connectIop = true;
		bool m_connectEe = true;
		bool m_iopConnected = false;
		bool m_eeConnected = false;
		bool m_iopStatusSent = false;
		bool m_eeStatusSent = false;
		// The boot parameters for the DCMP connects (EE 8 bytes, IOP 8 bytes), from NETMP RESET.
		std::array<u8, 16> m_bootParams = {};
	};
} // namespace Pif
