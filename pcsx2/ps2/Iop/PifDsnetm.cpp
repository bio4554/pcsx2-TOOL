// SPDX-FileCopyrightText: 2002-2026 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

#include "ps2/Iop/PifDsnetm.h"

#include <algorithm>
#include <cstring>

namespace Pif
{
	namespace
	{
		constexpr u16 PROTO_DCMP = 0x0001;
		constexpr u16 PROTO_DRFP2 = 0x0122;
		constexpr u16 PROTO_NETMP = 0x0400;
		constexpr u16 PROTO_DSNETM_CMD = 0x0410; // dsnetm's own command line
		constexpr u16 PROTO_COMPORT = 0x0412;
		constexpr u16 PROTO_NETDEV = 0x0510;

		constexpr u8 NODE_HOST = 'H';
		constexpr u8 NODE_IOP = 'I';
		constexpr u8 NODE_EE = 'E';

		enum : u8
		{
			NETMP_CONNECT = 0x00,
			NETMP_RESET = 0x02,
			NETMP_RESETR = 0x03,
			NETMP_MESSAGE = 0x04,
			NETMP_STATUS = 0x06,
			NETMP_KILL = 0x08,
			NETMP_VERSION = 0x0a,
			NETMP_POWEROFF = 0x0c,
			NETMP_COMPORT = 0x0e,
		};
		enum : u8
		{
			NETMP_OK = 0,
			NETMP_INVAL = 1,
			NETMP_BUSY = 2,
			NETMP_ALREADYCONN = 4,
			NETMP_NOPROTO = 6,
		};

		enum : u8
		{
			DCMP_CONNECT = 0,
			DCMP_ECHO = 1,
			DCMP_STATUS = 2,
			DCMP_ERROR = 3,
		};
		enum : u8
		{
			DCMP_CODE_CONNECT = 0,
			DCMP_CODE_CONNECTR = 1,
			DCMP_CODE_DISCONNECT = 2,
		};
		enum : u8
		{
			STATUS_CONNECTED = 0,
			STATUS_PROTO = 1,
			STATUS_ROUTE = 4,
		};
		enum : u8
		{
			ERR_NOROUTE = 0,
			ERR_NOPROTO = 1,
			ERR_NOCONNECT = 5,
		};

		// DRFP error for ENOENT (dsnetm's drfp_err: errno in both halves).
		constexpr u32 DRFP_ENOENT = 0x00020002;

		u16 Get16(const Dsnetm::Packet& p, size_t off)
		{
			return static_cast<u16>(p[off] | (p[off + 1] << 8));
		}

		u32 Get32(const Dsnetm::Packet& p, size_t off)
		{
			return p[off] | (p[off + 1] << 8) | (p[off + 2] << 16) | (static_cast<u32>(p[off + 3]) << 24);
		}

		void Put32(Dsnetm::Packet& p, u32 v)
		{
			for (int i = 0; i < 4; i++)
				p.push_back(static_cast<u8>(v >> (i * 8)));
		}

		u16 Proto(const Dsnetm::Packet& p)
		{
			return Get16(p, 4);
		}

		// A packet from dsnetm itself (source H).
		Dsnetm::Packet Make(u16 proto, u8 dst, const u8* body, size_t n)
		{
			Dsnetm::Packet p(8 + n);
			p[0] = static_cast<u8>((8 + n) & 0xff);
			p[1] = static_cast<u8>((8 + n) >> 8);
			p[4] = static_cast<u8>(proto & 0xff);
			p[5] = static_cast<u8>(proto >> 8);
			p[6] = NODE_HOST;
			p[7] = dst;
			if (n)
				std::memcpy(p.data() + 8, body, n);
			return p;
		}

		Dsnetm::Packet Make(u16 proto, u8 dst, const Dsnetm::Packet& body)
		{
			return Make(proto, dst, body.data(), body.size());
		}

		// xrecv_common: dsnetm drops packets too short for their protocol, from either side.
		bool WellFormed(const Dsnetm::Packet& p)
		{
			if (p.size() < 8)
				return false;
			const u16 proto = Proto(p);
			const u32 body = static_cast<u32>(p.size() - 8);
			if (Dsnetm::IsTtyp(proto))
				return true;
			if (proto >= 0x0120 && proto <= 0x0122) // DRFP0-2
			{
				if (body <= 3)
					return false;
				switch (Get32(p, 8))
				{
					case 0:
					case 2:
					case 3:
						return body > 11;
					case 1:
					case 4:
					case 7:
					case 9:
						return body > 15;
					case 5:
					case 6:
						return body > 15 && Get32(p, 20) >= body - 16;
					case 8:
						return body > 19;
					default:
						return true;
				}
			}
			if (proto == PROTO_DCMP)
			{
				if (body < 4)
					return false;
				const u32 rest = body - 4;
				switch (p[8])
				{
					case DCMP_CONNECT:
					case DCMP_ECHO:
						return rest > 3;
					case DCMP_STATUS:
						return rest > 1;
					case DCMP_ERROR:
						return rest > 0x17;
					default:
						return true;
				}
			}
			if (proto == 0x0130 || proto == 0x0140 || proto == 0x0230 || proto == 0x0240)
				return body >= 8;
			if (proto == PROTO_NETMP)
				return body >= 2;
			return true;
		}
	} // namespace

	Dsnetm::Dsnetm(Output& out)
		: m_out(out)
	{
	}

	void Dsnetm::SetConnectNodes(bool iop, bool ee)
	{
		m_connectIop = iop;
		m_connectEe = ee;
	}

	bool Dsnetm::IsTtyp(u16 proto)
	{
		return (proto >= 0x0110 && proto <= 0x0119) || proto == 0x011f || (proto >= 0x0210 && proto <= 0x0219) ||
		       proto == 0x021f || proto == 0x0412 || proto == 0x0413;
	}

	void Dsnetm::ClientOpened(int client, u64 now_ms)
	{
		m_clients.push_back(Client{client, now_ms});
	}

	void Dsnetm::ClientClosed(int client)
	{
		m_clients.erase(std::remove_if(m_clients.begin(), m_clients.end(), [client](const Client& c) { return c.id == client; }),
			m_clients.end());
		// dsnetm leaves these pointing at the closed client.
		m_bounced.erase(std::remove_if(m_bounced.begin(), m_bounced.end(), [client](const Bounced& b) { return b.owner == client; }),
			m_bounced.end());
	}

	Dsnetm::Client* Dsnetm::FindClient(int id)
	{
		for (Client& c : m_clients)
		{
			if (c.id == id)
				return &c;
		}
		return nullptr;
	}

	Dsnetm::Client* Dsnetm::Holder(u16 proto)
	{
		Client* best = nullptr;
		int best_prio = -1;
		for (Client& c : m_clients)
		{
			for (const auto& e : c.protos)
			{
				if ((e[2] | (e[3] << 8)) == proto && best_prio < e[0])
				{
					best = &c;
					best_prio = e[0];
				}
			}
		}
		return best;
	}

	bool Dsnetm::Busy(u8 prio, u16 proto) const
	{
		if (IsTtyp(proto))
			return false;
		for (const Client& c : m_clients)
		{
			for (const auto& e : c.protos)
			{
				if (e[0] == prio && (e[2] | (e[3] << 8)) == proto)
					return true;
			}
		}
		return false;
	}

	void Dsnetm::Send(int owner, Packet pkt)
	{
		if (owner == TARGET)
			m_out.ToTarget(std::move(pkt));
		else
			m_out.ToClient(owner, std::move(pkt));
	}

	void Dsnetm::Deliver(u16 proto, const Packet& pkt)
	{
		if (proto == 0)
		{
			for (const Client& c : m_clients)
			{
				if (c.registered)
					m_out.ToClient(c.id, pkt);
			}
			return;
		}
		if (IsTtyp(proto))
		{
			for (const Client& c : m_clients)
			{
				for (const auto& e : c.protos)
				{
					if ((e[2] | (e[3] << 8)) == proto)
					{
						m_out.ToClient(c.id, pkt);
						break;
					}
				}
			}
			return;
		}
		if (const Client* h = Holder(proto))
			m_out.ToClient(h->id, pkt);
	}

	void Dsnetm::Route(int owner, const Packet& pkt)
	{
		const u16 proto = Proto(pkt);
		// A TTYP packet with only its header and a first byte of 1 asks to drop what's still queued.
		if (((proto >= 0x0110 && proto <= 0x011f) || (proto >= 0x0210 && proto <= 0x021f)) && pkt.size() == 12 && pkt[8] == 1)
			m_out.FlushQueued(proto);
		if (proto == 0 || Holder(proto))
		{
			Deliver(proto, pkt);
			return;
		}
		const u8 node = pkt[6];
		if (std::none_of(m_bounced.begin(), m_bounced.end(), [&](const Bounced& b) { return b.proto == proto && b.node == node; }))
			m_bounced.push_back(Bounced{owner, proto, node});
		DcmpError(owner, pkt, ERR_NOPROTO);
	}

	void Dsnetm::DcmpError(int owner, const Packet& pkt, u8 code)
	{
		// Type 3, the code, then the packet's header and up to 16 bytes of its body.
		Packet body(28, 0);
		body[0] = DCMP_ERROR;
		body[1] = code;
		std::memcpy(body.data() + 4, pkt.data(), std::min<size_t>(pkt.size(), 24));
		Send(owner, Make(PROTO_DCMP, pkt[6], body));
	}

	Dsnetm::Packet Dsnetm::ConnectPacket(u8 node, const std::array<u8, 16>& boot_params)
	{
		// Type 0, code 0, result 0, then the boot parameters.
		Packet body(8, 0);
		body.insert(body.end(), boot_params.begin(), boot_params.end());
		return Make(PROTO_DCMP, node, body);
	}

	void Dsnetm::SendConnect(u8 node)
	{
		m_out.ToTarget(ConnectPacket(node, m_bootParams));
	}

	void Dsnetm::ForgetNodes()
	{
		m_iopConnected = m_eeConnected = false;
		m_iopStatusSent = m_eeStatusSent = false;
		m_bounced.clear();
	}

	void Dsnetm::TargetBooted(int cause, bool connects_sent)
	{
		ForgetNodes();
		if (cause >= 0)
		{
			const u8 body[3] = {NETMP_RESETR, NETMP_OK, static_cast<u8>(cause)};
			Deliver(0, Make(PROTO_NETMP, NODE_HOST, body, sizeof(body)));
		}
		if (connects_sent)
			return;
		if (m_connectIop)
			SendConnect(NODE_IOP);
		if (m_connectEe)
			SendConnect(NODE_EE);
	}

	void Dsnetm::FromClient(int client, Packet pkt, u64 now_ms)
	{
		Client* c = FindClient(client);
		if (!c || !WellFormed(pkt))
			return;
		const u16 proto = Proto(pkt);
		switch (pkt[7])
		{
			case NODE_IOP:
				if (proto != PROTO_COMPORT && m_iopConnected)
					m_out.ToTarget(std::move(pkt));
				else
					DcmpError(client, pkt, ERR_NOCONNECT);
				return;
			case NODE_EE:
				if (m_eeConnected)
					m_out.ToTarget(std::move(pkt));
				else
					DcmpError(client, pkt, ERR_NOCONNECT);
				return;
			default:
				break;
		}
		if (proto == PROTO_NETMP)
			Netmp(*c, pkt, now_ms);
		else if (proto != PROTO_DSNETM_CMD) // dsnetm's command line isn't emulated
			Route(client, pkt);
	}

	void Dsnetm::Netmp(Client& client, const Packet& pkt, u64 now_ms)
	{
		const int id = client.id;
		const size_t na = pkt.size() - 10; // after code and result
		// Most replies are the request with the code and result changed.
		Packet reply = Make(PROTO_NETMP, NODE_HOST, pkt.data() + 8, pkt.size() - 8);
		u8& code = reply[8];
		u8& result = reply[9];
		switch (pkt[8])
		{
			case NETMP_CONNECT:
			{
				code = NETMP_CONNECT + 1;
				if (client.registered)
				{
					result = NETMP_ALREADYCONN;
					break;
				}
				std::vector<std::array<u8, 4>> entries(na / 4);
				for (size_t i = 0; i < entries.size(); i++)
					std::memcpy(entries[i].data(), pkt.data() + 10 + i * 4, 4);
				result = NETMP_OK;
				for (const auto& e : entries)
				{
					if (Busy(e[0], static_cast<u16>(e[2] | (e[3] << 8))))
					{
						result = NETMP_BUSY;
						break;
					}
				}
				if (result != NETMP_OK)
					break;
				// Tell whoever had packets bounced for these protocols that they're there now.
				for (const auto& e : entries)
				{
					const u16 proto = static_cast<u16>(e[2] | (e[3] << 8));
					for (auto it = m_bounced.begin(); it != m_bounced.end();)
					{
						if (it->proto != proto)
						{
							++it;
							continue;
						}
						const u8 body[6] = {DCMP_STATUS, STATUS_PROTO, 0, 0, e[2], e[3]};
						Send(it->owner, Make(PROTO_DCMP, it->node, body, sizeof(body)));
						it = m_bounced.erase(it);
					}
				}
				client.registered = true;
				client.protos = std::move(entries);
				break;
			}

			case NETMP_RESET:
			{
				// No reply: the RESETR goes to every client once the target has booted.
				if (na == 34)
				{
					// EE and IOP boot parameters; all ones keeps the saved ones.
					const auto keep = [&](size_t off) { return Get32(pkt, off) == 0xffffffffu && Get32(pkt, off + 4) == 0xffffffffu; };
					if (!keep(12))
						std::memcpy(m_bootParams.data(), pkt.data() + 12, 8);
					if (!keep(20))
						std::memcpy(m_bootParams.data() + 8, pkt.data() + 20, 8);
				}
				m_out.ResetTarget(m_bootParams);
				ForgetNodes();
				return;
			}

			case NETMP_MESSAGE:
				code = NETMP_MESSAGE + 1;
				client.message.assign(reinterpret_cast<const char*>(pkt.data() + 10), na);
				client.message.resize(std::strlen(client.message.c_str()));
				client.hasMessage = true;
				result = NETMP_OK;
				break;

			case NETMP_STATUS:
				m_out.ToClient(id, StatusReply(now_ms));
				return;

			case NETMP_KILL:
			{
				code = NETMP_KILL + 1;
				Client* holder = Holder(na >= 2 ? Get16(pkt, 10) : 0);
				if (!holder)
				{
					result = NETMP_NOPROTO;
					break;
				}
				const int victim = holder->id;
				ClientClosed(victim);
				m_out.CloseClient(victim);
				if (victim == id)
					return;
				result = NETMP_OK;
				break;
			}

			case NETMP_VERSION:
			{
				Packet body = {NETMP_VERSION + 1, NETMP_OK};
				body.insert(body.end(), VERSION, VERSION + std::strlen(VERSION));
				m_out.ToClient(id, Make(PROTO_NETMP, NODE_HOST, body));
				return;
			}

			case NETMP_POWEROFF:
				// The TOOL powers its PS2 side off; the emulator doesn't.
				code = NETMP_POWEROFF + 1;
				result = NETMP_INVAL;
				break;

			case NETMP_COMPORT:
				// No serial port behind this dsnetm.
				code = NETMP_COMPORT + 1;
				result = NETMP_INVAL;
				break;

			default:
				result = NETMP_INVAL;
				break;
		}
		m_out.ToClient(id, std::move(reply));
	}

	Dsnetm::Packet Dsnetm::StatusReply(u64 now_ms) const
	{
		// Per protocol of each client: the CONNECT entry, seconds since the client
		// connected, and its owner string, NUL-terminated and padded to a word.
		// dsnetm sizes the reply one byte short for each entry of a client that sent
		// no MESSAGE (it leaves out the NUL), so the reply loses that many bytes at
		// its end. Kept: the TOOL's reply is the reference.
		Packet body = {NETMP_STATUS + 1, NETMP_OK, 0, 0};
		size_t sized = body.size();
		for (const Client& c : m_clients)
		{
			for (const auto& e : c.protos)
			{
				body.insert(body.end(), e.begin(), e.end());
				Put32(body, static_cast<u32>((now_ms - c.opened_ms) / 1000));
				body.insert(body.end(), c.message.begin(), c.message.end());
				const size_t field = (c.message.size() + 1 + 3) & ~size_t{3};
				body.insert(body.end(), field - c.message.size(), 0);
				sized += 8 + (c.hasMessage ? field : 3);
			}
		}
		body.resize(sized);
		return Make(PROTO_NETMP, NODE_HOST, body);
	}

	void Dsnetm::FromTarget(Packet pkt)
	{
		if (!WellFormed(pkt))
			return;
		const u16 proto = Proto(pkt);
		if (proto == PROTO_DRFP2)
		{
			Drfp2(pkt);
			return;
		}
		if (proto == PROTO_NETMP)
		{
			if (pkt[8] == NETMP_RESETR)
				Deliver(0, pkt);
			return;
		}
		if (proto == PROTO_NETDEV)
			return;
		if (proto != PROTO_DCMP)
		{
			Route(TARGET, pkt);
			return;
		}

		switch (pkt[8])
		{
			case DCMP_ECHO:
				Route(TARGET, pkt);
				break;
			case DCMP_CONNECT:
				DcmpConnect(pkt);
				break;
			case DCMP_STATUS:
				if (pkt[9] == STATUS_ROUTE)
				{
					// "Node up": connect to the nodes that aren't connected yet.
					if (!m_iopConnected && m_connectIop)
						SendConnect(NODE_IOP);
					if (!m_eeConnected && m_connectEe)
						SendConnect(NODE_EE);
				}
				else if (pkt[9] != STATUS_CONNECTED)
				{
					Deliver(0, pkt);
				}
				break;
			case DCMP_ERROR:
				DcmpErrorFromTarget(pkt);
				break;
			default:
				break;
		}
	}

	void Dsnetm::DcmpConnect(const Packet& pkt)
	{
		const u8 code = pkt[9];
		const u8 result = pkt[12];
		if (result != 0 && !(code == DCMP_CODE_CONNECTR && result == 2)) // 2: already connected
			return;
		const u8 node = pkt[6];
		if (code == DCMP_CODE_CONNECT)
		{
			// The target asks: answer CONNECTR with no boot parameters.
			const u8 body[8] = {DCMP_CONNECT, DCMP_CODE_CONNECTR, 0, 0, 0, 0, 0, 0};
			m_out.ToTarget(Make(PROTO_DCMP, node, body, sizeof(body)));
		}
		if (node != NODE_IOP && node != NODE_EE)
			return;
		bool& connected = (node == NODE_IOP) ? m_iopConnected : m_eeConnected;
		bool& status_sent = (node == NODE_IOP) ? m_iopStatusSent : m_eeStatusSent;

		if (code == DCMP_CODE_DISCONNECT)
		{
			connected = false;
			status_sent = false;
			m_bounced.erase(std::remove_if(m_bounced.begin(), m_bounced.end(), [node](const Bounced& b) { return b.node == node; }),
				m_bounced.end());
			return;
		}
		else if (code != DCMP_CODE_CONNECT && code != DCMP_CODE_CONNECTR)
		{
			return;
		}

		connected = true;
		if (!status_sent)
		{
			status_sent = true;
			const u8 body[6] = {DCMP_STATUS, STATUS_CONNECTED, 0, 0, node, 0};
			Deliver(0, Make(PROTO_DCMP, NODE_HOST, body, sizeof(body)));
		}
	}

	void Dsnetm::DcmpErrorFromTarget(const Packet& pkt)
	{
		// Only errors about a client's packet, passed to the holder of its protocol.
		if (pkt.size() != 36 || pkt[18] != NODE_HOST)
			return;
		const u16 proto = Get16(pkt, 16);
		if (!proto || !Holder(proto))
			return;
		if (pkt[9] == ERR_NOCONNECT)
			return;
		Packet copy = pkt;
		if (copy[9] == ERR_NOROUTE)
			copy[9] = ERR_NOCONNECT;
		Deliver(proto, copy);
	}

	void Dsnetm::Drfp2(const Packet& pkt)
	{
		// dsnetm serves DRFP2 itself from the Linux board's files (${IOPCONF},
		// ${IOPMODULES}: /usr/local/sce/iop/modules). The emulator has none of them,
		// so every open fails with ENOENT; with no handle ever opened, nothing else
		// is answered.
		if (Get32(pkt, 8) != 0)
			return;
		Packet body;
		Put32(body, 1);
		Put32(body, Get32(pkt, 12)); // the request's sequence number
		Put32(body, DRFP_ENOENT);
		Put32(body, 0xffffffffu);
		m_out.ToTarget(Make(PROTO_DRFP2, pkt[6], body));
	}
} // namespace Pif
