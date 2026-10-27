// SPDX-FileCopyrightText: 2002-2026 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

#include "Common.h"
#include "Host.h"
#include "VMManager.h"
#include "R3000A.h"
#include "R5900.h"
#include "Memory.h"
#include "Hw.h"
#include "IopHw.h"
#include "IopMem.h"
#include "ps2/Iop/Pif.h"
#include "ps2/Iop/PifDsnetm.h"
#include "ps2/Iop/PifEmup.h"
#include "SIO/Pad/PadScript.h"
#include "common/Threading.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#ifdef _WIN32
#include "common/RedtapeWindows.h"
#include <WinSock2.h>
#include <WS2tcpip.h>
using socket_t = SOCKET;
static constexpr socket_t BAD_SOCKET = INVALID_SOCKET;
#define close_socket closesocket
#define SHUT_BOTH_DIRS SD_BOTH
#else
#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
using socket_t = int;
static constexpr socket_t BAD_SOCKET = -1;
#define close_socket close
#define SHUT_BOTH_DIRS SHUT_RDWR
#endif

// The bridge plays what dsnetm sees of the TOOL's Linux board: a TCP stream of
// raw DECI2 packets (dsnetm -d <host>:<port>). Packets go to and from the PIF
// card, except NETMP reset, which the Linux side handles itself: it resets the
// target and answers NETMP RESETR (code 3) once the target has booted, which
// for us is DECI2DRP's boot handshake.
// A reboot the target does by itself (an IOP reboot, e.g. sceSifRebootIop) also
// ends in that handshake. On the TOOL, mrp.o then fails dsnetm's next read with
// EPIPE: dsnetm drops the link (recv_target with no packet: NETMP RESETR to its
// clients), and its idle loop reconnects (connect_dev), which sends the DCMP
// connects DECI2HSYN waits for. Closing the TCP link gets the same from dsnetm.
//
// In dsnetm mode the bridge is dsnetm itself (PifDsnetm.cpp): it listens for the
// clients (dekko, dsedb, ProDG's Target Manager), as the TOOL does on 8510, and
// handles resets and reboots directly. Its thread runs the sockets and the
// routing; the CPU thread hands it the target's packets and boots.
namespace Pif::Bridge
{
	static constexpr u16 PROTO_NETMP = 0x0400;
	static constexpr u8 NETMP_CODE_RESET = 0x02;
	static constexpr u8 NETMP_CODE_RESETR = 0x03;
	static constexpr u8 NETMP_CAUSE_RESET = 0x06;

	static bool s_enabled;
	static bool s_listenMode;
	// The built-in dsnetm instead of a link to an external one.
	static bool s_dsnetmMode;
	static std::string s_hostName;
	static u16 s_port;

	static std::thread s_thread;
	static std::thread s_watchdog;
	static std::atomic<bool> s_running{false};

	// Guards the sockets. Sending happens on the CPU thread, receiving on the bridge thread.
	static std::mutex s_sockMutex;
	static socket_t s_listenSock = BAD_SOCKET;
	static socket_t s_clientSock = BAD_SOCKET;

	// Host -> PS2 packets, drained on the CPU thread.
	static std::mutex s_inMutex;
	static std::deque<std::vector<u8>> s_inbound;
	static std::atomic<bool> s_inboundPending{false};

	// Set by a NETMP reset: hold host packets until the VM has actually reset,
	// so dsnetm's DCMP connects reach the new boot, not the old one.
	static std::atomic<bool> s_hold{false};
	static std::atomic<bool> s_resetReplyOwed{false};
	// The target has booted since the last VM reset, so another boot handshake is a reboot.
	static std::atomic<bool> s_targetBooted{false};

	static bool ParseConfig()
	{
		const char* env = std::getenv("PCSX2_PIF_BRIDGE");
		if (!env || !env[0])
			return false;

		std::string cfg(env);
		s_listenMode = true;
		s_dsnetmMode = false;
		s_hostName = "127.0.0.1";
		if (cfg.rfind("connect:", 0) == 0)
		{
			s_listenMode = false;
			cfg = cfg.substr(8);
		}
		else if (cfg.rfind("listen:", 0) == 0)
		{
			cfg = cfg.substr(7);
		}
		else if (cfg.rfind("dsnetm:", 0) == 0)
		{
			s_dsnetmMode = true;
			cfg = cfg.substr(7);
		}

		const size_t colon = cfg.rfind(':');
		if (colon != std::string::npos)
		{
			s_hostName = cfg.substr(0, colon);
			cfg = cfg.substr(colon + 1);
		}
		else if (!s_listenMode)
		{
			Console.Error("PIF: PCSX2_PIF_BRIDGE=connect: needs host:port");
			return false;
		}

		const int port = std::atoi(cfg.c_str());
		if (port <= 0 || port > 65535)
		{
			Console.Error("PIF: bad port in PCSX2_PIF_BRIDGE");
			return false;
		}
		s_port = static_cast<u16>(port);
		return true;
	}

	static bool ResolveAddress(sockaddr_in* addr)
	{
		std::memset(addr, 0, sizeof(*addr));
		addr->sin_family = AF_INET;
		addr->sin_port = htons(s_port);
		if (inet_pton(AF_INET, s_hostName.c_str(), &addr->sin_addr) == 1)
			return true;

		addrinfo hints = {};
		hints.ai_family = AF_INET;
		hints.ai_socktype = SOCK_STREAM;
		addrinfo* res = nullptr;
		if (getaddrinfo(s_hostName.c_str(), nullptr, &hints, &res) != 0 || !res)
			return false;
		addr->sin_addr = reinterpret_cast<sockaddr_in*>(res->ai_addr)->sin_addr;
		freeaddrinfo(res);
		return true;
	}

	static void SetNoDelay(socket_t sock)
	{
		int one = 1;
		setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&one), sizeof(one));
	}

	static socket_t WaitForClient()
	{
		if (s_listenMode)
			return accept(s_listenSock, nullptr, nullptr);

		sockaddr_in addr;
		if (!ResolveAddress(&addr))
			return BAD_SOCKET;
		socket_t sock = socket(AF_INET, SOCK_STREAM, 0);
		if (sock == BAD_SOCKET)
			return BAD_SOCKET;
		if (connect(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0)
		{
			close_socket(sock);
			return BAD_SOCKET;
		}
		return sock;
	}

	static void ClearInbound()
	{
		std::lock_guard lock(s_inMutex);
		s_inbound.clear();
		s_inboundPending.store(false, std::memory_order_release);
	}

	static void QueueToTarget(std::vector<u8> pkt)
	{
		std::lock_guard lock(s_inMutex);
		s_inbound.push_back(std::move(pkt));
		s_inboundPending.store(true, std::memory_order_release);
	}

	// NETMP reset: reset the VM, holding host packets until it has.
	static void ResetTarget()
	{
		Console.WriteLn("PIF: NETMP reset from the host, resetting the VM");
		ClearInbound();
		s_hold.store(true, std::memory_order_release);
		s_resetReplyOwed.store(true, std::memory_order_release);
		Host::RunOnCPUThread([]() {
			if (VMManager::HasValidVM())
				VMManager::Reset();
		});
	}

	// Bridge thread: one DECI2 packet from the host.
	static void HandleHostPacket(std::vector<u8> pkt)
	{
		const u16 proto = pkt[4] | (pkt[5] << 8);
		if (proto == PROTO_NETMP && pkt.size() >= 10 && pkt[8] == NETMP_CODE_RESET)
		{
			// The boot parameters in the body are dsnetm's business: it puts them
			// in the DCMP connects that follow.
			ResetTarget();
			return;
		}
		QueueToTarget(std::move(pkt));
	}

	// ---- dsnetm mode ----

	// A dsnetm client: its socket, the bytes of a packet still arriving, and the
	// packets waiting to go out. dsnetm doesn't limit that queue, except for TTY
	// output (TTY_MAX_SIZE).
	struct DsnetmClient
	{
		int id;
		socket_t sock;
		u16 port; // the client's, for the capture
		std::vector<u8> rx;
		std::deque<std::vector<u8>> queue;
		std::vector<u8> sending;
		size_t sent = 0;
		size_t ttyBytes = 0;
		bool closed = false;
	};

	// dsnetm's tty_max_size option (default 8 MB): a client with more TTY output
	// queued than this loses its whole queue, replaced by a note.
	static constexpr size_t TTY_MAX_SIZE = 0x800000;

	// Manager thread only.
	static std::unique_ptr<Dsnetm> s_dsnetm;
	static std::vector<std::unique_ptr<DsnetmClient>> s_dsnetmClients;
	static int s_nextClientId;

	// From the CPU thread: the target's packets and boots, in order.
	struct TargetEvent
	{
		std::vector<u8> pkt; // empty for a boot
		int cause;
		std::vector<std::vector<u8>> connects; // a boot's DCMP connects, already in the card
	};

	// The boot parameters for the next boot's connects (from NETMP RESET), which the CPU thread
	// sends itself (OnHandshake).
	static std::mutex s_bootParamsMutex;
	static std::array<u8, 16> s_bootParams;
	static std::mutex s_eventMutex;
	static std::deque<TargetEvent> s_events;
	// EMUP replies the CPU thread made, for these clients (s_eventMutex).
	static std::deque<std::pair<int, std::vector<u8>>> s_emupReplies;

	// A UDP socket connected to itself, so the CPU thread can wake select().
	static socket_t s_wakeSock = BAD_SOCKET;
	static std::atomic<bool> s_wakePending{false};

	static bool WouldBlock()
	{
#ifdef _WIN32
		return WSAGetLastError() == WSAEWOULDBLOCK;
#else
		return errno == EAGAIN || errno == EWOULDBLOCK;
#endif
	}

	static void SetNonBlocking(socket_t sock)
	{
#ifdef _WIN32
		u_long one = 1;
		ioctlsocket(sock, FIONBIO, &one);
#else
		fcntl(sock, F_SETFL, fcntl(sock, F_GETFL, 0) | O_NONBLOCK);
#endif
	}

	static u64 NowMs()
	{
		return static_cast<u64>(std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::steady_clock::now().time_since_epoch())
				.count());
	}

	static void Wake()
	{
		if (!s_wakePending.exchange(true, std::memory_order_acq_rel))
		{
			const char b = 0;
			// A byte that didn't go leaves the next wake to try again.
			if (send(s_wakeSock, &b, 1, 0) != 1)
				s_wakePending.store(false, std::memory_order_release);
		}
	}

	static void PostTargetEvent(TargetEvent ev)
	{
		{
			std::lock_guard lock(s_eventMutex);
			s_events.push_back(std::move(ev));
		}
		Wake();
	}

	static void PostEmupReply(int client, std::vector<u8> pkt)
	{
		{
			std::lock_guard lock(s_eventMutex);
			s_emupReplies.emplace_back(client, std::move(pkt));
		}
		Wake();
	}

	// CPU thread: an EMUP request about the VM. Runs from the CPU thread's queue,
	// which a running VM pumps at each vsync (VMManager::Internal::PollInputOnCPUThread).
	static std::vector<u8> AnswerOnCpu(const Emup::Request& req)
	{
		if (!VMManager::HasValidVM())
			return Emup::Reply(req, Emup::RESULT_NOVM);
		switch (req.code)
		{
			case Emup::CODE_PAD_STATUS:
			{
				const std::optional<Emup::PadRequest> pr = Emup::ParsePad(req);
				if (!pr)
					return Emup::Reply(req, Emup::RESULT_INVAL);
				return Emup::Reply(req, Emup::RESULT_OK, Emup::PadStatusPayload(PadScript::Status(pr->port, pr->slot)));
			}
			case Emup::CODE_PAD_SET:
			{
				const std::optional<Emup::PadSet> set = Emup::ParsePadSet(req);
				if (!set)
					return Emup::Reply(req, Emup::RESULT_INVAL);
				const Emup::Result r = PadScript::Set(set->pad.port, set->pad.slot, set->data);
				if (r != Emup::RESULT_OK)
					return Emup::Reply(req, r);
				return Emup::Reply(req, Emup::RESULT_OK, Emup::PadStatusPayload(PadScript::Status(set->pad.port, set->pad.slot)));
			}
			case Emup::CODE_PAD_DETACH:
			{
				const std::optional<Emup::PadRequest> pr = Emup::ParsePad(req);
				if (!pr)
					return Emup::Reply(req, Emup::RESULT_INVAL);
				PadScript::Detach(pr->port, pr->slot);
				return Emup::Reply(req, Emup::RESULT_OK, Emup::PadStatusPayload(PadScript::Status(pr->port, pr->slot)));
			}
			case Emup::CODE_PAD_PLAY:
			{
				const std::optional<Emup::PadPlayPart> part = Emup::ParsePadPlay(req);
				if (!part)
					return Emup::Reply(req, Emup::RESULT_INVAL);
				const Emup::Result r = PadScript::Play(*part);
				if (r != Emup::RESULT_OK)
					return Emup::Reply(req, r);
				return Emup::Reply(req, Emup::RESULT_OK, Emup::PadStatusPayload(PadScript::Status(part->pad.port, part->pad.slot)));
			}
			case Emup::CODE_PAD_STOP:
			{
				const std::optional<Emup::PadRequest> pr = Emup::ParsePad(req);
				if (!pr)
					return Emup::Reply(req, Emup::RESULT_INVAL);
				PadScript::Stop(pr->port, pr->slot);
				return Emup::Reply(req, Emup::RESULT_OK, Emup::PadStatusPayload(PadScript::Status(pr->port, pr->slot)));
			}
			case Emup::CODE_PAD_PLUG:
			{
				const std::optional<Emup::PadPlug> plug = Emup::ParsePadPlug(req);
				if (!plug)
					return Emup::Reply(req, Emup::RESULT_INVAL);
				const Emup::Result r = PadScript::Plug(plug->pad.port, plug->pad.slot, plug->plugged);
				if (r != Emup::RESULT_OK)
					return Emup::Reply(req, r);
				return Emup::Reply(req, Emup::RESULT_OK, Emup::PadStatusPayload(PadScript::Status(plug->pad.port, plug->pad.slot)));
			}
			case Emup::CODE_PAD_ANALOG:
			{
				const std::optional<Emup::PadRequest> pr = Emup::ParsePad(req);
				if (!pr)
					return Emup::Reply(req, Emup::RESULT_INVAL);
				const Emup::Result r = PadScript::Analog(pr->port, pr->slot);
				if (r != Emup::RESULT_OK)
					return Emup::Reply(req, r);
				return Emup::Reply(req, Emup::RESULT_OK, Emup::PadStatusPayload(PadScript::Status(pr->port, pr->slot)));
			}
			case Emup::CODE_PAD_TYPE:
			{
				const std::optional<Emup::PadTypeRequest> type = Emup::ParsePadType(req);
				if (!type)
					return Emup::Reply(req, Emup::RESULT_INVAL);
				const Emup::Result r = PadScript::Type(type->pad.port, type->pad.slot, static_cast<Pad::DualShockModel>(type->model));
				if (r != Emup::RESULT_OK)
					return Emup::Reply(req, r);
				return Emup::Reply(req, Emup::RESULT_OK, Emup::PadStatusPayload(PadScript::Status(type->pad.port, type->pad.slot)));
			}
			default:
				return Emup::Reply(req, Emup::RESULT_NOCODE);
		}
	}

	static DsnetmClient* FindDsnetmClient(int id)
	{
		for (const auto& c : s_dsnetmClients)
		{
			if (c->id == id && !c->closed)
				return c.get();
		}
		return nullptr;
	}

	static u16 PacketProto(const std::vector<u8>& pkt)
	{
		return pkt[4] | (pkt[5] << 8);
	}

	// PCSX2_PIF_PCAP=<file> (dsnetm mode): every packet dsnetm passes, as a capture in tcpdump's
	// format (Ethernet, IPv4, TCP, all on 127.0.0.1) so the WSL harness's tools and Wireshark read it:
	// each client's stream from its own port to the listen port, the target's link from port 8600 (the
	// relay's, in the WSL setup) to 8599. Checksums are left at 0. Manager thread only.
	class Capture
	{
	public:
		static constexpr u16 TARGET_PORT = 8600;
		static constexpr u16 DSNETM_PORT = 8599;

		bool Open(const char* path)
		{
			m_file = std::fopen(path, "wb");
			if (!m_file)
				return false;
			u8 header[24] = {};
			Put(header, 0, 0xa1b2c3d4u, 4); // microsecond timestamps
			Put(header, 4, 2, 2);
			Put(header, 6, 4, 2);
			Put(header, 16, 262144, 4); // snaplen
			Put(header, 20, 1, 4); // Ethernet
			std::fwrite(header, 1, sizeof(header), m_file);
			std::fflush(m_file);
			return true;
		}

		void Close()
		{
			if (m_file)
				std::fclose(m_file);
			m_file = nullptr;
			m_seq.clear();
		}

		void Connected(u16 client_port, u16 listen_port) { Frame(client_port, listen_port, 0x02, nullptr, 0); }
		void Closed(u16 client_port, u16 listen_port) { Frame(client_port, listen_port, 0x11, nullptr, 0); }

		void Data(u16 from, u16 to, const std::vector<u8>& pkt)
		{
			// An IPv4 packet holds at most 65,495 bytes of TCP payload.
			for (size_t at = 0; at < pkt.size(); at += 65000)
				Frame(from, to, 0x18, pkt.data() + at, std::min<size_t>(65000, pkt.size() - at));
		}

	private:
		static void Put(u8* p, size_t off, u32 v, int bytes, bool big = false)
		{
			for (int i = 0; i < bytes; i++)
				p[off + i] = static_cast<u8>(v >> ((big ? bytes - 1 - i : i) * 8));
		}

		void Frame(u16 from, u16 to, u8 flags, const u8* data, size_t n)
		{
			if (!m_file)
				return;
			u32& seq = m_seq[(static_cast<u32>(from) << 16) | to];
			u8 h[16 + 14 + 20 + 20] = {};
			const auto now = std::chrono::duration_cast<std::chrono::microseconds>(
				std::chrono::system_clock::now().time_since_epoch())
			                     .count();
			const u32 frame = static_cast<u32>(14 + 20 + 20 + n);
			Put(h, 0, static_cast<u32>(now / 1000000), 4);
			Put(h, 4, static_cast<u32>(now % 1000000), 4);
			Put(h, 8, frame, 4);
			Put(h, 12, frame, 4);
			u8* eth = h + 16;
			Put(eth, 12, 0x0800, 2, true);
			u8* ip = eth + 14;
			ip[0] = 0x45;
			Put(ip, 2, static_cast<u32>(20 + 20 + n), 2, true);
			Put(ip, 6, 0x4000, 2, true); // don't fragment
			ip[8] = 64;
			ip[9] = 6; // TCP
			Put(ip, 12, 0x7f000001, 4, true);
			Put(ip, 16, 0x7f000001, 4, true);
			u8* tcp = ip + 20;
			Put(tcp, 0, from, 2, true);
			Put(tcp, 2, to, 2, true);
			Put(tcp, 4, seq, 4, true);
			tcp[12] = 5 << 4;
			tcp[13] = flags;
			Put(tcp, 14, 0xffff, 2, true);
			std::fwrite(h, 1, sizeof(h), m_file);
			if (n)
				std::fwrite(data, 1, n, m_file);
			std::fflush(m_file);
			seq += static_cast<u32>(n) + ((flags & 0x03) ? 1 : 0); // SYN and FIN take a number
		}

		std::FILE* m_file = nullptr;
		std::unordered_map<u32, u32> m_seq;
	};
	static Capture s_capture;

	class DsnetmOutput final : public Dsnetm::Output
	{
	public:
		void ToClient(int client, Dsnetm::Packet pkt) override
		{
			DsnetmClient* c = FindDsnetmClient(client);
			if (!c)
				return;
			if (Dsnetm::IsTtyp(PacketProto(pkt)))
			{
				if (c->ttyBytes > TTY_MAX_SIZE)
					OverflowTty(*c, PacketProto(pkt));
				c->ttyBytes += pkt.size();
			}
			s_capture.Data(s_port, c->port, pkt);
			c->queue.push_back(std::move(pkt));
		}

		void ToTarget(Dsnetm::Packet pkt) override
		{
			s_capture.Data(Capture::DSNETM_PORT, Capture::TARGET_PORT, pkt);
			QueueToTarget(std::move(pkt));
		}

		void CloseClient(int client) override
		{
			if (DsnetmClient* c = FindDsnetmClient(client))
			{
				Console.WriteLn("PIF: dsnetm client %d killed (NETMP KILL)", client);
				c->closed = true;
			}
		}

		void ResetTarget(const std::array<u8, 16>& boot_params) override
		{
			{
				std::lock_guard lock(s_bootParamsMutex);
				s_bootParams = boot_params;
			}
			Bridge::ResetTarget();
		}

		void Emulator(int client, Dsnetm::Packet pkt) override
		{
			std::optional<Emup::Request> req = Emup::Parse(pkt);
			if (!req)
			{
				ToClient(client, Emup::BadReply());
				return;
			}
			switch (req->code)
			{
				case Emup::CODE_VERSION:
					ToClient(client, Emup::Reply(*req, Emup::RESULT_OK, Emup::VersionPayload()));
					return;
				case Emup::CODE_PAD_STATUS:
				case Emup::CODE_PAD_SET:
				case Emup::CODE_PAD_DETACH:
				case Emup::CODE_PAD_PLAY:
				case Emup::CODE_PAD_STOP:
				case Emup::CODE_PAD_PLUG:
				case Emup::CODE_PAD_ANALOG:
				case Emup::CODE_PAD_TYPE:
					Host::RunOnCPUThread([client, req = std::move(*req)]() { PostEmupReply(client, AnswerOnCpu(req)); });
					return;
				default:
					ToClient(client, Emup::Reply(*req, Emup::RESULT_NOCODE));
					return;
			}
		}

		void FlushQueued(u16 proto) override
		{
			for (const auto& c : s_dsnetmClients)
			{
				c->queue.erase(std::remove_if(c->queue.begin(), c->queue.end(),
								   [proto](const std::vector<u8>& p) { return PacketProto(p) == proto; }),
					c->queue.end());
				if (Dsnetm::IsTtyp(proto))
					c->ttyBytes = 0;
			}
		}

	private:
		// ds_flush_tty: drop everything queued (not the packet being sent) for a
		// note on this TTY protocol, addressed like the last packet dropped.
		static void OverflowTty(DsnetmClient& c, u16 proto)
		{
			size_t skipped = 0;
			u8 dst = 'H';
			for (const std::vector<u8>& p : c.queue)
			{
				skipped += p.size();
				dst = p[7];
			}
			c.queue.clear();
			const std::string note = fmt::format("\n\n*** TTY buffer overflow. Skip {} bytes data ***\n\n", skipped);
			std::vector<u8> pkt(12 + note.size(), 0);
			pkt[0] = static_cast<u8>(pkt.size() & 0xff);
			pkt[1] = static_cast<u8>(pkt.size() >> 8);
			pkt[4] = static_cast<u8>(proto & 0xff);
			pkt[5] = static_cast<u8>(proto >> 8);
			pkt[6] = 'H';
			pkt[7] = dst;
			std::memcpy(pkt.data() + 12, note.data(), note.size());
			c.queue.push_back(std::move(pkt));
			c.ttyBytes = note.size();
		}
	};
	static DsnetmOutput s_dsnetmOutput;

	static void AcceptDsnetmClient()
	{
		sockaddr_in addr = {};
		socklen_t len = sizeof(addr);
		const socket_t sock = accept(s_listenSock, reinterpret_cast<sockaddr*>(&addr), &len);
		if (sock == BAD_SOCKET)
			return;
		// select() can't take many more on Windows (FD_SETSIZE 64).
		if (s_dsnetmClients.size() >= 60)
		{
			Console.Warning("PIF: dsnetm has too many clients, refusing one");
			close_socket(sock);
			return;
		}
		SetNoDelay(sock);
		SetNonBlocking(sock);
		auto c = std::make_unique<DsnetmClient>();
		c->id = s_nextClientId++;
		c->sock = sock;
		c->port = ntohs(addr.sin_port);
		s_capture.Connected(c->port, s_port);
		char ip[INET_ADDRSTRLEN] = {};
		inet_ntop(AF_INET, &addr.sin_addr, ip, sizeof(ip));
		Console.WriteLn("PIF: dsnetm client %d connected from %s:%u", c->id, ip, ntohs(addr.sin_port));
		s_dsnetm->ClientOpened(c->id, NowMs());
		s_dsnetmClients.push_back(std::move(c));
	}

	static void ReceiveFromClient(DsnetmClient& c)
	{
		char tmp[16384];
		for (;;)
		{
			const int n = recv(c.sock, tmp, sizeof(tmp), 0);
			if (n < 0 && WouldBlock())
				break;
			if (n <= 0)
			{
				c.closed = true;
				return;
			}
			c.rx.insert(c.rx.end(), tmp, tmp + n);
		}

		size_t pos = 0;
		while (!c.closed && c.rx.size() - pos >= 8)
		{
			const u16 len = c.rx[pos] | (c.rx[pos + 1] << 8);
			if (len < 8)
			{
				Console.Error("PIF: dsnetm client %d sent a bad packet length %u, closing it", c.id, len);
				c.closed = true;
				return;
			}
			if (c.rx.size() - pos < len)
				break;
			std::vector<u8> pkt(c.rx.begin() + pos, c.rx.begin() + pos + len);
			s_capture.Data(c.port, s_port, pkt);
			s_dsnetm->FromClient(c.id, std::move(pkt), NowMs());
			pos += len;
		}
		c.rx.erase(c.rx.begin(), c.rx.begin() + pos);
	}

	static void SendToClient(DsnetmClient& c)
	{
		while (!c.closed)
		{
			if (c.sent == c.sending.size())
			{
				if (c.queue.empty())
					return;
				c.sending = std::move(c.queue.front());
				c.queue.pop_front();
				c.sent = 0;
				if (Dsnetm::IsTtyp(PacketProto(c.sending)))
					c.ttyBytes -= std::min(c.ttyBytes, c.sending.size());
			}
			const int n = send(c.sock, reinterpret_cast<const char*>(c.sending.data() + c.sent),
				static_cast<int>(c.sending.size() - c.sent), 0);
			if (n < 0 && WouldBlock())
				return;
			if (n <= 0)
			{
				c.closed = true;
				return;
			}
			c.sent += n;
		}
	}

	static void DsnetmMain()
	{
		Threading::SetNameOfCurrentThread("PIF dsnetm");

		while (s_running.load(std::memory_order_acquire))
		{
			fd_set rfds, wfds;
			FD_ZERO(&rfds);
			FD_ZERO(&wfds);
			socket_t maxfd = std::max(s_listenSock, s_wakeSock);
			FD_SET(s_listenSock, &rfds);
			FD_SET(s_wakeSock, &rfds);
			for (const auto& c : s_dsnetmClients)
			{
				FD_SET(c->sock, &rfds);
				if (c->sent < c->sending.size() || !c->queue.empty())
					FD_SET(c->sock, &wfds);
				maxfd = std::max(maxfd, c->sock);
			}
			timeval tv = {1, 0};
			if (select(static_cast<int>(maxfd + 1), &rfds, &wfds, nullptr, &tv) < 0)
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
				continue;
			}

			if (FD_ISSET(s_wakeSock, &rfds))
			{
				char tmp[64];
				while (recv(s_wakeSock, tmp, sizeof(tmp), 0) > 0)
				{
				}
			}
			// Only after draining (crusade TR-31): a wake from here on sends a byte the next select()
			// sees, and an event posted before it is in the swap below. Clearing first let the drain
			// eat a wake's byte with the flag left set, and no wake sent anything after that: every
			// target packet then waited out select()'s 1 s timeout.
			s_wakePending.store(false, std::memory_order_seq_cst);
			std::deque<TargetEvent> events;
			std::deque<std::pair<int, std::vector<u8>>> emupReplies;
			{
				std::lock_guard lock(s_eventMutex);
				events.swap(s_events);
				emupReplies.swap(s_emupReplies);
			}
			for (auto& [client, pkt] : emupReplies)
				s_dsnetmOutput.ToClient(client, std::move(pkt));
			for (TargetEvent& ev : events)
			{
				if (ev.pkt.empty())
				{
					for (const std::vector<u8>& c : ev.connects)
						s_capture.Data(Capture::DSNETM_PORT, Capture::TARGET_PORT, c);
					s_dsnetm->TargetBooted(ev.cause, true);
				}
				else
				{
					s_capture.Data(Capture::TARGET_PORT, Capture::DSNETM_PORT, ev.pkt);
					s_dsnetm->FromTarget(std::move(ev.pkt));
				}
			}

			if (FD_ISSET(s_listenSock, &rfds))
				AcceptDsnetmClient();
			for (size_t i = 0; i < s_dsnetmClients.size(); i++)
			{
				DsnetmClient& c = *s_dsnetmClients[i];
				if (!c.closed && FD_ISSET(c.sock, &rfds))
					ReceiveFromClient(c);
			}
			// Send what's queued now rather than after the next select().
			for (const auto& c : s_dsnetmClients)
				SendToClient(*c);

			for (auto it = s_dsnetmClients.begin(); it != s_dsnetmClients.end();)
			{
				if (!(*it)->closed)
				{
					++it;
					continue;
				}
				Console.WriteLn("PIF: dsnetm client %d disconnected", (*it)->id);
				close_socket((*it)->sock);
				s_capture.Closed((*it)->port, s_port);
				s_dsnetm->ClientClosed((*it)->id);
				it = s_dsnetmClients.erase(it);
			}
		}

		for (const auto& c : s_dsnetmClients)
			close_socket(c->sock);
		s_dsnetmClients.clear();
	}

	static bool MakeWakeSocket()
	{
		s_wakeSock = socket(AF_INET, SOCK_DGRAM, 0);
		if (s_wakeSock == BAD_SOCKET)
			return false;
		sockaddr_in addr = {};
		addr.sin_family = AF_INET;
		addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
		socklen_t len = sizeof(addr);
		if (bind(s_wakeSock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0 ||
			getsockname(s_wakeSock, reinterpret_cast<sockaddr*>(&addr), &len) != 0 ||
			connect(s_wakeSock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0)
		{
			close_socket(s_wakeSock);
			s_wakeSock = BAD_SOCKET;
			return false;
		}
		SetNonBlocking(s_wakeSock);
		return true;
	}

	static void ReceiveLoop(socket_t sock)
	{
		std::vector<u8> buf;
		char tmp[4096];
		while (s_running.load(std::memory_order_acquire))
		{
			const int n = recv(sock, tmp, sizeof(tmp), 0);
			if (n <= 0)
				return;
			buf.insert(buf.end(), tmp, tmp + n);

			// Split the stream by the 16-bit length at the start of each DECI2 header.
			size_t pos = 0;
			while (buf.size() - pos >= 8)
			{
				const u16 len = buf[pos] | (buf[pos + 1] << 8);
				if (len < 8)
				{
					Console.Error("PIF: bad packet length %u from the host, dropping the connection", len);
					return;
				}
				if (buf.size() - pos < len)
					break;
				HandleHostPacket(std::vector<u8>(buf.begin() + pos, buf.begin() + pos + len));
				pos += len;
			}
			buf.erase(buf.begin(), buf.begin() + pos);
		}
	}

	// Diagnostics for the stalls in docs/06 (PCSX2_PIF_WATCHDOG=1): logs where both
	// CPUs are every 2s. Reads the registers without any locking, so treat the
	// numbers as samples.
	static void WatchdogMain()
	{
		Threading::SetNameOfCurrentThread("PIF Watchdog");
		// PCSX2_PIF_WATCH_EE / PCSX2_PIF_WATCH_IOP=<hex addr>,<hex addr>,... add EE or IOP
		// RAM words to each line.
		const auto parse = [](const char* name) {
			std::vector<u32> list;
			if (const char* env = std::getenv(name))
			{
				for (const char* p = env; *p;)
				{
					char* end;
					list.push_back(static_cast<u32>(std::strtoul(p, &end, 16)));
					if (end == p)
						break;
					p = (*end == ',') ? end + 1 : end;
				}
			}
			return list;
		};
		const std::vector<u32> watch = parse("PCSX2_PIF_WATCH_EE");
		const std::vector<u32> watchIop = parse("PCSX2_PIF_WATCH_IOP");
		while (s_running.load(std::memory_order_acquire))
		{
			std::this_thread::sleep_for(std::chrono::seconds(2));
			std::string words;
			for (u32 a : watch)
				words += fmt::format(" [{:08x}]={:08x}", a, *reinterpret_cast<const u32*>(eeMem->Main + (a & 0x1fffffc & (Ps2MemSize::ExposedRam - 1))));
			for (u32 a : watchIop)
				words += fmt::format(" iop[{:06x}]={:08x}", a, *reinterpret_cast<const u32*>(iopMem->Main + (a & 0x1ffffc)));
			if (!watch.empty() || !watchIop.empty())
				words += fmt::format(" ms={:08x} sm={:08x} istat={:08x} imask={:08x} dicr={:08x}", psHu32(SBUS_F220), psHu32(SBUS_F230),
					psxHu32(0x1070), psxHu32(0x1074), psxHu32(0x10f4));
			Console.WriteLn("PIF watchdog: vm=%d ee pc=%08x cycle=%u  iop pc=%08x cycle=%u  sock=%d%s",
				static_cast<int>(VMManager::GetState()), cpuRegs.pc, cpuRegs.cycle, psxRegs.pc, psxRegs.cycle,
				s_clientSock != BAD_SOCKET, words.c_str());
		}
	}

	static void ThreadMain()
	{
		Threading::SetNameOfCurrentThread("PIF Bridge");

		while (s_running.load(std::memory_order_acquire))
		{
			socket_t sock = WaitForClient();
			if (sock == BAD_SOCKET)
			{
				if (!s_running.load(std::memory_order_acquire))
					break;
				// Connect mode retries; a failed accept is usually Stop() closing the socket.
				std::this_thread::sleep_for(std::chrono::seconds(1));
				continue;
			}

			SetNoDelay(sock);
			{
				std::lock_guard lock(s_sockMutex);
				s_clientSock = sock;
			}
			Console.WriteLn("PIF: bridge connected");

			ReceiveLoop(sock);

			{
				std::lock_guard lock(s_sockMutex);
				if (s_clientSock != BAD_SOCKET)
					close_socket(s_clientSock);
				s_clientSock = BAD_SOCKET;
			}
			Console.WriteLn("PIF: bridge disconnected");
			// In connect mode, WSL's localhost forwarder accepts and drops when nothing
			// listens behind it, so don't spin.
			if (!s_listenMode && s_running.load(std::memory_order_acquire))
				std::this_thread::sleep_for(std::chrono::seconds(1));
		}
	}

	void Start()
	{
		if (s_running.load(std::memory_order_acquire))
			return;

		s_enabled = ParseConfig();
		if (!s_enabled)
			return;

#ifdef _WIN32
		WSADATA wsa = {};
		if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
		{
			Console.Error("PIF: WSAStartup failed");
			s_enabled = false;
			return;
		}
#endif

		if (s_listenMode)
		{
			sockaddr_in addr;
			if (!ResolveAddress(&addr))
			{
				Console.Error("PIF: can't resolve %s", s_hostName.c_str());
				s_enabled = false;
				return;
			}
			s_listenSock = socket(AF_INET, SOCK_STREAM, 0);
			int one = 1;
#ifdef _WIN32
			// Windows' SO_REUSEADDR would let a second emulator take dsnetm's port.
			const int reuse = s_dsnetmMode ? SO_EXCLUSIVEADDRUSE : SO_REUSEADDR;
#else
			const int reuse = SO_REUSEADDR;
#endif
			setsockopt(s_listenSock, SOL_SOCKET, reuse, reinterpret_cast<const char*>(&one), sizeof(one));
			if (s_listenSock == BAD_SOCKET ||
				bind(s_listenSock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0 ||
				listen(s_listenSock, s_dsnetmMode ? 5 : 1) != 0 ||
				(s_dsnetmMode && !MakeWakeSocket()))
			{
				Console.Error("PIF: can't listen on %s:%u", s_hostName.c_str(), s_port);
				if (s_listenSock != BAD_SOCKET)
					close_socket(s_listenSock);
				s_listenSock = BAD_SOCKET;
				s_enabled = false;
				return;
			}
			if (s_dsnetmMode)
				Console.WriteLn("PIF: dsnetm %s listening on %s:%u", Dsnetm::VERSION, s_hostName.c_str(), s_port);
			else
				Console.WriteLn("PIF: bridge listening on %s:%u", s_hostName.c_str(), s_port);
		}
		else
		{
			Console.WriteLn("PIF: bridge connecting to %s:%u", s_hostName.c_str(), s_port);
		}

		s_running.store(true, std::memory_order_release);
		if (s_dsnetmMode)
		{
			SetNonBlocking(s_listenSock);
			s_dsnetm = std::make_unique<Dsnetm>(s_dsnetmOutput);
			s_nextClientId = 1;
			{
				std::lock_guard lock(s_bootParamsMutex);
				s_bootParams = {};
			}
			if (const char* pcap = std::getenv("PCSX2_PIF_PCAP"); pcap && pcap[0])
			{
				if (s_capture.Open(pcap))
					Console.WriteLn("PIF: dsnetm capture in %s", pcap);
				else
					Console.Error("PIF: can't write the capture %s", pcap);
			}
			s_thread = std::thread(DsnetmMain);
		}
		else
		{
			s_thread = std::thread(ThreadMain);
		}
		if (const char* wd = std::getenv("PCSX2_PIF_WATCHDOG"); wd && wd[0] == '1')
			s_watchdog = std::thread(WatchdogMain);
	}

	void Stop()
	{
		if (!s_running.exchange(false, std::memory_order_acq_rel))
			return;

		if (s_dsnetmMode)
		{
			Wake();
			if (s_thread.joinable())
				s_thread.join();
			close_socket(s_listenSock);
			s_listenSock = BAD_SOCKET;
			close_socket(s_wakeSock);
			s_wakeSock = BAD_SOCKET;
			s_wakePending.store(false, std::memory_order_release);
			s_dsnetm.reset();
			s_capture.Close();
			std::lock_guard lock(s_eventMutex);
			s_events.clear();
		}

		{
			std::lock_guard lock(s_sockMutex);
			if (s_listenSock != BAD_SOCKET)
				close_socket(s_listenSock);
			s_listenSock = BAD_SOCKET;
			if (s_clientSock != BAD_SOCKET)
			{
#ifdef _WIN32
				shutdown(s_clientSock, SD_BOTH);
#else
				shutdown(s_clientSock, SHUT_RDWR);
#endif
			}
		}
		if (s_thread.joinable())
			s_thread.join();
		if (s_watchdog.joinable())
			s_watchdog.join();

		std::lock_guard lock(s_inMutex);
		s_inbound.clear();
		s_inboundPending.store(false, std::memory_order_release);
		s_hold.store(false, std::memory_order_release);
		s_resetReplyOwed.store(false, std::memory_order_release);
	}

	bool IsEnabled()
	{
		return s_enabled;
	}

	void Poll()
	{
		if (!s_inboundPending.load(std::memory_order_acquire) || s_hold.load(std::memory_order_acquire))
			return;

		std::deque<std::vector<u8>> packets;
		{
			std::lock_guard lock(s_inMutex);
			packets.swap(s_inbound);
			s_inboundPending.store(false, std::memory_order_release);
		}
		for (const std::vector<u8>& pkt : packets)
			QueueHostPacket(pkt);
	}

	void SendToHost(const u8* data, size_t size)
	{
		if (s_dsnetmMode)
		{
			if (size >= 8)
				PostTargetEvent(TargetEvent{std::vector<u8>(data, data + size), 0, {}});
			return;
		}

		std::lock_guard lock(s_sockMutex);
		if (s_clientSock == BAD_SOCKET)
			return;

		size_t sent = 0;
		while (sent < size)
		{
			const int n = send(s_clientSock, reinterpret_cast<const char*>(data + sent), static_cast<int>(size - sent), 0);
			if (n <= 0)
			{
				Console.Warning("PIF: send to the host failed");
				return;
			}
			sent += n;
		}
	}

	void OnHandshake()
	{
		const bool rebooted = s_targetBooted.exchange(true, std::memory_order_acq_rel);
		if (s_dsnetmMode)
		{
			// What dsnetm does on mrp.o's EPIPE, with the cause RSTINFO would give: a NETMP
			// reset, a reboot by the target itself, or a boot or reset from the emulator
			// (taken as the reset button). Anything still queued was meant for the old boot.
			int cause = rebooted ? Dsnetm::CAUSE_SOFTWARE : Dsnetm::CAUSE_BUTTON;
			if (s_resetReplyOwed.exchange(false, std::memory_order_acq_rel))
				cause = Dsnetm::CAUSE_NETMP;
			// The DCMP connects go into the card here, at the handshake, rather than from dsnetm's
			// thread a moment later. Arriving at some random point after it, while the new boot's
			// DECI2 stack wasn't ready yet, they sometimes broke a reboot the program did itself: the
			// IOP read its own replies as DCMP requests and sent its "E not up" error forever, or
			// crashed (crusade TR-30). In the card at the handshake, the new driver finds them once it
			// starts receiving, as after a VM reset with the old WSL dsnetm.
			Console.WriteLn("PIF: target booted (cause %d), dsnetm connects to it", cause);
			ClearInbound();
			std::array<u8, 16> params;
			{
				std::lock_guard lock(s_bootParamsMutex);
				params = s_bootParams;
			}
			TargetEvent ev{{}, cause, {}};
			for (const u8 node : {'I', 'E'})
			{
				ev.connects.push_back(Dsnetm::ConnectPacket(node, params));
				QueueHostPacket(ev.connects.back());
			}
			PostTargetEvent(std::move(ev));
			return;
		}
		if (s_resetReplyOwed.exchange(false, std::memory_order_acq_rel))
		{
			// NETMP RESETR: header (len 11, NETMP, H->H), then code 3, result 0 and the cause
			// byte, 6 = reset by NETMP_CODE_RESET (NETMP spec; the TOOL sends 0b 00 00 00 00 04
			// 48 48 03 00 06). dsnetm passes this to its clients as it is.
			const u8 reply[11] = {11, 0, 0, 0, PROTO_NETMP & 0xff, PROTO_NETMP >> 8, 'H', 'H', NETMP_CODE_RESETR, 0, NETMP_CAUSE_RESET};
			Console.WriteLn("PIF: target booted, sending NETMP reset reply");
			SendToHost(reply, sizeof(reply));
			return;
		}
		if (!rebooted)
			return;

		// The target rebooted by itself: drop the link like mrp.o's EPIPE (see the top of
		// this file). Packets still queued were meant for the old boot. Needs checking
		// against a real TOOL, e.g. whether dsnetm gets anything before the EPIPE.
		// dsnetm then sends its clients its own RESETR. On the TOOL the cause comes from mrp.o
		// (0x00, software restart); over TCP dsnetm has no driver to ask and sends 0xff. A
		// RESETR from here wouldn't help: dsnetm only resends its DCMP connects after a link
		// drop or a client's reset, so the boot would wait in DECI2HSYN.
		Console.WriteLn("PIF: target rebooted by itself, dropping the link so dsnetm reconnects");
		{
			std::lock_guard lock(s_inMutex);
			s_inbound.clear();
			s_inboundPending.store(false, std::memory_order_release);
		}
		std::lock_guard lock(s_sockMutex);
		if (s_clientSock != BAD_SOCKET)
			shutdown(s_clientSock, SHUT_BOTH_DIRS);
	}

	void OnVmReset()
	{
		// dsnetm's packets for the old boot (its own connects come after the handshake).
		if (s_dsnetmMode)
			ClearInbound();
		s_hold.store(false, std::memory_order_release);
		s_targetBooted.store(false, std::memory_order_release);
	}
} // namespace Pif::Bridge
