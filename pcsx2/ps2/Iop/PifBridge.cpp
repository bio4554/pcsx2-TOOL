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
#include "common/Threading.h"

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

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
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
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
namespace Pif::Bridge
{
	static constexpr u16 PROTO_NETMP = 0x0400;
	static constexpr u8 NETMP_CODE_RESET = 0x02;
	static constexpr u8 NETMP_CODE_RESETR = 0x03;
	static constexpr u8 NETMP_CAUSE_RESET = 0x06;

	static bool s_enabled;
	static bool s_listenMode;
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

	// Bridge thread: one DECI2 packet from the host.
	static void HandleHostPacket(std::vector<u8> pkt)
	{
		const u16 proto = pkt[4] | (pkt[5] << 8);
		if (proto == PROTO_NETMP && pkt.size() >= 10 && pkt[8] == NETMP_CODE_RESET)
		{
			// The boot parameters in the body are dsnetm's business: it puts them
			// in the DCMP connects that follow.
			Console.WriteLn("PIF: NETMP reset from the host, resetting the VM");
			{
				std::lock_guard lock(s_inMutex);
				s_inbound.clear();
				s_inboundPending.store(false, std::memory_order_release);
			}
			s_hold.store(true, std::memory_order_release);
			s_resetReplyOwed.store(true, std::memory_order_release);
			Host::RunOnCPUThread([]() {
				if (VMManager::HasValidVM())
					VMManager::Reset();
			});
			return;
		}

		std::lock_guard lock(s_inMutex);
		s_inbound.push_back(std::move(pkt));
		s_inboundPending.store(true, std::memory_order_release);
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
			setsockopt(s_listenSock, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&one), sizeof(one));
			if (s_listenSock == BAD_SOCKET ||
				bind(s_listenSock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0 ||
				listen(s_listenSock, 1) != 0)
			{
				Console.Error("PIF: can't listen on %s:%u", s_hostName.c_str(), s_port);
				if (s_listenSock != BAD_SOCKET)
					close_socket(s_listenSock);
				s_listenSock = BAD_SOCKET;
				s_enabled = false;
				return;
			}
			Console.WriteLn("PIF: bridge listening on %s:%u", s_hostName.c_str(), s_port);
		}
		else
		{
			Console.WriteLn("PIF: bridge connecting to %s:%u", s_hostName.c_str(), s_port);
		}

		s_running.store(true, std::memory_order_release);
		s_thread = std::thread(ThreadMain);
		if (const char* wd = std::getenv("PCSX2_PIF_WATCHDOG"); wd && wd[0] == '1')
			s_watchdog = std::thread(WatchdogMain);
	}

	void Stop()
	{
		if (!s_running.exchange(false, std::memory_order_acq_rel))
			return;

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
		s_hold.store(false, std::memory_order_release);
		s_targetBooted.store(false, std::memory_order_release);
	}
} // namespace Pif::Bridge
