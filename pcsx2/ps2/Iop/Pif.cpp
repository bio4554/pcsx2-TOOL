// SPDX-FileCopyrightText: 2002-2026 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

#include "Common.h"
#include "IopHw.h"
#include "IopDma.h"
#include "R3000A.h"
#include "ps2/BiosTools.h"
#include "ps2/Iop/Pif.h"

#include <cstdlib>
#include <cstring>
#include <deque>

#define PIFLOG_ENABLE 1
#define PifLog if (PIFLOG_ENABLE) DevCon

namespace Pif
{
	// Board ID. 0x4127 selects the small (0x7f) FIFO thresholds in DECI2DRP;
	// 0x4126 selects 0x3ff. Which one a real TOOL reports still needs checking.
	static constexpr u16 BOARD_ID = 0x4127;

	// How often the IOP polls the bridge for host packets (about 110us).
	static constexpr s32 POLL_CYCLES = 4096;

	static constexpr u32 FIFO_WORDS = 4096;
	static constexpr u32 FIFO_THRESHOLD = 0x7f;

	enum Reg : u32
	{
		BID = 0x00,
		RST = 0x04,
		CPS = 0x08,
		CPR = 0x0c,
		FST = 0x10,
		TXC = 0x14,
		RXC = 0x18,
		F1C = 0x1c,
		IST = 0x20,
		ISP = 0x24,
		IER = 0x28,
		CSI = 0x2c,
		FSI = 0x30,
		DMA_CTRL = 0x34,
		DMA_UNIT = 0x38,
		DMA_COUNT = 0x3c,
		DATA = 0x40,
	};

	// IST bits. FIFO flags use the same polarity as FST: the flags in ISP
	// (0xe0c0: TFF TAF THF RAE REF) are active-low, the others active-high.
	enum IstBits : u16
	{
		IST_SI0 = 1 << 0, // char from the host
		IST_SI1 = 1 << 1, // host consumed our char
		IST_SI2 = 1 << 2, // boot handshake
		IST_RST = 1 << 3,
		IST_RXC = 1 << 4, // packets waiting
		IST_TXC = 1 << 5,
		IST_SIGNALS = IST_SI0 | IST_SI1 | IST_SI2,
	};

	// FIFO condition bits, active-high, in FST order: E AE HF AF F.
	enum FifoCond : u16
	{
		COND_E = 1 << 0,
		COND_AE = 1 << 1,
		COND_HF = 1 << 2,
		COND_AF = 1 << 3,
		COND_F = 1 << 4,
	};

	// Active-low flags per FIFO, matching the 0x1c03 decode in both drivers.
	static constexpr u16 TX_ACTIVE_LOW = COND_HF | COND_AF | COND_F;
	static constexpr u16 RX_ACTIVE_LOW = COND_E | COND_AE;

	struct State
	{
		u16 rst;
		u16 cps; // char from the host
		u16 cpr; // char to the host
		u16 f1c;
		u16 signals; // IST SI0-2
		u16 isp;
		u16 ier;
		u16 dmaCtrl;
		u16 dmaUnit;
		u16 dmaCount;
		u32 rxPackets; // complete packets in the Rx FIFO, read through RXC
		bool irqLine;
	};

	static State s_state;

	// Debug aid until the bridge exists: with PCSX2_PIF_SELFCONNECT=1, play dsnetm
	// and send DCMP connects to I and E once the handshake is accepted. The IOP's
	// DECI2HSYN busy-waits for the boot mode the connect registers.
	static bool s_selfConnect;

	static std::string s_cpLine;

	// PS2 -> host. The host drains words as soon as they arrive, so the Tx
	// FIFO always reads empty and a packet just accumulates here until TXC.
	static std::vector<u32> s_txPacket;

	// Host -> PS2. Words in the FIFO, plus packets waiting for FIFO space.
	static std::deque<u32> s_rxFifo;
	static std::deque<std::vector<u32>> s_rxPending;
	static u32 s_rxPendingOffset; // words of s_rxPending.front() already in the FIFO

	static u16 FifoCondition(u32 level)
	{
		u16 c = 0;
		if (level == 0)
			c |= COND_E;
		if (level <= FIFO_THRESHOLD)
			c |= COND_AE;
		if (level > FIFO_WORDS / 2)
			c |= COND_HF;
		if (level >= FIFO_WORDS - FIFO_THRESHOLD)
			c |= COND_AF;
		if (level >= FIFO_WORDS)
			c |= COND_F;
		return c;
	}

	static u16 TxFlags() { return FifoCondition(0) ^ TX_ACTIVE_LOW; }
	static u16 RxFlags() { return FifoCondition(static_cast<u32>(s_rxFifo.size())) ^ RX_ACTIVE_LOW; }

	static u16 ReadFst() { return (TxFlags() << 8) | RxFlags(); }

	static u16 ReadIst()
	{
		u16 ist = (TxFlags() << 11) | (RxFlags() << 6) | s_state.signals;
		if (s_state.rxPackets)
			ist |= IST_RXC;
		return ist;
	}

	static u32 IrqNumber()
	{
		// DECI2DRP uses IRQ 10 in PS1 mode (ICFG bit 3), IRQ 23 otherwise.
		return (psxHu32(0x1450) & 8) ? 10 : 23;
	}

	// The IOP latches interrupts on an edge, so only raise one when the
	// card's line goes from idle to pending.
	static void UpdateIrq()
	{
		const bool line = ((ReadIst() ^ s_state.isp) & s_state.ier) != 0;
		if (line && !s_state.irqLine)
			iopIntcIrq(IrqNumber());
		s_state.irqLine = line;
	}

	// Move queued host packets into the Rx FIFO while there is space.
	static void FillRxFifo()
	{
		while (!s_rxPending.empty() && s_rxFifo.size() < FIFO_WORDS)
		{
			const std::vector<u32>& pkt = s_rxPending.front();
			const u32 n = std::min<u32>(static_cast<u32>(pkt.size()) - s_rxPendingOffset, FIFO_WORDS - static_cast<u32>(s_rxFifo.size()));
			s_rxFifo.insert(s_rxFifo.end(), pkt.begin() + s_rxPendingOffset, pkt.begin() + s_rxPendingOffset + n);
			s_rxPendingOffset += n;
			if (s_rxPendingOffset == pkt.size())
			{
				// The host writes TXC once the whole packet is in the FIFO.
				s_rxPending.pop_front();
				s_rxPendingOffset = 0;
				s_state.rxPackets++;
			}
		}
	}

	static void TxPush(u32 word)
	{
		s_txPacket.push_back(word);
	}

	static u32 RxPop()
	{
		if (s_rxFifo.empty())
		{
			Console.Warning("PIF: read from empty Rx FIFO");
			return 0;
		}
		const u32 word = s_rxFifo.front();
		s_rxFifo.pop_front();
		return word;
	}

	// PS2 wrote TXC: the packet in s_txPacket is complete.
	static void TxPacketDone()
	{
		if (s_txPacket.empty())
		{
			PifLog.WriteLn("PIF: TXC with no data");
			return;
		}

		// DECI2 header: u16 length, u16 reserved, u16 protocol, u8 source, u8 destination.
		// DECI1 packets start with u16 magic 0xa14c, u16 length instead.
		const bool deci1 = (s_txPacket[0] & 0xffff) == 0xa14c;
		const u32 len = deci1 ? (s_txPacket[0] >> 16) : (s_txPacket[0] & 0xffff);
		const u32 proto = s_txPacket.size() > 1 ? (s_txPacket[1] & 0xffff) : 0;
		const u32 src = s_txPacket.size() > 1 ? ((s_txPacket[1] >> 16) & 0xff) : 0;
		const u32 dst = s_txPacket.size() > 1 ? (s_txPacket[1] >> 24) : 0;
		if (deci1)
			Console.Warning("PIF: DECI1 packet from the PS2 (len=%u), the host only speaks DECI2", len);
		else
			PifLog.WriteLn("PIF: Tx packet len=%u proto=%04x %c->%c (%u words)", len, proto,
				src ? src : '?', dst ? dst : '?', static_cast<u32>(s_txPacket.size()));
		if ((len + 3) / 4 != s_txPacket.size())
			Console.Warning("PIF: Tx packet length %u doesn't match %u words written", len, static_cast<u32>(s_txPacket.size()));

		if (!deci1)
		{
			std::vector<u8> bytes(s_txPacket.size() * 4);
			std::memcpy(bytes.data(), s_txPacket.data(), bytes.size());
			Bridge::SendToHost(bytes.data(), std::min<size_t>(len, bytes.size()));
		}
		s_txPacket.clear();
	}

	// Model of the Linux side reacting to a signal from the PS2 (FSI write).
	static void HostSignal(u16 bits)
	{
		if (bits & IST_SI0)
		{
			// Char from the PS2 in CPR (mrp_cpr). Acknowledge with SI1. DECI2
			// uses this channel as an early debug console, so print it by line.
			const char c = static_cast<char>(s_state.cpr & 0xff);
			if (c == '\n')
			{
				PifLog.WriteLn("PIF CP: %s", s_cpLine.c_str());
				s_cpLine.clear();
			}
			else if (c != '\r')
			{
				s_cpLine += c;
			}
			s_state.signals |= IST_SI1;
		}
		if (bits & IST_SI2)
		{
			// Boot handshake (mrp_bootp). The PS2 driver puts its protocol in CPR:
			// DECI2DRP asks for 0x1300, deci1drv for 0x0200. The host accepts
			// 0x1300 and answers anything else with 0x1f00, which makes that driver
			// fail its probe, so the ROM falls through to the DECI2 stack.
			s_state.cps = (s_state.cpr == 0x1300) ? 0x1300 : 0x1f00;
			PifLog.WriteLn("PIF: boot handshake, CPR=%04x -> CPS=%04x", s_state.cpr, s_state.cps);
			s_state.signals |= IST_SI2;

			if (s_state.cps == 0x1300)
				Bridge::OnHandshake();

			if (s_selfConnect && s_state.cps == 0x1300)
			{
				for (const u8 node : {'I', 'E'})
				{
					// 32-byte DCMP connect (type 0, code 0), zero body, as dsnetm sends.
					std::vector<u8> pkt(32, 0);
					pkt[0] = 32;
					pkt[4] = 0x01; // DCMP
					pkt[6] = 'H';
					pkt[7] = node;
					QueueHostPacket(pkt);
				}
				PifLog.WriteLn("PIF: self-connect, queued DCMP connects to I and E");
			}
		}
	}

	bool IsPresent()
	{
		return BiosZone == "T10K";
	}

	void Reset()
	{
		s_state = {};
		const char* env = std::getenv("PCSX2_PIF_SELFCONNECT");
		s_selfConnect = env && env[0] == '1';
		s_cpLine.clear();
		s_txPacket.clear();
		s_rxFifo.clear();
		s_rxPending.clear();
		s_rxPendingOffset = 0;

		ToolSif2::Reset();

		if (IsPresent())
		{
			Bridge::Start();
			Bridge::OnVmReset();
			if (Bridge::IsEnabled())
			{
				s_selfConnect = false;
				PSX_INT(IopEvt_PIF, POLL_CYCLES);
			}
		}
	}

	u16 Read16(u32 addr)
	{
		u16 ret;
		switch (addr - BASE)
		{
			case BID: ret = BOARD_ID; break;
			case RST: ret = s_state.rst & ~0x100; break; // bit 8 set means no card
			case CPS: ret = s_state.cps; break;
			case CPR: ret = s_state.cpr; break;
			case FST: ret = ReadFst(); break;
			case TXC: ret = 0; break;
			case RXC: ret = static_cast<u16>(s_state.rxPackets); break;
			case F1C: ret = s_state.f1c; break;
			case IST: ret = ReadIst(); break;
			case ISP: ret = s_state.isp; break;
			case IER: ret = s_state.ier; break;
			case DMA_CTRL: ret = s_state.dmaCtrl; break;
			case DMA_UNIT: ret = s_state.dmaUnit; break;
			case DMA_COUNT: ret = s_state.dmaCount; break;
			case DATA:
				Console.Warning("PIF: 16-bit read from the data port");
				ret = static_cast<u16>(RxPop());
				UpdateIrq();
				break;
			default:
				ret = 0;
				break;
		}
		return ret;
	}

	u32 Read32(u32 addr)
	{
		if (addr - BASE == DATA)
		{
			const u32 ret = RxPop();
			FillRxFifo();
			UpdateIrq();
			return ret;
		}
		return Read16(addr);
	}

	u8 Read8(u32 addr)
	{
		const u16 val = Read16(addr & ~1);
		return (addr & 1) ? (val >> 8) : (val & 0xff);
	}

	void Write16(u32 addr, u16 val)
	{
		switch (addr - BASE)
		{
			case BID: break; // DECI2DRP writes 0 and expects the ID to read back
			case RST: s_state.rst = val; break;
			case CPS: s_state.cps = val; break;
			case CPR: s_state.cpr = val; break;
			case FST: break;
			case TXC:
				if (val)
					TxPacketDone();
				break;
			case RXC:
				// PS2 consumed a packet.
				if (val && s_state.rxPackets)
					s_state.rxPackets--;
				break;
			case F1C: s_state.f1c = val; break;
			case IST: break;
			case ISP: s_state.isp = val; break;
			case IER: s_state.ier = val; break;
			case CSI: s_state.signals &= ~(val & IST_SIGNALS); break;
			case FSI: HostSignal(val & IST_SIGNALS); break;
			case DMA_CTRL: s_state.dmaCtrl = val; break;
			case DMA_UNIT: s_state.dmaUnit = val; break;
			case DMA_COUNT: s_state.dmaCount = val; break;
			case DATA:
				Console.Warning("PIF: 16-bit write to the data port");
				TxPush(val);
				break;
			default:
				break;
		}
		UpdateIrq();
	}

	void Write32(u32 addr, u32 val)
	{
		if (addr - BASE == DATA)
		{
			TxPush(val);
			UpdateIrq();
			return;
		}
		Write16(addr, static_cast<u16>(val));
	}

	void Write8(u32 addr, u8 val)
	{
		// Byte writes to CPR are the IOP early console (logged by the caller).
		if (addr - BASE == CPR)
			s_state.cpr = (s_state.cpr & 0xff00) | val;
	}

	u32 DmaRead(u32* dst, u32 words)
	{
		u32 n = 0;
		while (n < words && !s_rxFifo.empty())
		{
			dst[n++] = s_rxFifo.front();
			s_rxFifo.pop_front();
			FillRxFifo();
		}
		if (n < words)
			Console.Warning("PIF: DMA read of %u words, only %u available", words, n);
		UpdateIrq();
		return n;
	}

	u32 DmaWrite(const u32* src, u32 words)
	{
		s_txPacket.insert(s_txPacket.end(), src, src + words);
		UpdateIrq();
		return words;
	}

	void QueueHostPacket(const std::vector<u8>& packet)
	{
		std::vector<u32> words((packet.size() + 3) / 4, 0);
		std::memcpy(words.data(), packet.data(), packet.size());
		s_rxPending.push_back(std::move(words));
		FillRxFifo();
		UpdateIrq();
	}
} // namespace Pif

void pifInterrupt()
{
	if (!Pif::Bridge::IsEnabled())
		return;
	Pif::Bridge::Poll();
	ToolSif2::Poll();
	PSX_INT(IopEvt_PIF, Pif::POLL_CYCLES);
}

// IOP DMA channel 5 (PIF). Transfers complete immediately; needs proper testing
// against DECI2DRP's DMA path (sends of 8+ words, large receives).
void psxDma5(u32 madr, u32 bcr, u32 chcr)
{
	const u32 words = (bcr >> 16) * (bcr & 0xffff);

	switch (chcr & 0x01000201)
	{
		case 0x01000201: // IOP RAM -> card
			PSXDMA_LOG("*** DMA 5 - PIF mem2pif *** %x addr = %x size = %x", chcr, madr, bcr);
			Pif::DmaWrite((u32*)iopPhysMem(madr), words);
			break;

		case 0x01000200: // card -> IOP RAM
			PSXDMA_LOG("*** DMA 5 - PIF pif2mem *** %x addr = %x size = %x", chcr, madr, bcr);
			Pif::DmaRead((u32*)iopPhysMem(madr), words);
			psxCpu->Clear(madr, words);
			break;

		default:
			Console.Error("*** DMA 5 - PIF unknown *** %x addr = %x size = %x", chcr, madr, bcr);
			break;
	}

	HW_DMA5_CHCR &= ~0x01000000;
	psxDmaInterrupt(5);
}
