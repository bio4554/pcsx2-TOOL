// SPDX-FileCopyrightText: 2002-2026 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

#include "Common.h"
#include "Dmac.h"
#include "Hw.h"
#include "IopHw.h"
#include "IopDma.h"
#include "IopMem.h"
#include "R3000A.h"
#include "ps2/Iop/Pif.h"

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <deque>

// SIF2 as the TOOL's DECI2 stack uses it (see Pif.h). The IOP and EE agree on
// every chunk through mailboxes before either side starts its DMA, so the model
// is a plain word FIFO between the two channels:
//  - the source (IOP DMA2 from memory, or EE ch7 from memory) pushes all of its
//    words at once, but only completes once the destination has taken them, as the
//    real FIFO is small. Leftovers below the other side's rounding (a QW for an EE
//    source, a 32-word block for an IOP source) count as taken;
//  - the destination drains the FIFO and completes when it is full, or when the
//    FIFO runs dry after it got something (the block/QW rounding on either side
//    asks for a few words more than were sent);
//  - an accept (SM or MS bit 30) drops whatever rounding left in the FIFO.
// Needs proper testing against a real TOOL, in particular the FIFO size and what
// happens to the rounding words.

#define SIF2LOG_ENABLE 0
#define Sif2Log if (SIF2LOG_ENABLE) DevCon

// Debug trace of the whole handshake: PCSX2_SIF2_TRACE=<hex IOP address of DECI2DRS's
// state word> (0x2fb94 with the 2004-08-20 ROM: DECI2DRS's interface struct + 4).

namespace ToolSif2
{
	static constexpr u32 DMA2_MADR = 0x1f8010a0;
	static constexpr u32 DMA2_BCR = 0x1f8010a4;
	static constexpr u32 DMA2_CHCR = 0x1f8010a8;

	static constexpr u32 CHCR_FROM_MEM = 1 << 0;
	static constexpr u32 CHCR_BUSY = 1 << 24;

	struct Dest
	{
		bool active;
		u32 addr;
		u32 remaining; // words
		u32 got;       // words
	};

	static std::deque<u32> s_fifo;
	static Dest s_iopDest;
	static Dest s_eeDest;
	// The source side waiting for its words to be taken.
	enum class Source : u8
	{
		None,
		Iop,
		Ee,
	};
	static Source s_source;
	static u32 s_traceAddr;
	// Polls for which MS/SM bit 31 (a chunk waiting) has been set with no flag write.
	static u32 s_msAge;
	static u32 s_smAge;

	static void Trace(const char* what, u32 value = 0);
	static void CompleteSource();

	bool Active()
	{
		return Pif::IsPresent() && !(psxHu32(HW_ICFG) & 8);
	}

	void Reset()
	{
		const char* trace = std::getenv("PCSX2_SIF2_TRACE");
		s_traceAddr = trace ? static_cast<u32>(std::strtoul(trace, nullptr, 16)) : 0;
		s_fifo.clear();
		s_iopDest = {};
		s_eeDest = {};
		s_source = Source::None;
		s_msAge = 0;
		s_smAge = 0;
	}

	void Poll()
	{
		if (!Active())
			return;
		// Safety net: a chunk still waiting after about 1ms lost its doorbell, so ring again.
		// With the completion order below this shouldn't be needed; it warns when it is.
		static constexpr u32 LOST_POLLS = 9;
		s_msAge = (psHu32(SBUS_F220) & 0x80000000) ? s_msAge + 1 : 0;
		s_smAge = (psHu32(SBUS_F230) & 0x80000000) ? s_smAge + 1 : 0;
		if (s_msAge >= LOST_POLLS)
		{
			Console.Warning("SIF2: EE chunk waiting for 1ms, ringing the IOP again");
			Trace("re-ring IOP");
			s_msAge = 0;
			iopIntcIrq(1);
		}
		if (s_smAge >= LOST_POLLS)
		{
			Console.Warning("SIF2: IOP chunk waiting for 1ms, ringing the EE again");
			Trace("re-ring EE");
			s_smAge = 0;
			hwIntcIrq(INTC_SBUS);
		}
	}

	static void Trace(const char* what, u32 value)
	{
		if (!s_traceAddr)
			return;
		DevCon.WriteLn("SIF2T: %-16s %08x  drs=%08x ms=%08x sm=%08x fifo=%u  ee=%u iop=%u", what, value,
			*reinterpret_cast<const u32*>(iopMem->Main + (s_traceAddr & 0x1ffffc)), psHu32(SBUS_F220), psHu32(SBUS_F230),
			static_cast<u32>(s_fifo.size()), cpuRegs.cycle, psxRegs.cycle);
	}

	void OnFlagWrite(const char* what, u32 value)
	{
		if (!Active() || !(value & 0xc0000000))
			return;
		Trace(what, value);
		if (value & 0x80000000)
		{
			s_msAge = 0;
			s_smAge = 0;
		}
		if ((value & 0x40000000) && std::strstr(what, "sets"))
		{
			if (!s_fifo.empty())
				Sif2Log.WriteLn("SIF2: accept drops %u leftover words", static_cast<u32>(s_fifo.size()));
			s_fifo.clear();
			if (s_source != Source::None)
			{
				Console.Warning("SIF2: accept while a source was still waiting");
				CompleteSource();
			}
		}
	}

	void EeInterrupt()
	{
		Trace("EE ch7 done");
	}

	static void Push(const u32* src, u32 words)
	{
		s_fifo.insert(s_fifo.end(), src, src + words);
	}

	static u32 Pull(u32* dst, u32 words)
	{
		const u32 n = std::min<u32>(words, static_cast<u32>(s_fifo.size()));
		std::copy_n(s_fifo.begin(), n, dst);
		s_fifo.erase(s_fifo.begin(), s_fifo.begin() + n);
		return n;
	}

	// Completion timing, in EE cycles (the IOP runs at 1/8 of the EE clock). Both DECI2
	// drivers lose a chunk whose doorbell rings while they still see their own DMA as
	// running (DECI2DRS defers it until an RCV_ON that may never come; the EE kernel
	// ignores SM bit 31 while its own send is pending). So the source completes no
	// earlier than the destination took its data, and is signalled first.
	static constexpr u32 SOURCE_CYCLES = 64;
	static constexpr u32 DEST_CYCLES = 192;

	static void CompleteIop(bool dest)
	{
		PSX_INT(IopEvt_SIF2, (dest ? DEST_CYCLES : SOURCE_CYCLES) / 8);
	}

	static void CompleteEe(u32 qwc, bool dest)
	{
		sif2dma.madr += qwc * 16;
		sif2dma.qwc = 0;
		CPU_INT(DMAC_SIF2, dest ? DEST_CYCLES : SOURCE_CYCLES);
	}

	static void CompleteSource()
	{
		if (s_source == Source::Iop)
			CompleteIop(false);
		else if (s_source == Source::Ee)
			CompleteEe(sif2dma.qwc, false);
		s_source = Source::None;
	}

	// After a destination finished: has the source's data all been taken?
	static void CheckSource()
	{
		const u32 rounding = (s_source == Source::Ee) ? 4 : 32;
		if (s_source != Source::None && s_fifo.size() < rounding)
			CompleteSource();
	}

	static void Pump()
	{
		if (s_iopDest.active && !s_fifo.empty())
		{
			u32* dst = reinterpret_cast<u32*>(iopPhysMem(s_iopDest.addr + s_iopDest.got * 4));
			const u32 n = Pull(dst, s_iopDest.remaining);
			psxCpu->Clear(s_iopDest.addr + s_iopDest.got * 4, n);
			s_iopDest.got += n;
			s_iopDest.remaining -= n;
			if (s_iopDest.remaining == 0 || s_fifo.empty())
			{
				Sif2Log.WriteLn("SIF2: EE->IOP %u words to %06x done", s_iopDest.got, s_iopDest.addr);
				HW_DMA2_MADR = s_iopDest.addr + s_iopDest.got * 4;
				s_iopDest.active = false;
				CheckSource();
				CompleteIop(true);
			}
		}
		if (s_eeDest.active && !s_fifo.empty())
		{
			tDMA_TAG* tag = dmaGetAddr(s_eeDest.addr + s_eeDest.got * 4, true);
			if (!tag)
			{
				Console.Error("SIF2: bad EE address %08x", s_eeDest.addr);
				s_eeDest.active = false;
				s_fifo.clear();
				CheckSource();
				CompleteEe(sif2dma.qwc, true);
				return;
			}
			const u32 n = Pull(reinterpret_cast<u32*>(tag), s_eeDest.remaining);
			s_eeDest.got += n;
			s_eeDest.remaining -= n;
			if (s_eeDest.remaining == 0 || s_fifo.empty())
			{
				Sif2Log.WriteLn("SIF2: IOP->EE %u words to %08x done", s_eeDest.got, s_eeDest.addr);
				s_eeDest.active = false;
				// The EE takes a whole chunk at once: whatever is left is the IOP's block rounding.
				s_fifo.clear();
				CheckSource();
				CompleteEe(sif2dma.qwc, true);
			}
		}
	}

	static void IopStart()
	{
		Trace("IOP DMA2 start", HW_DMA2_CHCR);
		const u32 chcr = HW_DMA2_CHCR;
		const u32 madr = HW_DMA2_MADR & 0xffffff;
		const u32 words = HW_DMA2_BCR_L16 * HW_DMA2_BCR_H16;

		if (!(HW_DMA_PCR & (8 << (2 * 4))))
		{
			Console.Warning("SIF2: IOP DMA2 started while disabled in DPCR");
			return;
		}

		if (chcr & CHCR_FROM_MEM)
		{
			Sif2Log.WriteLn("SIF2: IOP send %u words from %06x (chcr %08x)", words, madr, chcr);
			Push(reinterpret_cast<const u32*>(iopPhysMem(madr)), words);
			HW_DMA2_MADR = madr + words * 4;
			s_source = Source::Iop;
		}
		else
		{
			Sif2Log.WriteLn("SIF2: IOP receive %u words to %06x (chcr %08x)", words, madr, chcr);
			s_iopDest = {true, madr, words, 0};
		}
		Pump();
	}

	void IopWrite(u32 addr, u32 val, int size)
	{
		if (size == 2)
			psxHu16(addr) = static_cast<u16>(val);
		else
			psxHu32(addr & ~3u) = val;

		if ((addr & ~3u) != DMA2_CHCR || size != 4)
			return;

		if (val & CHCR_BUSY)
			IopStart();
		else if (s_iopDest.active)
		{
			// sceSifDma2Transfer clears CHCR before every start; anything else is an abort.
			Sif2Log.WriteLn("SIF2: IOP receive stopped after %u words", s_iopDest.got);
			s_iopDest.active = false;
		}
	}

	u32 IopRead(u32 addr, int size)
	{
		return (size == 2) ? psxHu16(addr) : psxHu32(addr & ~3u);
	}

	void IopInterrupt()
	{
		Trace("IOP DMA2 done");
		HW_DMA2_CHCR &= ~CHCR_BUSY;
		psxDmaInterrupt(2);
	}

	void EeStart()
	{
		Trace("EE ch7 start", sif2dma.chcr._u32);
		if (sif2dma.chcr.MOD != NORMAL_MODE)
			Console.Warning("SIF2: EE ch7 mode %u, only normal mode is modelled", sif2dma.chcr.MOD);

		const u32 qwc = sif2dma.qwc;
		if (sif2dma.chcr.DIR)
		{
			tDMA_TAG* tag = dmaGetAddr(sif2dma.madr, false);
			Sif2Log.WriteLn("SIF2: EE send %u QW from %08x", qwc, sif2dma.madr);
			s_source = Source::Ee;
			if (tag)
				Push(reinterpret_cast<const u32*>(tag), qwc * 4);
			else
			{
				Console.Error("SIF2: bad EE address %08x", sif2dma.madr);
				CompleteSource();
			}
		}
		else
		{
			Sif2Log.WriteLn("SIF2: EE receive %u QW to %08x", qwc, sif2dma.madr);
			s_eeDest = {true, sif2dma.madr, qwc * 4, 0};
		}
		Pump();
	}
} // namespace ToolSif2
