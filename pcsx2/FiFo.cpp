// SPDX-FileCopyrightText: 2002-2026 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

#include "Common.h"

#include "Gif.h"
#include "Gif_Unit.h"
#include "MTGS.h"
#include "Vif.h"
#include "Vif_Dma.h"
#include "ps2/Iop/Pif.h"

#include <algorithm>
#include <vector>

//////////////////////////////////////////////////////////////////////////
/////////////////////////// Quick & dirty FIFO :D ////////////////////////
//////////////////////////////////////////////////////////////////////////

// Notes on FIFO implementation
//
// The FIFO consists of four separate pages of HW register memory, each mapped to a
// PS2 device.  They are listed as follows:
//
// 0x4000 - 0x5000 : VIF0  (all registers map to 0x4000)
// 0x5000 - 0x6000 : VIF1  (all registers map to 0x5000)
// 0x6000 - 0x7000 : GS    (all registers map to 0x6000)
// 0x7000 - 0x8000 : IPU   (registers map to 0x7000 and 0x7010, respectively)

// TOOL mode (crusade TR-29): the EE kernel's RDIMG, the debuggers' VRAM grab, reads a GS download
// through the VIF1 FIFO register one quadword at a time. Each read waited for the GS thread, which held
// grabs to about half the TOOL's rate. The GS reads the whole image when the download starts anyway
// (GSState::InitReadFIFO), so the first read takes all that's left of it at once and the next ones come
// from here. Not saved in savestates: TOOL mode has none.
static std::vector<u128> s_vif1Download;
static size_t s_vif1DownloadPos;

void Vif1DownloadDiscard()
{
	s_vif1Download.clear();
	s_vif1DownloadPos = 0;
}

u32 Vif1DownloadTake(u128* out, u32 qwc)
{
	const u32 n = std::min<u32>(qwc, static_cast<u32>(s_vif1Download.size() - s_vif1DownloadPos));
	std::copy_n(s_vif1Download.data() + s_vif1DownloadPos, n, out);
	s_vif1DownloadPos += n;
	return n;
}

void ReadFIFO_VIF1(mem128_t* out)
{
	if (vif1Regs.stat.test(VIF1_STAT_INT | VIF1_STAT_VSS | VIF1_STAT_VIS | VIF1_STAT_VFS))
		DevCon.Warning("Reading from vif1 fifo when stalled");

	ZeroQWC(out); // Clear first in case no data gets written...
	pxAssertRel(vif1Regs.stat.FQC != 0, "FQC = 0 on VIF FIFO READ!");
	if (vif1Regs.stat.FDR)
	{
		if (vif1Regs.stat.FQC > vif1.GSLastDownloadSize)
		{
			DevCon.Warning("Warning! GS Download size < FIFO count!");
		}
		if (vif1Regs.stat.FQC > 0)
		{
			if (Pif::IsPresent())
			{
				if (s_vif1DownloadPos == s_vif1Download.size())
				{
					s_vif1Download.assign(std::max<u32>(1, vif1.GSLastDownloadSize), u128{});
					s_vif1DownloadPos = 0;
					MTGS::InitAndReadFIFO(reinterpret_cast<u8*>(s_vif1Download.data()), static_cast<u32>(s_vif1Download.size()));
				}
				Vif1DownloadTake(out, 1);
			}
			else
			{
				MTGS::InitAndReadFIFO(reinterpret_cast<u8*>(out), 1);
			}
			vif1.GSLastDownloadSize--;
			GUNIT_LOG("ReadFIFO_VIF1");
			if (vif1.GSLastDownloadSize <= 16)
				gifRegs.stat.OPH = false;
			vif1Regs.stat.FQC = std::min((u32)16, vif1.GSLastDownloadSize);
		}
	}

	VIF_LOG("ReadFIFO/VIF1 -> 0x%08X.%08X.%08X.%08X", out->_u32[0], out->_u32[1], out->_u32[2], out->_u32[3]);
}

//////////////////////////////////////////////////////////////////////////
// WriteFIFO Pages
//
void WriteFIFO_VIF0(const mem128_t* value)
{
	VIF_LOG("WriteFIFO/VIF0 <- 0x%08X.%08X.%08X.%08X", value->_u32[0], value->_u32[1], value->_u32[2], value->_u32[3]);

	vif0ch.qwc += 1;
	if (vif0.irqoffset.value != 0 && vif0.vifstalled.enabled)
		DevCon.Warning("Offset on VIF0 FIFO start!");
	[[maybe_unused]] bool ret = VIF0transfer((u32*)value, 4);

	if (vif0.cmd)
	{
		if (vif0.done && vif0ch.qwc == 0)
			vif0Regs.stat.VPS = VPS_WAITING;
	}
	else
	{
		vif0Regs.stat.VPS = VPS_IDLE;
	}

	pxAssertMsg(ret, "vif stall code not implemented");
}

void WriteFIFO_VIF1(const mem128_t* value)
{
	VIF_LOG("WriteFIFO/VIF1 <- 0x%08X.%08X.%08X.%08X", value->_u32[0], value->_u32[1], value->_u32[2], value->_u32[3]);

	if (vif1Regs.stat.FDR)
	{
		DevCon.Warning("writing to fifo when fdr is set!");
	}
	if (vif1Regs.stat.test(VIF1_STAT_INT | VIF1_STAT_VSS | VIF1_STAT_VIS | VIF1_STAT_VFS))
	{
		DevCon.Warning("writing to vif1 fifo when stalled");
	}
	if (vif1.irqoffset.value != 0 && vif1.vifstalled.enabled)
	{
		DevCon.Warning("Offset on VIF1 FIFO start!");
	}

	// TOOL mode: behind what the FIFO already holds, or while the VIF waits, the write queues (Vif1_Dma.cpp).
	if (vif1FifoTakeCpuWrite(value->_u32))
		return;

	[[maybe_unused]] bool ret = VIF1transfer((u32*)value, 4);

	if (vif1.cmd)
	{
		if (vif1.done && !vif1ch.qwc)
			vif1Regs.stat.VPS = VPS_WAITING;
	}
	else
		vif1Regs.stat.VPS = VPS_IDLE;

	if (gifRegs.stat.APATH == 2 && gifUnit.gifPath[1].isDone())
	{
		gifRegs.stat.APATH = 0;
		gifRegs.stat.OPH = 0;
		vif1Regs.stat.VGW = false; //Let vif continue if it's stuck on a flush

		if (gifUnit.checkPaths(1, 0, 1))
			gifUnit.Execute(false, true);
	}

	pxAssertMsg(ret, "vif stall code not implemented");
}

void WriteFIFO_GIF(const mem128_t* value)
{
	GUNIT_LOG("WriteFIFO_GIF()");
	if ((!gifUnit.CanDoPath3() || gif_fifo.fifoSize > 0))
	{
		//DevCon.Warning("GIF FIFO HW Write");
		gif_fifo.write_fifo((u32*)value, 1);
		gif_fifo.read_fifo();
	}
	else
	{
		gifUnit.TransferGSPacketData(GIF_TRANS_FIFO, (u8*)value, 16);
	}

	if (gifUnit.gifPath[GIF_PATH_3].state == GIF_PATH_WAIT)
		gifUnit.gifPath[GIF_PATH_3].state = GIF_PATH_IDLE;

	if (gifRegs.stat.APATH == 3)
	{
		gifRegs.stat.APATH = 0;
		gifRegs.stat.OPH = 0;

		if (gifUnit.gifPath[GIF_PATH_3].state == GIF_PATH_IDLE || gifUnit.gifPath[GIF_PATH_3].state == GIF_PATH_WAIT)
		{
			if (gifUnit.checkPaths(1, 1, 0))
				gifUnit.Execute(false, true);
		}
	}
}
