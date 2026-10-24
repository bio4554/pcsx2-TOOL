// SPDX-FileCopyrightText: 2002-2026 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

#pragma once

#include <vector>

// PIF/MRP bridge card of the DTL-T10000 (TOOL). It links the IOP (Expansion 2,
// registers at 0x1f803800) to the TOOL's Linux board, which runs dsnetm. Only
// the PS2 side is emulated; the Linux side is modelled as a host that answers
// signals immediately and drains outgoing packets as soon as they are written.
//
// Register bank (16-bit, stride 4) as seen from the IOP:
//   00 BID   04 RST   08 CPS   0C CPR   10 FST   14 TXC   18 RXC   1C F1C
//   20 IST   24 ISP   28 IER   2C CSI   30 FSI   34 DMA   38 DMA   3C DMA count
//   40 FIFO data port (32-bit)
namespace Pif
{
	static constexpr u32 BASE = 0x1f803800;
	static constexpr u32 END = 0x1f803850;

	// The card only exists when booting a TOOL ROM.
	bool IsPresent();
	// TOOL mode computes every VU status flag: the mVU flag hack (EmuCore/Speedhacks vuFlagHack) is ignored, so
	// debuggers and EE-side reads see the real flags. PCSX2_VU_FLAG_HACK=1 keeps the hack. Set by Reset().
	bool ExactVuFlags();
	__fi bool Handles(u32 addr) { return addr >= BASE && addr < END && IsPresent(); }

	void Reset();

	u8 Read8(u32 addr);
	u16 Read16(u32 addr);
	u32 Read32(u32 addr);
	void Write8(u32 addr, u8 val);
	void Write16(u32 addr, u16 val);
	void Write32(u32 addr, u32 val);

	// IOP DMA channel 5. Returns the number of words moved.
	u32 DmaRead(u32* dst, u32 words);        // card -> IOP RAM
	u32 DmaWrite(const u32* src, u32 words); // IOP RAM -> card

	// Host side: queue one DECI2 packet for the IOP.
	void QueueHostPacket(const std::vector<u8>& packet);

	// The TOOL's Linux board: a TCP link to dsnetm, or dsnetm itself. Configured
	// with the PCSX2_PIF_BRIDGE environment variable:
	//   <port> or listen:[addr:]<port>   accept dsnetm (default addr 127.0.0.1)
	//   connect:<host>:<port>            connect out, retrying every second
	//   dsnetm:[addr:]<port>             be dsnetm: accept its clients (dekko,
	//                                    dsedb, ...), as the TOOL does on 8510
	// The sockets live on their own thread. Everything that touches the card runs
	// on the CPU thread (Poll, SendToHost, OnHandshake, OnVmReset).
	namespace Bridge
	{
		void Start();
		void Stop();
		bool IsEnabled();

		// Moves packets received from the host into the card.
		void Poll();
		// Sends one packet to the host, if connected.
		void SendToHost(const u8* data, size_t size);
		// DECI2DRP's handshake was accepted: the target has finished booting.
		void OnHandshake();
		// The VM was reset (called from Pif::Reset).
		void OnVmReset();
	} // namespace Bridge
} // namespace Pif

// SIF2 for the TOOL: DECI2DRS (IOP) and the EE kernel's DECI2 manager relay
// DECI2 packets for node E over it. Only active with a TOOL ROM outside PS1
// mode; otherwise IOP DMA2 stays with the PS1 GPU (pgif) and ch7 with sif2.cpp.
// Per chunk the sender writes a spec word to IOP RAM (0x3e4 from the IOP, 0x3e0
// from the EE), sets bit 31 of its flag register (SM or MS) and rings the other
// side; the receiver answers with bit 30 and both sides start their DMA.
namespace ToolSif2
{
	bool Active();
	void Reset();

	// IOP DMA2 registers (0x1f8010a0-0x1f8010af). size is 2 or 4.
	u32 IopRead(u32 addr, int size);
	void IopWrite(u32 addr, u32 val, int size);
	// IOP DMA2 completion (IopEvt_SIF2).
	void IopInterrupt();
	// EE ch7 started.
	void EeStart();
	// A write to MS or SM (what says who and how). Setting bit 30 (an accept) drops
	// the FIFO's leftover words.
	void OnFlagWrite(const char* what, u32 value);
	// EE ch7 completion is being delivered (for the trace).
	void EeInterrupt();
	// Called from the bridge poll (about every 110us): rings again for chunks whose
	// doorbell was lost.
	void Poll();
} // namespace ToolSif2

// IOP event that polls the bridge (IopEvt_PIF).
extern void pifInterrupt();
