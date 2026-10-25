// SPDX-FileCopyrightText: 2002-2026 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

#pragma once

#include "common/Pcsx2Types.h"

// TOOL mode's drive timing: a model of a 2000-2003 retail DVD drive from the figures Sony gave
// developers (CAV rate by radius, seeks by distance and radius, rotational latency, layer
// changes). It replaces the stock rates and seek times while a TOOL BIOS runs; the read-ahead
// and DMA machinery in CDVD.cpp stays as it is. Its numbers are Sony's, not measured on a
// drive here, so it needs proper testing against real hardware.
//
// Switches (environment, read at reset):
//  - PCSX2_CDVD_TIMING=stock   PCSX2's own timing instead;
//  - PCSX2_CDVD_TIMING=best    the documented best case (4.8 MB/s at the outer edge);
//  - PCSX2_CDVD_DL0_DERATE=x   the rate factor on layer 0 of a dual-layer disc (default 0.8);
//  - PCSX2_CDVD_LOG=<file>     a line per drive command: time, sectors, spindle, rate, seek.
namespace ToolCdvd
{
	struct Disc
	{
		bool dvd;
		bool dual;        // a dual-layer DVD (PTP): both layers start at the inner edge
		u32 layer1Start;  // its first layer-1 sector
		u32 maxSector;    // the image's end, for the outer-edge warning
	};

	// Whether the model times the drive: TOOL mode, and not PCSX2_CDVD_TIMING=stock.
	bool Active();
	void Reset();

	// IOP cycles per sector for a read starting at lsn, in the spindle mode the command gave.
	u32 SectorCycles(const Disc& disc, int spindle, u32 lsn);
	// IOP cycles per revolution at lsn.
	u32 RotationCycles(const Disc& disc, int spindle, u32 lsn);

	enum class SeekKind
	{
		Lens,  // inside the lens range: the lens moves, the sled doesn't
		Sled,  // the sled moves
	};
	struct Seek
	{
		u32 cycles;  // the move, rotational latency included (Sony's figures), and the start of the
		             // read's 16-sector ECC block read through up to its first sector (DVD)
		SeekKind kind;
	};
	Seek SeekCycles(const Disc& disc, int spindle, u32 from, u32 to);

	// A command's fixed setup in the drive, paid when it waits for the disc to come round.
	u32 SetupCycles();

	// The command log (PCSX2_CDVD_LOG) and the outer-edge warning. command: read, seek or standby; how: what timed it
	// (read-ahead, wait, seek-lens, seek-sled, spin-up); cycles: until the first sector.
	void LogCommand(const Disc& disc, const char* command, const char* how, u32 from, u32 lsn, u32 sectors, int spindle,
		u32 sector_cycles, u32 cycles);
} // namespace ToolCdvd
