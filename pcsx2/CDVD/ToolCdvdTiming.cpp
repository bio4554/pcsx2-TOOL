// SPDX-FileCopyrightText: 2002-2026 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

#include "CDVD/ToolCdvdTiming.h"
#include "CDVD/CDVDcommon.h"
#include "Common.h"
#include "R3000A.h"
#include "ps2/Iop/Pif.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

// The figures (developer conference decks, 2000-2003; SCEE's dvdspeed sample, 2000):
//  - CAV rate rising with radius, 2.1 MB/s at the inner edge to 4.2 MB/s (observed, realistic) or
//    4.6-5.0 (best case) at the outer; CD 1.2 / 2.4 MB/s. Radius from standard DVD geometry, 24 to
//    58 mm over 2,295,104 sectors a layer, which gives dvdspeed's SpinNom curve (2.2, 2.9, 3.59,
//    4.27 MB/s at 0, 1, 2, 3 GB);
//  - SpinMax (= SpinStm in this SDK: cdvdman sends it as CLV) flat at about 2.2 MB/s (dvdspeed);
//  - seeks: 25-40 ms inside the lens range (14,764 sectors at the inner radius, 35,000 at the outer;
//    13,360 on a dual-layer disc), growing to 300 ms at full stroke (square root shape assumed);
//    CD 4,371 sectors, 30-400 ms;
//  - about 5 ms of setup plus rotational latency for a read the drive has to wait for;
//  - a layer change, 60-80 ms; layer 0 of a dual-layer disc less dependable (derate assumed).
// MB here is 10^6 bytes.

namespace ToolCdvd
{
	namespace
	{
		struct Media
		{
			double r0, r1;          // mm: the first and the last sector's radius
			double fullSectors;     // sectors a full layer holds
			double sectorLength;    // mm of track a 2048-byte sector takes
			double rateIn, rateOut; // MB/s, CAV at the inner edge and the most at the outer
			double lensIn, lensOut; // sectors: the lens range at the inner and the outer radius
			double seekLensMin, seekLensMax, seekFull; // ms
		};

		// DVD 1x is 11.08 Mbit/s of user data at 3.49 m/s; CD 1x is 75 sectors/s at about 1.3 m/s.
		constexpr Media s_dvd = {24.0, 58.0, 2295104.0, 3490.0 * 2048.0 / 1385000.0, 2.1, 4.2, 14764.0, 35000.0, 25.0, 40.0, 300.0};
		constexpr Media s_cd = {25.0, 58.0, 360000.0, 1300.0 / 75.0, 1.2, 2.4, 4371.0, 11500.0, 30.0, 45.0, 400.0};

		constexpr double DVD_CLV_MAX = 2.2;   // MB/s: SpinMax/SpinStm (dvdspeed)
		constexpr double DVD_1X = 1.385;      // MB/s
		constexpr double CD_1X = 0.1536;      // MB/s
		constexpr double DUAL_LENS = 13360.0 / 14764.0;
		constexpr double LAYER_CHANGE_MS = 70.0;
		constexpr double SETUP_MS = 5.0;
		constexpr double OUTER_EDGE = 0.99;   // of the image: Sony's "don't seek to the extreme outer edge"

		bool s_active = false;
		double s_rateOut[2] = {s_cd.rateOut, s_dvd.rateOut};
		double s_dl0Derate = 0.8;
		std::FILE* s_log = nullptr;
		u64 s_clock = 0;     // IOP cycles, 64-bit (psxRegs.cycle wraps every 116 s)
		u32 s_lastCycle = 0;
		bool s_warnedOuterEdge = false;

		const Media& MediaOf(const Disc& disc) { return disc.dvd ? s_dvd : s_cd; }

		u32 Cycles(double ms) { return static_cast<u32>(std::lround(ms * static_cast<double>(PSXCLK) / 1000.0)); }

		bool OnLayer1(const Disc& disc, u32 lsn) { return disc.dual && disc.layer1Start && lsn >= disc.layer1Start; }

		// The sector's place in its layer (PTP: layer 1 starts at the inner edge too).
		double InLayer(const Disc& disc, u32 lsn) { return OnLayer1(disc, lsn) ? lsn - disc.layer1Start : lsn; }

		double Radius(const Media& m, double in_layer)
		{
			const double f = std::clamp(in_layer / m.fullSectors, 0.0, 1.0);
			return std::sqrt(m.r0 * m.r0 + (m.r1 * m.r1 - m.r0 * m.r0) * f);
		}

		// MB/s for a read at lsn.
		double Rate(const Disc& disc, int spindle, u32 lsn)
		{
			const Media& m = MediaOf(disc);
			const int code = spindle & CDVD_SPINDLE_SPEED;
			const double cav = std::min(m.rateIn * Radius(m, InLayer(disc, lsn)) / m.r0, s_rateOut[disc.dvd]);
			double rate;
			if (disc.dvd)
			{
				// cdvdman: SpinNom is CAV at code 3 (or 2 with an old mechacon), SpinMax/SpinStm CLV code 2.
				if (spindle & CDVD_SPINDLE_CAV)
					rate = code >= 3 ? cav : std::min(cav, DVD_CLV_MAX);
				else
					rate = code <= 1 ? DVD_1X : DVD_CLV_MAX;
				if (disc.dual && !OnLayer1(disc, lsn))
					rate *= s_dl0Derate;
			}
			else
			{
				// SpinNom is CAV at code 5 (24x), SpinMax/SpinStm CLV code 4 (12x): CLV can go no faster than
				// the inner edge's CAV rate.
				if (spindle & CDVD_SPINDLE_CAV)
					rate = cav;
				else
					rate = std::min(CD_1X * (code <= 1 ? 1 : code == 2 ? 2 : code == 3 ? 4 : 12), m.rateIn);
			}
			return rate;
		}

		double SectorMs(const Disc& disc, int spindle, u32 lsn) { return 2048.0 / (Rate(disc, spindle, lsn) * 1000.0); }

		u64 Now()
		{
			s_clock += psxRegs.cycle - s_lastCycle;
			s_lastCycle = psxRegs.cycle;
			return s_clock;
		}
	} // namespace

	bool Active() { return s_active && Pif::IsPresent(); }

	void Reset()
	{
		const char* timing = std::getenv("PCSX2_CDVD_TIMING");
		s_active = !(timing && std::strcmp(timing, "stock") == 0);
		s_rateOut[1] = (timing && std::strcmp(timing, "best") == 0) ? 4.8 : s_dvd.rateOut;
		const char* derate = std::getenv("PCSX2_CDVD_DL0_DERATE");
		s_dl0Derate = derate ? std::clamp(std::atof(derate), 0.1, 1.0) : 0.8;
		s_clock = 0;
		s_lastCycle = psxRegs.cycle;
		s_warnedOuterEdge = false;
		if (s_log)
		{
			std::fclose(s_log);
			s_log = nullptr;
		}
		if (!Pif::IsPresent())
			return;
		if (const char* log = std::getenv("PCSX2_CDVD_LOG"); log && log[0] && s_active)
		{
			s_log = std::fopen(log, "w");
			if (s_log)
				std::fprintf(s_log, "# ms command how from lsn sectors spindle MB/s ms_per_sector first_sector_ms about_done_ms\n");
			else
				Console.Warning("CDVD: can't write the command log %s", log);
		}
		if (s_active)
			Console.WriteLn("CDVD: TOOL mode: the drive model from Sony's figures (%s; PCSX2_CDVD_TIMING=stock for PCSX2's)",
				s_rateOut[1] > s_dvd.rateOut ? "best case" : "realistic");
	}

	u32 SectorCycles(const Disc& disc, int spindle, u32 lsn) { return Cycles(SectorMs(disc, spindle, lsn)); }

	u32 RotationCycles(const Disc& disc, int spindle, u32 lsn)
	{
		const Media& m = MediaOf(disc);
		const double sectors_per_turn = 2.0 * 3.14159265358979 * Radius(m, InLayer(disc, lsn)) / m.sectorLength;
		return Cycles(sectors_per_turn * SectorMs(disc, spindle, lsn));
	}

	Seek SeekCycles(const Disc& disc, int spindle, u32 from, u32 to)
	{
		const Media& m = MediaOf(disc);
		const double at = InLayer(disc, from);
		const double distance = std::abs(InLayer(disc, to) - at);
		const double r = Radius(m, at);
		double lens = m.lensIn + (m.lensOut - m.lensIn) * (r - m.r0) / (m.r1 - m.r0);
		if (disc.dual)
			lens *= DUAL_LENS;

		Seek seek;
		double ms;
		if (distance < lens)
		{
			ms = m.seekLensMin + (m.seekLensMax - m.seekLensMin) * distance / lens;
			seek.kind = SeekKind::Lens;
		}
		else
		{
			const double stroke = std::clamp((distance - lens) / (m.fullSectors - lens), 0.0, 1.0);
			ms = m.seekLensMax + (m.seekFull - m.seekLensMax) * std::sqrt(stroke);
			seek.kind = SeekKind::Sled;
		}
		if (OnLayer1(disc, from) != OnLayer1(disc, to))
			ms += LAYER_CHANGE_MS;
		// A DVD reads whole 16-sector ECC blocks: the read starts at its block's first sector.
		if (disc.dvd)
			ms += (to & 15) * SectorMs(disc, spindle, to);
		seek.cycles = Cycles(ms);
		return seek;
	}

	u32 SetupCycles() { return Cycles(SETUP_MS); }

	void LogCommand(const Disc& disc, const char* command, const char* how, u32 from, u32 lsn, u32 sectors, int spindle,
		u32 sector_cycles, u32 cycles)
	{
		const u64 now = Now();
		const bool outer = disc.maxSector && sectors && lsn + sectors > static_cast<u32>(disc.maxSector * OUTER_EDGE);
		if (outer && !s_warnedOuterEdge)
		{
			s_warnedOuterEdge = true;
			Console.Warning("CDVD: a read of sectors %u-%u touches the last 1%% of the disc (%u sectors): on a real drive the "
							"head may overshoot there and not recover; Sony's advice is padding after the last data",
				lsn, lsn + sectors - 1, disc.maxSector);
		}
		if (!s_log)
			return;
		const double ms_per_cycle = 1000.0 / static_cast<double>(PSXCLK);
		const double first = cycles * ms_per_cycle;
		const double per_sector = sector_cycles * ms_per_cycle;
		std::fprintf(s_log, "%.3f %s %s%s %u %u %u 0x%02x %.3f %.4f %.3f %.3f\n", static_cast<double>(now) * ms_per_cycle,
			command, how, outer ? ",outer-edge" : "", from, lsn, sectors, spindle, Rate(disc, spindle, lsn), per_sector, first,
			sectors ? first + (sectors - 1) * per_sector : first);
		std::fflush(s_log);
	}
} // namespace ToolCdvd
