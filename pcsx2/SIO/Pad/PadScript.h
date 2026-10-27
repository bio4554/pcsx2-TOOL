// SPDX-FileCopyrightText: 2002-2026 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

#pragma once

#include "ps2/Iop/PifEmup.h"

// The pads as a host tool sees and drives them over EMUP (crusade TR-32, dekko's
// `pad` commands). CPU thread only, like all of the pads' input: host input
// arrives there too (InputManager::PollSources, and the window's key events
// through Host::RunOnCPUThread).
//
// A port the tool has set is its own until it detaches: Pad::SetControllerState
// and the other host paths leave the pad alone (override, not merge), and Apply
// writes the tool's state into the pad at each vsync, so it holds whatever the
// window's focus, a settings reload or a reset does to the pad. Only ports 1
// and 2 without a multitap.
namespace PadScript
{
	// What the pad in this port and slot presents to a poll right now.
	Pif::Emup::PadStatus Status(u8 port, u8 slot);

	// Takes the pad and has it send these data bytes (PadStatus::data's layout)
	// from now on, ending a play; RESULT_NOPAD if the port has no DualShock 2.
	Pif::Emup::Result Set(u8 port, u8 slot, const std::array<u8, Pif::Emup::PAD_DATA_SIZE>& data);
	// Hands the pad back to the host: released, until the host's next input.
	// Ends a play.
	void Detach(u8 port, u8 slot);

	// A part of a play (PAD_PLAY): loads it, and the last part starts it, frame
	// 0 at the next Apply. Frames count vsyncs, so a play keeps to the program's
	// time whatever the host's speed. RESULT_NOPAD if the port has no DualShock
	// 2, RESULT_INVAL for a part out of order. Set and Detach end a play. With
	// PLAY_SYNC, frames count the changes of a u32 the program keeps (its frame
	// counter), read at each vsync; RESULT_NOMEM if nothing's readable there.
	Pif::Emup::Result Play(const Pif::Emup::PadPlayPart& part);
	// Ends a play; the pad holds what it sends now, and stays the tool's.
	void Stop(u8 port, u8 slot);

	// Whether the tool has this pad (Pad's unified slot numbering).
	bool Owns(u32 unifiedSlot);
	// Writes the tool's state into its pads; once per vsync, after host input.
	void Apply();
} // namespace PadScript
