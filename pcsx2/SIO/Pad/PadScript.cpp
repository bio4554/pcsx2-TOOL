// SPDX-FileCopyrightText: 2002-2026 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

#include "SIO/Pad/PadScript.h"

#include "Counters.h"
#include "SIO/Pad/Pad.h"
#include "SIO/Pad/PadBase.h"
#include "SIO/Pad/PadDualshock2.h"
#include "SIO/Sio.h"
#include "vtlb.h"

#include <iterator>
#include <optional>
#include <utility>
#include <vector>

namespace PadScript
{
	namespace
	{
		// The pressure bytes' order in a poll, as bits of its two button bytes
		// (IsButtonBitSet's numbering) and as the pad's inputs. Keep in step with
		// PadDualshock2::Poll.
		constexpr std::pair<u8, u32> s_pressures[] = {
			{13, PadDualshock2::Inputs::PAD_RIGHT},
			{15, PadDualshock2::Inputs::PAD_LEFT},
			{12, PadDualshock2::Inputs::PAD_UP},
			{14, PadDualshock2::Inputs::PAD_DOWN},
			{4, PadDualshock2::Inputs::PAD_TRIANGLE},
			{5, PadDualshock2::Inputs::PAD_CIRCLE},
			{6, PadDualshock2::Inputs::PAD_CROSS},
			{7, PadDualshock2::Inputs::PAD_SQUARE},
			{2, PadDualshock2::Inputs::PAD_L1},
			{3, PadDualshock2::Inputs::PAD_R1},
			{0, PadDualshock2::Inputs::PAD_L2},
			{1, PadDualshock2::Inputs::PAD_R2},
		};
		// The buttons without a pressure byte.
		constexpr std::pair<u8, u32> s_plain[] = {
			{8, PadDualshock2::Inputs::PAD_SELECT},
			{9, PadDualshock2::Inputs::PAD_L3},
			{10, PadDualshock2::Inputs::PAD_R3},
			{11, PadDualshock2::Inputs::PAD_START},
		};

		using Data = std::array<u8, Pif::Emup::PAD_DATA_SIZE>;

		// A pad with nothing down and the sticks centred.
		constexpr Data s_released = {0xff, 0xff, 0x7f, 0x7f, 0x7f, 0x7f};

		struct Script
		{
			u8 state = Pif::Emup::PLAY_NONE;
			std::vector<Pif::Emup::PlayEntry> entries;
			size_t next = 0;
			u32 frame = 0;
			u32 start = 0;
			std::optional<u32> sync; // the counter's EE address: frames count its changes
			u32 sync_start = 0;
			u32 sync_value = 0;
		};

		// The tool's pads, by port (slot 0 only): the data bytes they send, and
		// their play.
		std::optional<Data> s_owned[2];
		Script s_play[2];

		PadDualshock2* Dualshock2(u8 port)
		{
			PadBase* pad = Pad::GetPad(port, 0);
			return pad && pad->GetType() == Pad::ControllerType::DualShock2 ? static_cast<PadDualshock2*>(pad) : nullptr;
		}

		// Whether the tool can hold the pad in this port: a DualShock 2, or one
		// it has unplugged, whose state waits for it to be plugged back in.
		bool Holdable(u8 port)
		{
			const u8 unifiedSlot = sioConvertPortAndSlotToPad(port, 0);
			return Dualshock2(port) || (Pad::IsUnplugged(unifiedSlot) &&
			                               EmuConfig.Pad.Ports[unifiedSlot].Type == Pad::ControllerType::DualShock2);
		}

		// The data bytes into the pad, past the deadzone, scale and pressure
		// modifier that host input goes through (input recording's way in).
		void Write(PadDualshock2* pad, const Data& data)
		{
			const u32 buttons = (data[0] << 8) | data[1];
			for (size_t i = 0; i < std::size(s_pressures); i++)
			{
				const bool down = IsButtonBitSet(buttons, s_pressures[i].first);
				pad->SetRawPressureButton(s_pressures[i].second, {down, down ? data[6 + i] : 0});
			}
			for (const auto& [bit, input] : s_plain)
			{
				const bool down = IsButtonBitSet(buttons, bit);
				pad->SetRawPressureButton(input, {down, down ? 0xff : 0});
			}
			// Nor held by the tool: the analog button and the pressure modifier.
			pad->SetRawPressureButton(PadDualshock2::Inputs::PAD_ANALOG, {false, 0});
			pad->SetRawPressureButton(PadDualshock2::Inputs::PAD_PRESSURE, {false, 0});
			pad->SetRawAnalogs({data[4], data[5]}, {data[2], data[3]});
		}

		// A u32 of EE memory through the EE's current mapping, without side effects:
		// nullopt for an unmapped page or a register. Needs proper testing with
		// TLB-mapped programs beyond the kernel's default mapping.
		std::optional<u32> ReadEe(u32 addr)
		{
			u32 v;
			if (!vtlb_memSafeReadBytes(addr, &v, sizeof(v)))
				return std::nullopt;
			return v;
		}

		// One vsync of a play: the frame it is now, and the entries due by then. A
		// synced play moves a frame only when its counter has changed since the
		// last vsync (by any amount), so a frame the program drops holds the line;
		// while the counter can't be read, the play waits.
		void Advance(Script& play, Data& data)
		{
			using namespace Pif::Emup;
			const std::optional<u32> counter = play.sync ? ReadEe(*play.sync) : std::nullopt;
			if (play.state == PLAY_ARMED)
			{
				play.state = PLAY_PLAYING;
				play.start = g_FrameCount;
				play.frame = 0;
				if (counter)
					play.sync_start = play.sync_value = *counter;
			}
			else if (play.state == PLAY_PLAYING || play.state == PLAY_DONE)
			{
				if (!play.sync)
					play.frame++;
				else if (counter && *counter != play.sync_value)
				{
					play.frame++;
					play.sync_value = *counter;
				}
			}
			else
			{
				return;
			}
			while (play.next < play.entries.size() && play.entries[play.next].frame <= play.frame)
				data = play.entries[play.next++].data;
			if (play.next == play.entries.size())
				play.state = PLAY_DONE;
		}
	} // namespace

	Pif::Emup::PadStatus Status(u8 port, u8 slot)
	{
		using namespace Pif::Emup;
		PadStatus s;
		s.port = port;
		s.slot = slot;
		s.owner = port < std::size(s_owned) && slot == 0 && s_owned[port] ? OWNER_EMUP : OWNER_HOST;
		s.vsync = g_FrameCount;
		if (port < std::size(s_play) && slot == 0)
		{
			const Script& play = s_play[port];
			s.play.state = play.state;
			s.play.start = play.start;
			s.play.frame = play.frame;
			s.play.entries = static_cast<u32>(play.entries.size());
			s.play.next = static_cast<u32>(play.next);
			s.play.last = play.entries.empty() ? 0 : play.entries.back().frame;
			if (play.sync)
			{
				s.play.flags = PLAY_FLAG_SYNC;
				s.play.sync_addr = *play.sync;
				s.play.sync_start = play.sync_start;
				s.play.sync_value = play.sync_value;
			}
		}

		PadBase* pad = Pad::GetPad(port, slot);
		if (!pad || pad->GetType() == Pad::ControllerType::NotConnected)
		{
			if (Pad::IsUnplugged(sioConvertPortAndSlotToPad(port, slot)))
				s.flags = FLAG_UNPLUGGED;
			return s;
		}
		if (pad->GetType() != Pad::ControllerType::DualShock2)
		{
			s.type = PAD_OTHER;
			return s;
		}

		s.type = PAD_DUALSHOCK2;
		s.id = static_cast<u8>(pad->IsInConfig() ? Pad::Mode::CONFIG : pad->GetMode());
		s.flags = (pad->IsAnalogLightEnabled() ? FLAG_ANALOG_LIGHT : 0) |
		          (pad->IsAnalogLocked() ? FLAG_ANALOG_LOCKED : 0) |
		          (pad->IsInConfig() ? FLAG_CONFIG : 0) |
		          (pad->ejectTicks > 0 ? FLAG_PLUGGING : 0);

		// The data bytes as PadDualshock2::Poll sends them: keep the two in step.
		const u32 buttons = pad->GetButtons();
		const auto [lx, ly] = pad->GetRawLeftAnalog();
		const auto [rx, ry] = pad->GetRawRightAnalog();
		s.data[0] = static_cast<u8>((buttons >> 8) & 0xff);
		s.data[1] = static_cast<u8>(buttons & 0xff);
		s.data[2] = rx;
		s.data[3] = ry;
		s.data[4] = lx;
		s.data[5] = ly;
		for (size_t i = 0; i < std::size(s_pressures); i++)
			s.data[6 + i] = IsButtonBitSet(buttons, s_pressures[i].first) ? pad->GetPressure(s_pressures[i].second) : 0;
		return s;
	}

	Pif::Emup::Result Set(u8 port, u8 slot, const Data& data)
	{
		if (port >= std::size(s_owned) || slot != 0 || !Holdable(port))
			return Pif::Emup::RESULT_NOPAD;
		s_play[port] = {};
		s_owned[port] = data;
		if (PadDualshock2* pad = Dualshock2(port))
			Write(pad, data);
		return Pif::Emup::RESULT_OK;
	}

	Pif::Emup::Result Play(const Pif::Emup::PadPlayPart& part)
	{
		using namespace Pif::Emup;
		const u8 port = part.pad.port;
		if (port >= std::size(s_play) || part.pad.slot != 0 || !Holdable(port))
			return RESULT_NOPAD;
		Script& play = s_play[port];
		const bool first = part.flags & PLAY_FIRST;
		if (!first && play.state != PLAY_LOADING)
			return RESULT_INVAL; // a later part with no play begun
		const size_t loaded = first ? 0 : play.entries.size();
		if (loaded + part.entries.size() > PLAY_MAX_ENTRIES)
			return RESULT_INVAL;
		std::optional<u32> prev;
		if (loaded > 0)
			prev = play.entries.back().frame;
		for (const PlayEntry& e : part.entries)
		{
			if (prev && e.frame <= *prev)
				return RESULT_INVAL;
			prev = e.frame;
		}
		if ((part.flags & PLAY_START) && loaded == 0 && part.entries.empty())
			return RESULT_INVAL; // nothing to play
		if (part.flags & PLAY_SYNC)
		{
			if (!(part.flags & PLAY_START) || (part.sync_addr & 3))
				return RESULT_INVAL;
			if (!ReadEe(part.sync_addr))
				return RESULT_NOMEM;
		}

		// A new play leaves the pad alone until it starts.
		if (first)
			play = {PLAY_LOADING};
		play.entries.insert(play.entries.end(), part.entries.begin(), part.entries.end());
		if (part.flags & PLAY_START)
		{
			// Frame 0 is the next Apply: this request ran from the CPU thread's
			// queue, which Host::PumpMessagesOnCPUThread empties just before it in
			// the same vsync (or before the first vsync after a pause).
			play.state = PLAY_ARMED;
			if (part.flags & PLAY_SYNC)
				play.sync = part.sync_addr;
			s_owned[port] = s_released;
			if (PadDualshock2* pad = Dualshock2(port))
				Write(pad, s_released);
		}
		return RESULT_OK;
	}

	Pif::Emup::Result Plug(u8 port, u8 slot, bool plugged)
	{
		if (port >= std::size(s_owned) || slot != 0)
			return Pif::Emup::RESULT_NOPAD;
		const u8 unifiedSlot = sioConvertPortAndSlotToPad(port, slot);
		if (!plugged)
		{
			Pad::Unplug(unifiedSlot);
			return Pif::Emup::RESULT_OK;
		}
		if (EmuConfig.Pad.Ports[unifiedSlot].Type == Pad::ControllerType::NotConnected)
			return Pif::Emup::RESULT_NOPAD;
		Pad::Plug(unifiedSlot);
		// The new pad sends the tool's state from its first answer.
		if (PadDualshock2* pad = Dualshock2(port); pad && s_owned[port])
			Write(pad, *s_owned[port]);
		return Pif::Emup::RESULT_OK;
	}

	Pif::Emup::Result Analog(u8 port, u8 slot)
	{
		PadDualshock2* pad = slot == 0 ? Dualshock2(port) : nullptr;
		if (!pad)
			return Pif::Emup::RESULT_NOPAD;
		// The host's own way in (its "Analog Toggle" binding), which keeps to the
		// mode lock. Neither the tool's state nor Write holds the button, so the
		// release is all that's left of it.
		pad->Set(PadDualshock2::Inputs::PAD_ANALOG, 1.0f);
		pad->Set(PadDualshock2::Inputs::PAD_ANALOG, 0.0f);
		return Pif::Emup::RESULT_OK;
	}

	void Stop(u8 port, u8 slot)
	{
		if (port < std::size(s_play) && slot == 0)
			s_play[port] = {};
	}

	void Detach(u8 port, u8 slot)
	{
		if (port >= std::size(s_owned) || slot != 0)
			return;
		s_play[port] = {};
		if (!s_owned[port])
			return;
		s_owned[port].reset();
		Pad::ResetControllerInputs(sioConvertPortAndSlotToPad(port, 0));
	}

	bool Owns(u32 unifiedSlot)
	{
		const auto [port, slot] = sioConvertPadToPortAndSlot(unifiedSlot);
		return port < std::size(s_owned) && slot == 0 && s_owned[port];
	}

	void Apply()
	{
		for (u8 port = 0; port < std::size(s_owned); port++)
		{
			if (!s_owned[port])
				continue;
			Advance(s_play[port], *s_owned[port]);
			if (PadDualshock2* pad = Dualshock2(port))
				Write(pad, *s_owned[port]);
		}
	}
} // namespace PadScript
