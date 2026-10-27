// SPDX-FileCopyrightText: 2002-2026 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

#include "SIO/Pad/PadScript.h"

#include "Counters.h"
#include "SIO/Pad/Pad.h"
#include "SIO/Pad/PadBase.h"
#include "SIO/Pad/PadDualshock2.h"

#include <iterator>
#include <utility>

namespace PadScript
{
	Pif::Emup::PadStatus Status(u8 port, u8 slot)
	{
		using namespace Pif::Emup;
		PadStatus s;
		s.port = port;
		s.slot = slot;
		s.vsync = g_FrameCount;

		PadBase* pad = Pad::GetPad(port, slot);
		if (!pad || pad->GetType() == Pad::ControllerType::NotConnected)
			return s;
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
		static constexpr std::pair<u8, u32> pressures[] = {
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
		for (size_t i = 0; i < std::size(pressures); i++)
			s.data[6 + i] = IsButtonBitSet(buttons, pressures[i].first) ? pad->GetPressure(pressures[i].second) : 0;
		return s;
	}
} // namespace PadScript
