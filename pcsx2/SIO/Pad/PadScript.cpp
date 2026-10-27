// SPDX-FileCopyrightText: 2002-2026 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

#include "SIO/Pad/PadScript.h"

#include "Counters.h"
#include "SIO/Pad/Pad.h"
#include "SIO/Pad/PadBase.h"
#include "SIO/Pad/PadDualshock2.h"
#include "SIO/Sio.h"

#include <iterator>
#include <optional>
#include <utility>

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

		// The tool's pads, by port (slot 0 only): the data bytes they send.
		std::optional<std::array<u8, Pif::Emup::PAD_DATA_SIZE>> s_owned[2];

		PadDualshock2* Dualshock2(u8 port)
		{
			PadBase* pad = Pad::GetPad(port, 0);
			return pad && pad->GetType() == Pad::ControllerType::DualShock2 ? static_cast<PadDualshock2*>(pad) : nullptr;
		}

		// The data bytes into the pad, past the deadzone, scale and pressure
		// modifier that host input goes through (input recording's way in).
		void Write(PadDualshock2* pad, const std::array<u8, Pif::Emup::PAD_DATA_SIZE>& data)
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
	} // namespace

	Pif::Emup::PadStatus Status(u8 port, u8 slot)
	{
		using namespace Pif::Emup;
		PadStatus s;
		s.port = port;
		s.slot = slot;
		s.owner = port < std::size(s_owned) && slot == 0 && s_owned[port] ? OWNER_EMUP : OWNER_HOST;
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
		for (size_t i = 0; i < std::size(s_pressures); i++)
			s.data[6 + i] = IsButtonBitSet(buttons, s_pressures[i].first) ? pad->GetPressure(s_pressures[i].second) : 0;
		return s;
	}

	Pif::Emup::Result Set(u8 port, u8 slot, const std::array<u8, Pif::Emup::PAD_DATA_SIZE>& data)
	{
		PadDualshock2* pad = port < std::size(s_owned) && slot == 0 ? Dualshock2(port) : nullptr;
		if (!pad)
			return Pif::Emup::RESULT_NOPAD;
		s_owned[port] = data;
		Write(pad, data);
		return Pif::Emup::RESULT_OK;
	}

	void Detach(u8 port, u8 slot)
	{
		if (port >= std::size(s_owned) || slot != 0 || !s_owned[port])
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
			if (PadDualshock2* pad = Dualshock2(port))
				Write(pad, *s_owned[port]);
		}
	}
} // namespace PadScript
