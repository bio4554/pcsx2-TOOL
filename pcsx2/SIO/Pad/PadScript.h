// SPDX-FileCopyrightText: 2002-2026 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

#pragma once

#include "ps2/Iop/PifEmup.h"

// The pads as a host tool sees and drives them over EMUP (crusade TR-32, dekko's
// `pad` commands). CPU thread only.
namespace PadScript
{
	// What the pad in this port and slot presents to a poll right now.
	Pif::Emup::PadStatus Status(u8 port, u8 slot);
} // namespace PadScript
