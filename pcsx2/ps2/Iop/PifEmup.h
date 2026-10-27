// SPDX-FileCopyrightText: 2002-2026 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

#pragma once

#include "common/Pcsx2Types.h"

#include <array>
#include <optional>
#include <vector>

namespace Pif::Emup
{
	// EMUP: the emulator's own DECI2 protocol, for what a host tool can ask of the
	// emulator that the TOOL has no protocol for (the pad first, crusade TR-32).
	// The built-in dsnetm answers it itself (H to H); the TOOL's dsnetm, or the
	// WSL one, bounces it with a DCMP NOPROTO, which is how a client tells it's
	// not here. 0xE001 is in DECI2's "local use for licensee" range
	// (0xe000-0xefff, deci2_3.0).
	//
	// Body, little-endian: u8 code, u8 result, u16 reserved, u32 id, then the
	// code's payload. A reply's code is the request's plus one; it echoes the
	// id. Codes and payloads are in ps2-research docs/05 ("EMUP"); dekko's
	// src/deci2/emup.h is the other end.
	constexpr u16 PROTO = 0xE001;
	constexpr u16 VERSION = 1;
	constexpr size_t HEADER_SIZE = 8; // the body's own header, after DECI2's

	enum Code : u8
	{
		CODE_VERSION = 0x00,
		CODE_VERSIONR = 0x01,
		CODE_PAD_STATUS = 0x10,
		CODE_PAD_STATUSR = 0x11,
		CODE_PAD_SET = 0x12,
		CODE_PAD_SETR = 0x13,
		CODE_PAD_DETACH = 0x14,
		CODE_PAD_DETACHR = 0x15,
		CODE_BAD = 0xff, // the reply to a body too short to have a code and id
	};

	enum Result : u8
	{
		RESULT_OK = 0,
		RESULT_INVAL = 1, // a payload too short, or a value out of range
		RESULT_NOCODE = 2, // a code this build doesn't know
		RESULT_NOVM = 3, // nothing is running to ask
		RESULT_NOPAD = 4, // the port has no pad the request can drive (PAD_SET: a DualShock 2)
	};

	// VERSIONR's feature bits: what this build answers.
	constexpr u32 FEATURE_PAD_STATUS = 1u << 0;
	constexpr u32 FEATURE_PAD_SET = 1u << 1; // PAD_SET and PAD_DETACH
	constexpr u32 FEATURES = FEATURE_PAD_STATUS | FEATURE_PAD_SET;

	struct Request
	{
		u8 code;
		u32 id;
		std::vector<u8> payload;
	};

	// The request in a whole EMUP packet (DECI2 header included); nullopt if the
	// body is too short for its header.
	std::optional<Request> Parse(const std::vector<u8>& pkt);

	// A whole reply packet (H to H) to a request.
	std::vector<u8> Reply(const Request& req, u8 result, const std::vector<u8>& payload = {});
	// The reply to a packet Parse refused.
	std::vector<u8> BadReply();

	std::vector<u8> VersionPayload();

	// PAD_STATUS and PAD_DETACH ask with u8 port (0: port 1), u8 slot (0, no
	// multitap), u16 0. PAD_DETACH hands the pad back to the host's input.
	struct PadRequest
	{
		u8 port;
		u8 slot;
	};
	std::optional<PadRequest> ParsePad(const Request& req);

	// What the pad in a port presents, as PAD_STATUSR carries it.
	enum PadType : u8
	{
		PAD_NONE = 0,
		PAD_DUALSHOCK2 = 1,
		PAD_OTHER = 0x80, // one of PCSX2's other pads (guitar, jogcon, ...)
	};
	enum PadOwner : u8
	{
		OWNER_HOST = 0, // the window's keyboard and host controllers
		OWNER_EMUP = 1, // a host tool, over EMUP (PAD_SET)
	};
	enum PadFlags : u8
	{
		FLAG_ANALOG_LIGHT = 1 << 0,
		FLAG_ANALOG_LOCKED = 1 << 1,
		FLAG_CONFIG = 1 << 2, // in config mode (escape command 0x43)
		FLAG_PLUGGING = 1 << 3, // still "being plugged in" after a swap (ejectTicks)
	};
	constexpr size_t PAD_DATA_SIZE = 18;

	struct PadStatus
	{
		u8 port = 0;
		u8 slot = 0;
		u8 type = PAD_NONE;
		u8 owner = OWNER_HOST;
		u8 id = 0xff; // the mode byte a poll answers with: 0x41, 0x73, 0x79, 0xf3; 0xff none
		u8 flags = 0;
		u32 vsync = 0; // vsyncs since the VM's last reset
		// A poll's data bytes as the pad would send them now: the buttons (2,
		// active low), rx, ry, lx, ly, then the 12 pressures in poll order (right,
		// left, up, down, triangle, circle, cross, square, L1, R1, L2, R2). Which of
		// them a poll sends depends on the mode (id).
		std::array<u8, PAD_DATA_SIZE> data = {};
	};
	std::vector<u8> PadStatusPayload(const PadStatus& s);

	// PAD_SET asks with a PadRequest, then the 18 data bytes the pad is to send
	// from now on (PadStatus::data's layout; a pressure goes out only while its
	// button is down). The pad is then the tool's: the host's input to that port
	// is dropped until PAD_DETACH. PAD_SETR and PAD_DETACHR carry the PadStatus
	// after the change.
	struct PadSet
	{
		PadRequest pad;
		std::array<u8, PAD_DATA_SIZE> data;
	};
	std::optional<PadSet> ParsePadSet(const Request& req);
} // namespace Pif::Emup
