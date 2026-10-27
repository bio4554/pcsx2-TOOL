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
		CODE_PAD_PLAY = 0x16,
		CODE_PAD_PLAYR = 0x17,
		CODE_PAD_STOP = 0x18,
		CODE_PAD_STOPR = 0x19,
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
	constexpr u32 FEATURE_PAD_PLAY = 1u << 2; // PAD_PLAY and PAD_STOP, and PAD_STATUSR's play fields
	constexpr u32 FEATURES = FEATURE_PAD_STATUS | FEATURE_PAD_SET | FEATURE_PAD_PLAY;

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

	// Where a pad's play (PAD_PLAY) is.
	enum PlayState : u8
	{
		PLAY_NONE = 0,
		PLAY_LOADING = 1, // parts sent, not started
		PLAY_ARMED = 2, // started: frame 0 is the next vsync
		PLAY_PLAYING = 3,
		PLAY_DONE = 4, // past its last entry, which the pad holds
	};

	struct PadPlay
	{
		u8 state = PLAY_NONE;
		u32 start = 0; // the vsync count (PadStatus::vsync) at frame 0
		u32 frame = 0; // the frame now applied: vsyncs since frame 0
		u32 entries = 0; // loaded
		u32 next = 0; // entries applied so far
		u32 last = 0; // the last entry's frame
	};

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
		PadPlay play; // after the data bytes: u8 state, u8 0, u16 0, then the five u32s
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

	// PAD_PLAY loads a play in parts, then starts it: u8 port, u8 slot, u8
	// flags, u8 0, u32 count, then count entries of u32 frame, the 18 data bytes
	// and u16 0. Frames rise strictly, across parts too. PLAY_FIRST begins a new
	// play (dropping one loaded but not started); PLAY_START marks the last part:
	// frame 0 is the first vsync after it. Until its first entry the pad is
	// released; after its last, the pad holds that entry. The pad is the tool's
	// from the start, as with PAD_SET. PAD_PLAYR carries the PadStatus.
	//
	// PAD_STOP (a PadRequest) ends a play, loaded or playing; the pad holds what
	// it sends now and stays the tool's. PAD_STOPR carries the PadStatus.
	constexpr u8 PLAY_FIRST = 1 << 0;
	constexpr u8 PLAY_START = 1 << 1;
	constexpr size_t PLAY_ENTRY_SIZE = 24;
	constexpr size_t PLAY_MAX_ENTRIES = 1u << 20; // a play's limit, over all its parts

	struct PlayEntry
	{
		u32 frame;
		std::array<u8, PAD_DATA_SIZE> data;
	};
	struct PadPlayPart
	{
		PadRequest pad;
		u8 flags;
		std::vector<PlayEntry> entries;
	};
	// nullopt if the payload's count doesn't match its length.
	std::optional<PadPlayPart> ParsePadPlay(const Request& req);
} // namespace Pif::Emup
