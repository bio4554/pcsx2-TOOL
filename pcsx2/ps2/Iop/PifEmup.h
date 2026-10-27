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
		CODE_PAD_PLUG = 0x1a,
		CODE_PAD_PLUGR = 0x1b,
		CODE_PAD_ANALOG = 0x1c,
		CODE_PAD_ANALOGR = 0x1d,
		CODE_PAD_TYPE = 0x1e,
		CODE_PAD_TYPER = 0x1f,
		CODE_BAD = 0xff, // the reply to a body too short to have a code and id
	};

	enum Result : u8
	{
		RESULT_OK = 0,
		RESULT_INVAL = 1, // a payload too short, or a value out of range
		RESULT_NOCODE = 2, // a code this build doesn't know
		RESULT_NOVM = 3, // nothing is running to ask
		RESULT_NOPAD = 4, // the port has no pad the request can drive (PAD_SET: a DualShock 2)
		RESULT_NOMEM = 5, // an EE address with nothing readable there (PAD_PLAY's sync counter)
	};

	// VERSIONR's feature bits: what this build answers.
	constexpr u32 FEATURE_PAD_STATUS = 1u << 0;
	constexpr u32 FEATURE_PAD_SET = 1u << 1; // PAD_SET and PAD_DETACH
	constexpr u32 FEATURE_PAD_PLAY = 1u << 2; // PAD_PLAY and PAD_STOP, and PAD_STATUSR's play fields
	constexpr u32 FEATURE_PAD_SYNC = 1u << 3; // PLAY_SYNC, and PAD_STATUSR's sync fields
	constexpr u32 FEATURE_PAD_PLUG = 1u << 4; // PAD_PLUG and PAD_ANALOG, and FLAG_UNPLUGGED
	constexpr u32 FEATURE_PAD_TYPE = 1u << 5; // PAD_TYPE, PAD_STATUSR's model and FLAG_SWAPPING
	constexpr u32 FEATURES = FEATURE_PAD_STATUS | FEATURE_PAD_SET | FEATURE_PAD_PLAY | FEATURE_PAD_SYNC | FEATURE_PAD_PLUG |
	                         FEATURE_PAD_TYPE;

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
		PAD_DUALSHOCK2 = 1, // PCSX2's DualShock 2, presenting PadStatus::model
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
		FLAG_UNPLUGGED = 1 << 4, // a host tool has unplugged the pad (PAD_PLUG): type is PAD_NONE
		FLAG_SWAPPING = 1 << 5, // out for a PAD_TYPE swap: the new pad goes in by itself (with FLAG_UNPLUGGED)
	};
	// Which pad a DualShock 2 port presents (PAD_TYPE; Pad::DualShockModel's values).
	enum PadModel : u8
	{
		MODEL_DUALSHOCK2 = 0,
		MODEL_DUALSHOCK = 1,
		MODEL_DIGITAL = 2,
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

	// PadPlay::flags
	constexpr u8 PLAY_FLAG_SYNC = 1 << 0; // frames count the changes of a u32 in EE memory, not vsyncs

	struct PadPlay
	{
		u8 state = PLAY_NONE;
		u8 flags = 0;
		u32 start = 0; // the vsync count (PadStatus::vsync) at frame 0
		u32 frame = 0; // the frame now applied: vsyncs (or the sync counter's changes) since frame 0
		u32 entries = 0; // loaded
		u32 next = 0; // entries applied so far
		u32 last = 0; // the last entry's frame
		u32 sync_addr = 0; // PLAY_FLAG_SYNC: the counter's EE address
		u32 sync_start = 0; // its value at frame 0
		u32 sync_value = 0; // its value at the last vsync
	};

	struct PadStatus
	{
		u8 port = 0;
		u8 slot = 0;
		u8 type = PAD_NONE;
		u8 owner = OWNER_HOST;
		u8 id = 0xff; // the mode byte a poll answers with: 0x41, 0x73, 0x79, 0xf3; 0xff none
		u8 flags = 0;
		// The model the port's DualShock 2 presents, or will when it's plugged
		// in; then u8 0. Older builds send 0 (MODEL_DUALSHOCK2), which they are.
		u8 model = MODEL_DUALSHOCK2;
		u32 vsync = 0; // vsyncs since the VM's last reset
		// A poll's data bytes as the pad would send them now: the buttons (2,
		// active low), rx, ry, lx, ly, then the 12 pressures in poll order (right,
		// left, up, down, triangle, circle, cross, square, L1, R1, L2, R2). Which of
		// them a poll sends depends on the mode (id).
		std::array<u8, PAD_DATA_SIZE> data = {};
		// After the data bytes: u8 state, u8 flags, u16 0, then start, frame,
		// entries, next, last, sync_addr, sync_start and sync_value (u32s).
		PadPlay play;
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
	// PLAY_SYNC, on the PLAY_START part only, puts a u32 EE address (aligned)
	// between count and the entries: the play's frames then count the changes of
	// the u32 there, read at each vsync, so a program's own frame counter can pace
	// the play and a frame it drops holds the line. Frame 0 is still the first
	// vsync after the request. RESULT_NOMEM if nothing is readable there.
	//
	// PAD_STOP (a PadRequest) ends a play, loaded or playing; the pad holds what
	// it sends now and stays the tool's. PAD_STOPR carries the PadStatus.
	constexpr u8 PLAY_FIRST = 1 << 0;
	constexpr u8 PLAY_START = 1 << 1;
	constexpr u8 PLAY_SYNC = 1 << 2;
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
		u32 sync_addr; // PLAY_SYNC
		std::vector<PlayEntry> entries;
	};
	// nullopt if the payload's count doesn't match its length.
	std::optional<PadPlayPart> ParsePadPlay(const Request& req);

	// PAD_PLUG asks with u8 port, u8 slot, u8 plugged, u8 0: plugged 0 takes the
	// pad out of the port (the program sees the port empty), 1 puts the pad the
	// emulator's settings give the port back in, as from power-on: digital and
	// the analog light off, so the program's driver finds it and sets it up
	// again (the program sees the gap only if it polled while the port was
	// empty). Unplugging an empty port, or
	// plugging a full one, changes nothing; RESULT_NOPAD if the settings give the
	// port no pad. The tool's hold on the pad (PAD_SET, PAD_PLAY) is the port's,
	// so it survives: a play goes on counting frames while the port is empty.
	// PAD_DETACH doesn't plug a pad back in.
	//
	// PAD_ANALOG (a PadRequest) presses and releases the pad's analog button,
	// whoever holds the pad. Like a real one, the press switches between digital
	// and analog only while the program hasn't locked the mode (FLAG_ANALOG_LOCKED);
	// RESULT_NOPAD with no DualShock 2 in the port. PAD_PLUGR and PAD_ANALOGR
	// carry the PadStatus after the change.
	struct PadPlug
	{
		PadRequest pad;
		bool plugged;
	};
	std::optional<PadPlug> ParsePadPlug(const Request& req);

	// PAD_TYPE asks with u8 port, u8 slot, u8 model (PadModel), u8 0: the pad the
	// port presents. A pad in the port is swapped: taken out now, and a new one
	// of the model (as from power-on) put in PAD_TYPE_SWAP_VSYNCS later, so the
	// program sees the pad go and a new one come, as with a hand swap; the port
	// shows FLAG_UNPLUGGED and FLAG_SWAPPING meanwhile. A PAD_PLUG in the gap ends
	// it (plugging now, or leaving the port empty). An unplugged port takes the
	// model at its next plug. The same model changes nothing. RESULT_NOPAD unless
	// the settings give the port a DualShock 2; RESULT_INVAL for a model out of
	// range. The tool's hold and play go on, as with PAD_PLUG. PAD_TYPER carries
	// the PadStatus.
	constexpr u32 PAD_TYPE_SWAP_VSYNCS = 30;
	struct PadTypeRequest
	{
		PadRequest pad;
		u8 model;
	};
	std::optional<PadTypeRequest> ParsePadType(const Request& req);
} // namespace Pif::Emup
