// SPDX-FileCopyrightText: 2002-2026 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

#include "ps2/Iop/PifEmup.h"

#include <algorithm>

namespace Pif::Emup
{
	namespace
	{
		void Put16(std::vector<u8>& p, u16 v)
		{
			p.push_back(static_cast<u8>(v & 0xff));
			p.push_back(static_cast<u8>(v >> 8));
		}

		void Put32(std::vector<u8>& p, u32 v)
		{
			for (int i = 0; i < 4; i++)
				p.push_back(static_cast<u8>(v >> (i * 8)));
		}

		u32 Get32(const std::vector<u8>& p, size_t off)
		{
			return p[off] | (p[off + 1] << 8) | (p[off + 2] << 16) | (static_cast<u32>(p[off + 3]) << 24);
		}

		std::vector<u8> Make(u8 code, u8 result, u32 id, const std::vector<u8>& payload)
		{
			std::vector<u8> p(8);
			p[4] = static_cast<u8>(PROTO & 0xff);
			p[5] = static_cast<u8>(PROTO >> 8);
			p[6] = 'H';
			p[7] = 'H';
			p.push_back(code);
			p.push_back(result);
			Put16(p, 0);
			Put32(p, id);
			p.insert(p.end(), payload.begin(), payload.end());
			p[0] = static_cast<u8>(p.size() & 0xff);
			p[1] = static_cast<u8>(p.size() >> 8);
			return p;
		}
	} // namespace

	std::optional<Request> Parse(const std::vector<u8>& pkt)
	{
		if (pkt.size() < 8 + HEADER_SIZE)
			return std::nullopt;
		Request req;
		req.code = pkt[8];
		req.id = pkt[12] | (pkt[13] << 8) | (pkt[14] << 16) | (static_cast<u32>(pkt[15]) << 24);
		req.payload.assign(pkt.begin() + 8 + HEADER_SIZE, pkt.end());
		return req;
	}

	std::vector<u8> Reply(const Request& req, u8 result, const std::vector<u8>& payload)
	{
		return Make(static_cast<u8>(req.code + 1), result, req.id, payload);
	}

	std::vector<u8> BadReply()
	{
		return Make(CODE_BAD, RESULT_INVAL, 0, {});
	}

	std::vector<u8> VersionPayload()
	{
		std::vector<u8> p;
		Put16(p, VERSION);
		Put16(p, 0);
		Put32(p, FEATURES);
		return p;
	}

	std::optional<PadRequest> ParsePad(const Request& req)
	{
		// Ports 1 and 2 only; a multitap's slots aren't offered yet.
		if (req.payload.size() < 4 || req.payload[0] > 1 || req.payload[1] != 0)
			return std::nullopt;
		return PadRequest{req.payload[0], req.payload[1]};
	}

	std::optional<PadSet> ParsePadSet(const Request& req)
	{
		const std::optional<PadRequest> pad = ParsePad(req);
		if (!pad || req.payload.size() < 4 + PAD_DATA_SIZE)
			return std::nullopt;
		PadSet set{*pad, {}};
		std::copy_n(req.payload.begin() + 4, PAD_DATA_SIZE, set.data.begin());
		return set;
	}

	std::optional<PadPlug> ParsePadPlug(const Request& req)
	{
		const std::optional<PadRequest> pad = ParsePad(req);
		if (!pad || req.payload[2] > 1)
			return std::nullopt;
		return PadPlug{*pad, req.payload[2] != 0};
	}

	std::optional<PadTypeRequest> ParsePadType(const Request& req)
	{
		const std::optional<PadRequest> pad = ParsePad(req);
		if (!pad || req.payload[2] > MODEL_DIGITAL)
			return std::nullopt;
		return PadTypeRequest{*pad, req.payload[2]};
	}

	std::vector<u8> PadStatusPayload(const PadStatus& s)
	{
		std::vector<u8> p = {s.port, s.slot, s.type, s.owner, s.id, s.flags, s.model, 0};
		Put32(p, s.vsync);
		p.insert(p.end(), s.data.begin(), s.data.end());
		p.push_back(s.play.state);
		p.push_back(s.play.flags);
		Put16(p, 0);
		for (const u32 v : {s.play.start, s.play.frame, s.play.entries, s.play.next, s.play.last, s.play.sync_addr,
				 s.play.sync_start, s.play.sync_value})
			Put32(p, v);
		return p;
	}

	std::optional<PadPlayPart> ParsePadPlay(const Request& req)
	{
		const std::optional<PadRequest> pad = ParsePad(req);
		if (!pad || req.payload.size() < 8)
			return std::nullopt;
		const std::vector<u8>& p = req.payload;
		const u32 count = Get32(p, 4);
		const size_t first = (p[2] & PLAY_SYNC) ? 12 : 8;
		if (count > PLAY_MAX_ENTRIES || p.size() != first + count * PLAY_ENTRY_SIZE)
			return std::nullopt;
		PadPlayPart part{*pad, p[2], first == 12 ? Get32(p, 8) : 0, {}};
		part.entries.reserve(count);
		for (size_t off = first; off < p.size(); off += PLAY_ENTRY_SIZE)
		{
			PlayEntry e{Get32(p, off), {}};
			std::copy_n(p.begin() + off + 4, PAD_DATA_SIZE, e.data.begin());
			part.entries.push_back(e);
		}
		return part;
	}
} // namespace Pif::Emup
