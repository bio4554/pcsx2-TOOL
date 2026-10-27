// SPDX-FileCopyrightText: 2002-2026 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

#include "ps2/Iop/PifEmup.h"

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

	std::vector<u8> PadStatusPayload(const PadStatus& s)
	{
		std::vector<u8> p = {s.port, s.slot, s.type, s.owner, s.id, s.flags};
		Put16(p, 0);
		Put32(p, s.vsync);
		p.insert(p.end(), s.data.begin(), s.data.end());
		return p;
	}
} // namespace Pif::Emup
