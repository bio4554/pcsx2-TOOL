// SPDX-FileCopyrightText: 2002-2026 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

#include "ps2/Iop/PifEmup.h"

#include <gtest/gtest.h>

#include <vector>

using namespace Pif::Emup;

namespace
{
	std::vector<u8> Pkt(std::vector<u8> body)
	{
		std::vector<u8> p = {static_cast<u8>((8 + body.size()) & 0xff), static_cast<u8>((8 + body.size()) >> 8), 0, 0,
			0x01, 0xe0, 'H', 'H'};
		p.insert(p.end(), body.begin(), body.end());
		return p;
	}
} // namespace

TEST(EmupTest, ParseTakesCodeIdAndPayload)
{
	const auto req = Parse(Pkt({0x10, 0, 0, 0, 0x78, 0x56, 0x34, 0x12, 1, 0, 0, 0}));
	ASSERT_TRUE(req);
	EXPECT_EQ(req->code, CODE_PAD_STATUS);
	EXPECT_EQ(req->id, 0x12345678u);
	EXPECT_EQ(req->payload, (std::vector<u8>{1, 0, 0, 0}));
	EXPECT_FALSE(Parse(Pkt({0x10, 0, 0, 0, 1, 2, 3})));
}

TEST(EmupTest, ReplyEchoesTheIdWithTheNextCode)
{
	const auto req = Parse(Pkt({0x00, 0, 0, 0, 9, 0, 0, 0}));
	ASSERT_TRUE(req);
	EXPECT_EQ(Reply(*req, RESULT_OK, VersionPayload()),
		Pkt({0x01, 0, 0, 0, 9, 0, 0, 0, VERSION, 0, 0, 0, FEATURES, 0, 0, 0}));
	EXPECT_EQ(Reply(*req, RESULT_NOVM), Pkt({0x01, RESULT_NOVM, 0, 0, 9, 0, 0, 0}));
	EXPECT_EQ(BadReply(), Pkt({0xff, RESULT_INVAL, 0, 0, 0, 0, 0, 0}));
}

TEST(EmupTest, PadRequestIsPortOneOrTwoWithoutMultitap)
{
	auto req = *Parse(Pkt({0x10, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0}));
	ASSERT_TRUE(ParsePad(req));
	EXPECT_EQ(ParsePad(req)->port, 1);
	req.payload = {2, 0, 0, 0};
	EXPECT_FALSE(ParsePad(req));
	req.payload = {0, 1, 0, 0};
	EXPECT_FALSE(ParsePad(req));
	req.payload = {0, 0};
	EXPECT_FALSE(ParsePad(req));
}

TEST(EmupTest, PadStatusLayout)
{
	PadStatus s;
	s.port = 1;
	s.type = PAD_DUALSHOCK2;
	s.id = 0x79;
	s.flags = FLAG_ANALOG_LIGHT | FLAG_ANALOG_LOCKED;
	s.vsync = 0x01020304;
	for (size_t i = 0; i < s.data.size(); i++)
		s.data[i] = static_cast<u8>(0x80 + i);
	const auto p = PadStatusPayload(s);
	ASSERT_EQ(p.size(), 12 + PAD_DATA_SIZE);
	EXPECT_EQ((std::vector<u8>(p.begin(), p.begin() + 12)),
		(std::vector<u8>{1, 0, PAD_DUALSHOCK2, OWNER_HOST, 0x79, 3, 0, 0, 4, 3, 2, 1}));
	EXPECT_EQ(p[12], 0x80);
	EXPECT_EQ(p.back(), 0x80 + PAD_DATA_SIZE - 1);
}
