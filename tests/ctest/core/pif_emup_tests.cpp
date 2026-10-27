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
	s.model = MODEL_DIGITAL;
	s.vsync = 0x01020304;
	for (size_t i = 0; i < s.data.size(); i++)
		s.data[i] = static_cast<u8>(0x80 + i);
	s.play = {PLAY_PLAYING, PLAY_FLAG_SYNC, 0x100, 7, 3, 2, 0x20, 0x00123df0, 5, 12};
	const auto p = PadStatusPayload(s);
	ASSERT_EQ(p.size(), 12 + PAD_DATA_SIZE + 36);
	EXPECT_EQ((std::vector<u8>(p.begin(), p.begin() + 12)),
		(std::vector<u8>{1, 0, PAD_DUALSHOCK2, OWNER_HOST, 0x79, 3, MODEL_DIGITAL, 0, 4, 3, 2, 1}));
	EXPECT_EQ(p[12], 0x80);
	EXPECT_EQ(p[12 + PAD_DATA_SIZE - 1], 0x80 + PAD_DATA_SIZE - 1);
	EXPECT_EQ((std::vector<u8>(p.begin() + 12 + PAD_DATA_SIZE, p.end())),
		(std::vector<u8>{PLAY_PLAYING, PLAY_FLAG_SYNC, 0, 0, 0, 1, 0, 0, 7, 0, 0, 0, 3, 0, 0, 0, 2, 0, 0, 0, 0x20, 0,
			0, 0, 0xf0, 0x3d, 0x12, 0, 5, 0, 0, 0, 12, 0, 0, 0}));
}

TEST(EmupTest, PadSetCarriesThePadThenTheDataBytes)
{
	std::vector<u8> body = {CODE_PAD_SET, 0, 0, 0, 5, 0, 0, 0, 1, 0, 0, 0};
	for (size_t i = 0; i < PAD_DATA_SIZE; i++)
		body.push_back(static_cast<u8>(0x40 + i));
	auto req = *Parse(Pkt(body));
	const auto set = ParsePadSet(req);
	ASSERT_TRUE(set);
	EXPECT_EQ(set->pad.port, 1);
	EXPECT_EQ(set->data[0], 0x40);
	EXPECT_EQ(set->data[PAD_DATA_SIZE - 1], 0x40 + PAD_DATA_SIZE - 1);
	req.payload.pop_back();
	EXPECT_FALSE(ParsePadSet(req));
	req.payload = {2, 0, 0, 0};
	req.payload.resize(4 + PAD_DATA_SIZE);
	EXPECT_FALSE(ParsePadSet(req));
	EXPECT_EQ(Reply(req, RESULT_NOPAD), Pkt({CODE_PAD_SETR, RESULT_NOPAD, 0, 0, 5, 0, 0, 0}));
}

TEST(EmupTest, PadPlugCarriesWhetherThePadGoesIn)
{
	auto req = *Parse(Pkt({CODE_PAD_PLUG, 0, 0, 0, 8, 0, 0, 0, 1, 0, 1, 0}));
	auto plug = ParsePadPlug(req);
	ASSERT_TRUE(plug);
	EXPECT_EQ(plug->pad.port, 1);
	EXPECT_TRUE(plug->plugged);
	req.payload[2] = 0;
	plug = ParsePadPlug(req);
	ASSERT_TRUE(plug);
	EXPECT_FALSE(plug->plugged);
	req.payload[2] = 2;
	EXPECT_FALSE(ParsePadPlug(req));
	req.payload = {0, 0, 1};
	EXPECT_FALSE(ParsePadPlug(req));
	EXPECT_EQ(Reply(req, RESULT_NOPAD), Pkt({CODE_PAD_PLUGR, RESULT_NOPAD, 0, 0, 8, 0, 0, 0}));
}

TEST(EmupTest, PadTypeCarriesTheModel)
{
	auto req = *Parse(Pkt({CODE_PAD_TYPE, 0, 0, 0, 9, 0, 0, 0, 0, 0, MODEL_DUALSHOCK, 0}));
	auto type = ParsePadType(req);
	ASSERT_TRUE(type);
	EXPECT_EQ(type->pad.port, 0);
	EXPECT_EQ(type->model, MODEL_DUALSHOCK);
	req.payload[2] = MODEL_DIGITAL + 1;
	EXPECT_FALSE(ParsePadType(req));
	req.payload = {0, 0, 1};
	EXPECT_FALSE(ParsePadType(req));
	EXPECT_EQ(Reply(req, RESULT_NOPAD), Pkt({CODE_PAD_TYPER, RESULT_NOPAD, 0, 0, 9, 0, 0, 0}));
}

TEST(EmupTest, PadPlayCarriesItsEntriesByCount)
{
	std::vector<u8> body = {CODE_PAD_PLAY, 0, 0, 0, 6, 0, 0, 0, 0, 0, PLAY_FIRST | PLAY_START, 0, 2, 0, 0, 0};
	for (const u32 frame : {0u, 0x01020304u})
	{
		for (int i = 0; i < 4; i++)
			body.push_back(static_cast<u8>(frame >> (i * 8)));
		for (size_t i = 0; i < PAD_DATA_SIZE; i++)
			body.push_back(static_cast<u8>(frame + i));
		body.push_back(0);
		body.push_back(0);
	}
	auto req = *Parse(Pkt(body));
	const auto part = ParsePadPlay(req);
	ASSERT_TRUE(part);
	EXPECT_EQ(part->pad.port, 0);
	EXPECT_EQ(part->flags, PLAY_FIRST | PLAY_START);
	ASSERT_EQ(part->entries.size(), 2u);
	EXPECT_EQ(part->entries[0].frame, 0u);
	EXPECT_EQ(part->entries[1].frame, 0x01020304u);
	EXPECT_EQ(part->entries[1].data[0], 0x04);
	EXPECT_EQ(part->entries[1].data[PAD_DATA_SIZE - 1], static_cast<u8>(0x04 + PAD_DATA_SIZE - 1));
	req.payload.pop_back(); // a count that doesn't match the entries
	EXPECT_FALSE(ParsePadPlay(req));
	req.payload = {0, 0, PLAY_START, 0, 0, 0, 0, 0};
	ASSERT_TRUE(ParsePadPlay(req));
	EXPECT_TRUE(ParsePadPlay(req)->entries.empty());
}

TEST(EmupTest, PadPlaySyncPutsTheCounterBeforeTheEntries)
{
	std::vector<u8> payload = {1, 0, PLAY_FIRST | PLAY_START | PLAY_SYNC, 0, 1, 0, 0, 0, 0xf0, 0x3d, 0x12, 0};
	payload.insert(payload.end(), {9, 0, 0, 0});
	for (size_t i = 0; i < PAD_DATA_SIZE + 2; i++)
		payload.push_back(static_cast<u8>(i));
	Request req{CODE_PAD_PLAY, 7, payload};
	const auto part = ParsePadPlay(req);
	ASSERT_TRUE(part);
	EXPECT_EQ(part->sync_addr, 0x00123df0u);
	ASSERT_EQ(part->entries.size(), 1u);
	EXPECT_EQ(part->entries[0].frame, 9u);
	EXPECT_EQ(part->entries[0].data[1], 1);
	req.payload.resize(8 + PLAY_ENTRY_SIZE); // without its address the length doesn't match
	EXPECT_FALSE(ParsePadPlay(req));
	EXPECT_EQ(VersionPayload()[4] & FEATURE_PAD_SYNC, FEATURE_PAD_SYNC);
}
