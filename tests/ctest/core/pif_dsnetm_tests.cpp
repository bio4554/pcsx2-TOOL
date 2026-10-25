// SPDX-FileCopyrightText: 2002-2026 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

#include "ps2/Iop/PifDsnetm.h"

#include <gtest/gtest.h>

#include <string>
#include <vector>

using Pif::Dsnetm;
using Packet = Dsnetm::Packet;

namespace
{
	struct Recorder final : Dsnetm::Output
	{
		std::vector<std::pair<int, Packet>> toClient;
		std::vector<Packet> toTarget;
		std::vector<int> closed;
		std::vector<u16> flushed;
		int resets = 0;

		void ToClient(int client, Packet pkt) override { toClient.emplace_back(client, std::move(pkt)); }
		void ToTarget(Packet pkt) override { toTarget.push_back(std::move(pkt)); }
		void CloseClient(int client) override { closed.push_back(client); }
		void ResetTarget(const std::array<u8, 16>& params) override
		{
			resets++;
			bootParams = params;
		}
		std::array<u8, 16> bootParams = {};
		void FlushQueued(u16 proto) override { flushed.push_back(proto); }

		void Clear()
		{
			toClient.clear();
			toTarget.clear();
		}
	};

	Packet Pkt(u16 proto, char src, char dst, std::vector<u8> body)
	{
		Packet p = {static_cast<u8>((8 + body.size()) & 0xff), static_cast<u8>((8 + body.size()) >> 8), 0, 0,
			static_cast<u8>(proto & 0xff), static_cast<u8>(proto >> 8), static_cast<u8>(src), static_cast<u8>(dst)};
		p.insert(p.end(), body.begin(), body.end());
		return p;
	}

	// NETMP CONNECT with {priority, protocol} entries.
	Packet Connect(std::vector<std::pair<u8, u16>> entries)
	{
		std::vector<u8> body = {0, 0};
		for (auto [prio, proto] : entries)
			body.insert(body.end(), {prio, 0, static_cast<u8>(proto & 0xff), static_cast<u8>(proto >> 8)});
		return Pkt(0x0400, 'H', 'H', body);
	}

	// The target's CONNECTR to dsnetm's DCMP connect.
	Packet ConnectReply(char node, u8 result = 0)
	{
		return Pkt(0x0001, node, 'H', {0, 1, 0, 0, result, 0, 0, 0});
	}

	struct DsnetmTest : ::testing::Test
	{
		Recorder out;
		Dsnetm dsnetm{out};

		// A client registered for EDBGP and E0TTYP, and the target booted and connected.
		void SetUpSession(int client = 1)
		{
			dsnetm.ClientOpened(client, 0);
			dsnetm.FromClient(client, Connect({{0xd0, 0x0230}, {0xd0, 0x0210}}), 0);
			dsnetm.TargetBooted(-1);
			dsnetm.FromTarget(ConnectReply('I'));
			dsnetm.FromTarget(ConnectReply('E'));
			out.Clear();
		}
	};
} // namespace

TEST_F(DsnetmTest, BootSendsResetReplyThenConnects)
{
	dsnetm.ClientOpened(1, 0);
	dsnetm.FromClient(1, Connect({{0x80, 0x0230}}), 0);
	out.Clear();

	dsnetm.TargetBooted(Dsnetm::CAUSE_NETMP);
	ASSERT_EQ(out.toClient.size(), 1u);
	EXPECT_EQ(out.toClient[0].second, Pkt(0x0400, 'H', 'H', {3, 0, 6}));
	ASSERT_EQ(out.toTarget.size(), 2u);
	EXPECT_EQ(out.toTarget[0], Pkt(0x0001, 'H', 'I', std::vector<u8>(24, 0)));
	EXPECT_EQ(out.toTarget[1], Pkt(0x0001, 'H', 'E', std::vector<u8>(24, 0)));
}

TEST_F(DsnetmTest, ConnectedStatusOncePerBoot)
{
	dsnetm.ClientOpened(1, 0);
	dsnetm.FromClient(1, Connect({{0x80, 0x0230}}), 0);
	dsnetm.TargetBooted(-1);
	out.Clear();

	dsnetm.FromTarget(ConnectReply('I'));
	dsnetm.FromTarget(ConnectReply('I', 2)); // already connected: accepted, no second status
	ASSERT_EQ(out.toClient.size(), 1u);
	EXPECT_EQ(out.toClient[0].second, Pkt(0x0001, 'H', 'H', {2, 0, 0, 0, 'I', 0}));
}

TEST_F(DsnetmTest, NodeUpReconnectsOnlyMissingNodes)
{
	dsnetm.TargetBooted(-1);
	dsnetm.FromTarget(ConnectReply('I'));
	out.Clear();

	// The IOP's "node E up" (DCMP status, code 4).
	dsnetm.FromTarget(Pkt(0x0001, 'I', 'H', {2, 4, 0, 0, 'E', 0}));
	ASSERT_EQ(out.toTarget.size(), 1u);
	EXPECT_EQ(out.toTarget[0][7], 'E');
}

TEST_F(DsnetmTest, ClientPacketsWaitForTheNodeConnect)
{
	dsnetm.ClientOpened(1, 0);
	dsnetm.FromClient(1, Connect({{0x80, 0x0230}}), 0);
	out.Clear();

	const Packet dbgp = Pkt(0x0230, 'H', 'E', {0, 0, 0, 0, 0, 0, 0, 0});
	dsnetm.FromClient(1, dbgp, 0);
	ASSERT_EQ(out.toClient.size(), 1u);
	EXPECT_EQ(out.toClient[0].second[8], 3); // DCMP error
	EXPECT_EQ(out.toClient[0].second[9], 5); // not connected
	EXPECT_TRUE(out.toTarget.empty());

	dsnetm.FromTarget(ConnectReply('E'));
	out.Clear();
	dsnetm.FromClient(1, dbgp, 0);
	ASSERT_EQ(out.toTarget.size(), 1u);
	EXPECT_EQ(out.toTarget[0], dbgp);
}

TEST_F(DsnetmTest, ResetKeepsBootParametersAndForgetsNodes)
{
	SetUpSession();
	// NETMP RESET: EE parameters given, IOP ones all ones (keep).
	std::vector<u8> body = {2, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8};
	body.insert(body.end(), 8, 0xff);
	body.insert(body.end(), 16, 0);
	dsnetm.FromClient(1, Pkt(0x0400, 'H', 'H', body), 0);
	EXPECT_EQ(out.resets, 1);
	EXPECT_EQ(out.bootParams[0], 1); // the new parameters, for connects sent by the bridge
	EXPECT_EQ(out.bootParams[8], 0);
	EXPECT_TRUE(out.toClient.empty()); // no reply until the boot

	dsnetm.FromClient(1, Pkt(0x0230, 'H', 'E', {0, 0, 0, 0, 0, 0, 0, 0}), 0);
	ASSERT_EQ(out.toClient.size(), 1u);
	EXPECT_EQ(out.toClient[0].second[9], 5); // not connected any more

	out.Clear();
	dsnetm.TargetBooted(Dsnetm::CAUSE_NETMP);
	ASSERT_EQ(out.toTarget.size(), 2u);
	const std::vector<u8> params(out.toTarget[0].begin() + 16, out.toTarget[0].end());
	EXPECT_EQ(params, (std::vector<u8>{1, 2, 3, 4, 5, 6, 7, 8, 0, 0, 0, 0, 0, 0, 0, 0}));
}

TEST_F(DsnetmTest, TargetPacketNobodyHoldsIsBouncedThenAnnounced)
{
	SetUpSession();
	// EKTTYP: the EE kernel's banner, nobody registered it.
	const Packet banner = Pkt(0x021f, 'E', 'H', {0, 0, 0, 0, 'h', 'i'});
	dsnetm.FromTarget(banner);
	ASSERT_EQ(out.toTarget.size(), 1u);
	EXPECT_EQ(out.toTarget[0].size(), 36u);
	EXPECT_EQ(out.toTarget[0][7], 'E');
	EXPECT_EQ(out.toTarget[0][8], 3); // DCMP error
	EXPECT_EQ(out.toTarget[0][9], 1); // no protocol
	EXPECT_TRUE(out.toClient.empty());

	out.Clear();
	dsnetm.ClientOpened(2, 0);
	dsnetm.FromClient(2, Connect({{0x80, 0x021f}}), 0);
	ASSERT_EQ(out.toTarget.size(), 1u);
	EXPECT_EQ(out.toTarget[0], Pkt(0x0001, 'H', 'E', {2, 1, 0, 0, 0x1f, 0x02}));
}

TEST_F(DsnetmTest, TtyGoesToEveryHolderAndFlushRequestsFlush)
{
	SetUpSession(1);
	dsnetm.ClientOpened(2, 0);
	dsnetm.FromClient(2, Connect({{0x80, 0x0210}}), 0);
	out.Clear();

	dsnetm.FromTarget(Pkt(0x0210, 'E', 'H', {0, 0, 0, 0, 'x'}));
	ASSERT_EQ(out.toClient.size(), 2u);
	EXPECT_EQ(out.toClient[0].first, 1);
	EXPECT_EQ(out.toClient[1].first, 2);

	dsnetm.FromTarget(Pkt(0x0210, 'E', 'H', {1, 0, 0, 0}));
	ASSERT_EQ(out.flushed.size(), 1u);
	EXPECT_EQ(out.flushed[0], 0x0210);
}

TEST_F(DsnetmTest, TargetErrorGoesToTheHolderAsNotConnected)
{
	SetUpSession();
	// The IOP couldn't route an H->E DBGP packet (no route): 36 bytes with the original header.
	std::vector<u8> body = {3, 0, 0, 0, 16, 0, 0, 0, 0x30, 0x02, 'H', 'E'};
	body.insert(body.end(), 16, 0);
	dsnetm.FromTarget(Pkt(0x0001, 'I', 'H', body));
	ASSERT_EQ(out.toClient.size(), 1u);
	EXPECT_EQ(out.toClient[0].second[9], 5);
}

TEST_F(DsnetmTest, ConnectsAlreadySentAreNotSentAgain)
{
	SetUpSession();
	dsnetm.TargetBooted(Dsnetm::CAUSE_SOFTWARE, true);
	EXPECT_TRUE(out.toTarget.empty());
	ASSERT_EQ(out.toClient.size(), 1u); // the RESETR still goes out
	// The nodes are forgotten until they answer the connects.
	dsnetm.FromClient(1, Pkt(0x0230, 'H', 'E', {0, 0, 0, 0, 0, 0, 0, 0}), 0);
	EXPECT_EQ(out.toClient.back().second[9], 5);
	EXPECT_EQ(Dsnetm::ConnectPacket('I', {}), Pkt(0x0001, 'H', 'I', std::vector<u8>(24, 0)));
}

TEST_F(DsnetmTest, ProgramRebootGetsSoftwareCause)
{
	SetUpSession();
	dsnetm.TargetBooted(Dsnetm::CAUSE_SOFTWARE);
	ASSERT_GE(out.toClient.size(), 1u);
	EXPECT_EQ(out.toClient[0].second, Pkt(0x0400, 'H', 'H', {3, 0, 0}));
}

TEST_F(DsnetmTest, Drfp2OpenFails)
{
	std::vector<u8> body = {0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0};
	const std::string name = "${IOPMODULES}/x.irx";
	body.insert(body.end(), name.begin(), name.end());
	body.push_back(0);
	dsnetm.FromTarget(Pkt(0x0122, 'I', 'H', body));
	ASSERT_EQ(out.toTarget.size(), 1u);
	EXPECT_EQ(out.toTarget[0], Pkt(0x0122, 'H', 'I', {1, 0, 0, 0, 7, 0, 0, 0, 2, 0, 2, 0, 0xff, 0xff, 0xff, 0xff}));
}

TEST_F(DsnetmTest, StatusIsShortForClientsWithoutMessage)
{
	dsnetm.ClientOpened(1, 0);
	dsnetm.FromClient(1, Connect({{0x80, 0x0e01}}), 0);
	dsnetm.ClientOpened(2, 0);
	dsnetm.FromClient(2, Pkt(0x0400, 'H', 'H', {6, 0}), 2500);
	ASSERT_EQ(out.toClient.size(), 2u);
	// 4 bytes of header, then one 12-byte entry less the NUL dsnetm doesn't count.
	EXPECT_EQ(out.toClient[1].second, Pkt(0x0400, 'H', 'H', {7, 0, 0, 0, 0x80, 0, 0x01, 0x0e, 2, 0, 0, 0, 0, 0, 0}));
}
