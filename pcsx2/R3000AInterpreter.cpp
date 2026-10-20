// SPDX-FileCopyrightText: 2002-2026 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

#include "R3000A.h"
#include "Common.h"
#include "Config.h"
#include "VMManager.h"

#include "R5900OpcodeTables.h"
#include "DebugTools/Breakpoints.h"
#include "IopBios.h"
#include "IopHw.h"

using namespace R3000A;

// Used to flag delay slot instructions when throwig exceptions.
bool iopIsDelaySlot = false;

static bool branch2 = 0;
static u32 branchPC;

static void doBranch(s32 tar);	// forward declared prototype

/*********************************************************
* Register branch logic                                  *
* Format:  OP rs, offset                                 *
*********************************************************/

void psxBGEZ()         // Branch if Rs >= 0
{
	if (_i32(_rRs_) >= 0) doBranch(_BranchTarget_);
}

void psxBGEZAL()   // Branch if Rs >= 0 and link
{
	_SetLink(31);
	if (_i32(_rRs_) >= 0)
	{
		doBranch(_BranchTarget_);
	}
}

void psxBGTZ()          // Branch if Rs >  0
{
	if (_i32(_rRs_) > 0) doBranch(_BranchTarget_);
}

void psxBLEZ()         // Branch if Rs <= 0
{
	if (_i32(_rRs_) <= 0) doBranch(_BranchTarget_);
}
void psxBLTZ()          // Branch if Rs <  0
{
	if (_i32(_rRs_) < 0) doBranch(_BranchTarget_);
}

void psxBLTZAL()    // Branch if Rs <  0 and link
{
	_SetLink(31);
	if (_i32(_rRs_) < 0)
		{
			doBranch(_BranchTarget_);
		}
}

/*********************************************************
* Register branch logic                                  *
* Format:  OP rs, rt, offset                             *
*********************************************************/

void psxBEQ()   // Branch if Rs == Rt
{
	if (_i32(_rRs_) == _i32(_rRt_)) doBranch(_BranchTarget_);
}

void psxBNE()   // Branch if Rs != Rt
{
	if (_i32(_rRs_) != _i32(_rRt_)) doBranch(_BranchTarget_);
}

/*********************************************************
* Jump to target                                         *
* Format:  OP target                                     *
*********************************************************/
void psxJ()
{
	// check for iop module import table magic
	u32 delayslot = iopMemRead32(psxRegs.pc);
	if (delayslot >> 16 == 0x2400 && irxImportExec(irxImportTableAddr(psxRegs.pc), delayslot & 0xffff))
		return;

	doBranch(_JumpTarget_);
}

void psxJAL()
{
	_SetLink(31);
	doBranch(_JumpTarget_);
}

/*********************************************************
* Register jump                                          *
* Format:  OP rs, rd                                     *
*********************************************************/
void psxJR()
{
	doBranch(_u32(_rRs_));
}

void psxJALR()
{
	if (_Rd_)
	{
		_SetLink(_Rd_);
	}
	doBranch(_u32(_rRs_));
}

void psxBreakpoint(bool memcheck)
{
	u32 pc = psxRegs.pc;
	if (CBreakPoints::CheckSkipFirst(BREAKPOINT_IOP, pc) != 0)
	{
		CBreakPoints::ClearSkipFirst(BREAKPOINT_IOP);
		return;
	}

	if (!memcheck)
	{
		auto cond = CBreakPoints::GetBreakPointCondition(BREAKPOINT_IOP, pc);
		if (cond && !cond->Evaluate())
			return;

		if (!CBreakPoints::HandleBreakpointHit(BREAKPOINT_IOP, pc))
			return;
	}

	CBreakPoints::SetBreakpointTriggered(true, BREAKPOINT_IOP);
	VMManager::SetPaused(true);
	Cpu->ExitExecution();
}

void psxMemcheck(u32 op, u32 bits, bool store)
{
	// compute accessed address
	u32 start = psxRegs.GPR.r[(op >> 21) & 0x1F];
	if ((s16)op != 0)
		start += (s16)op;

	u32 end = start + bits / 8;

	const auto checks = CBreakPoints::GetMemChecks(BREAKPOINT_IOP);
	for (size_t i = 0; i < checks.size(); i++)
	{
		const auto& check = checks[i];

		if (check.result == 0)
			continue;
		if ((check.memCond & MEMCHECK_WRITE) == 0 && store)
			continue;
		if ((check.memCond & MEMCHECK_READ) == 0 && !store)
			continue;

		if (check.hasCond)
		{
			if (!check.cond.Evaluate())
				continue;
		}

		if (start < check.end && check.start < end)
			psxBreakpoint(true);
	}
}

void psxCheckMemcheck()
{
	u32 pc = psxRegs.pc;
	int needed = psxIsMemcheckNeeded(pc);
	if (needed == 0)
		return;

	u32 op = iopMemRead32(needed == 2 ? pc + 4 : pc);
	// Yeah, we use the R5900 opcode table for the R3000
	const R5900::OPCODE& opcode = R5900::GetInstruction(op);

	bool store = (opcode.flags & IS_STORE) != 0;
	switch (opcode.flags & MEMTYPE_MASK)
	{
	case MEMTYPE_BYTE:
		psxMemcheck(op, 8, store);
		break;
	case MEMTYPE_HALF:
		psxMemcheck(op, 16, store);
		break;
	case MEMTYPE_WORD:
		psxMemcheck(op, 32, store);
		break;
	case MEMTYPE_DWORD:
		psxMemcheck(op, 64, store);
		break;
	}
}

// R3000 load delay: the instruction right after a load still sees the old value of the loaded
// register. PCSX2 doesn't model this in general (assemblers fill load delay slots, so compiled code
// never depends on it), but hand-written ROM code does: DECI2's exception return restores DCIC with
// `lw k0,0x9c(k1); lw k1,0x80(k1); lw k0,0x68(k1); mtc0 k0,DCIC`. Only plain loads (not LWL/LWR,
// which forward) followed by a non-branch, non-exception instruction are handled.
static bool psxIsPlainLoad(u32 code)
{
	const u32 op = code >> 26;
	return op == 0x20 || op == 0x21 || op == 0x23 || op == 0x24 || op == 0x25;
}

// GPR written by `code`, or 0.
u32 psxLoadDelayDest(u32 code)
{
	const u32 op = code >> 26, rs = (code >> 21) & 31, rt = (code >> 16) & 31, rd = (code >> 11) & 31;
	switch (op)
	{
		case 0x00:
			switch (code & 0x3f)
			{
				case 0x11: case 0x13: case 0x18: case 0x19: case 0x1a: case 0x1b: // MTHI/MTLO/MULT/DIV
					return 0;
				default:
					return rd;
			}
		case 0x10: case 0x12: // MFC/CFC
			return (rs == 0 || rs == 2) ? rt : 0;
		default:
			return ((op >= 0x08 && op <= 0x0f) || (op >= 0x20 && op <= 0x26)) ? rt : 0;
	}
}

static bool psxReadsGPR(u32 code, u32 reg)
{
	const u32 op = code >> 26, rs = (code >> 21) & 31, rt = (code >> 16) & 31;
	switch (op)
	{
		case 0x00: return rs == reg || rt == reg;
		case 0x0f: return false; // LUI
		case 0x10: case 0x12: return (rs == 4 || rs == 6) && rt == reg; // MTC/CTC
		case 0x22: case 0x26: return rs == reg || rt == reg; // LWL/LWR merge
		case 0x3a: return rs == reg; // SWC2
		default:
			if ((op >= 0x08 && op <= 0x0e) || (op >= 0x20 && op <= 0x26) || op == 0x32)
				return rs == reg;
			if (op >= 0x28 && op <= 0x2e)
				return rs == reg || rt == reg;
			return false;
	}
}

static bool psxIsBranchOrException(u32 code)
{
	const u32 op = code >> 26, rs = (code >> 21) & 31, funct = code & 0x3f;
	if (op >= 0x01 && op <= 0x07)
		return true;
	if (op == 0x00 && (funct == 0x08 || funct == 0x09 || funct == 0x0c || funct == 0x0d))
		return true;
	if ((op == 0x10 || op == 0x12) && (rs == 8 || (op == 0x10 && rs == 0x10 && funct == 0x10))) // BCz, RFE
		return true;
	return false;
}

bool psxLoadDelayHazard(u32 load, u32 next)
{
	const u32 rt = (load >> 16) & 31;
	return psxIsPlainLoad(load) && rt != 0 && !psxIsBranchOrException(next) && psxReadsGPR(next, rt);
}

// Length of the run that starts with a hazardous load at `pc`: it continues while the current
// instruction is a plain load (its value is still pending) and the next one is a plain instruction,
// up to `endpc`. DECI2's `lw k1; lw k0,0x68(k1); mtc0 k0,DCIC` is a run of 3.
u32 psxLoadDelayRunLength(u32 pc, u32 endpc)
{
	u32 n = 1;
	while (psxIsPlainLoad(iopMemRead32(pc)) && pc + 4 < endpc && !psxIsBranchOrException(iopMemRead32(pc + 4)))
	{
		pc += 4;
		n++;
	}
	return n;
}

// Runs `n` instructions from `pc` with load delay semantics: a load's value lands at the end of
// the following instruction (unless that instruction, not being a load, wrote the same register
// itself). The caller advances pc and the cycle count for all of them.
void psxExecLoadDelayRun(u32 pc, u32 n)
{
	u32 pendingReg = 0, pendingValue = 0;
	for (u32 i = 0; i < n; i++, pc += 4)
	{
		const u32 code = iopMemRead32(pc);
		u32 newReg = 0, newValue = 0;
		if (psxIsPlainLoad(code))
		{
			const u32 ea = psxRegs.GPR.r[(code >> 21) & 31] + static_cast<s32>(static_cast<s16>(code));
			switch (code >> 26)
			{
				case 0x20: newValue = static_cast<s32>(static_cast<s8>(iopMemRead8(ea))); break; // LB
				case 0x21: newValue = static_cast<s32>(static_cast<s16>(iopMemRead16(ea))); break; // LH
				case 0x23: newValue = iopMemRead32(ea); break; // LW
				case 0x24: newValue = iopMemRead8(ea); break; // LBU
				case 0x25: newValue = iopMemRead16(ea); break; // LHU
			}
			newReg = (code >> 16) & 31;
		}
		else
		{
			psxRegs.code = code;
			psxBSC[code >> 26]();
		}

		if (pendingReg && (newReg || psxLoadDelayDest(code) != pendingReg))
			psxRegs.GPR.r[pendingReg] = pendingValue;
		pendingReg = newReg;
		pendingValue = newValue;
	}
	if (pendingReg) // the run ended on a load: let it land now (PCSX2's usual behaviour)
		psxRegs.GPR.r[pendingReg] = pendingValue;
}

///////////////////////////////////////////
// These macros are used to assemble the repassembler functions

static __fi void execI()
{
	// This function is called for every instruction.
	// Enabling the define below will probably, no, will cause the interpretor to be slower.
//#define EXTRA_DEBUG
#if defined(EXTRA_DEBUG) || defined(PCSX2_DEVBUILD)
	if (psxIsBreakpointNeeded(psxRegs.pc))
		psxBreakpoint(false);

	psxCheckMemcheck();
	
	CBreakPoints::CommitClearSkipFirst(BREAKPOINT_IOP);
#endif

	// Inject IRX hack
	if (psxRegs.pc == 0x1630 && EmuConfig.CurrentIRX.length() > 3) {
		if (iopMemRead32(0x20018) == 0x1F) {
			// FIXME do I need to increase the module count (0x1F -> 0x20)
			iopMemWrite32(0x20094, 0xbffc0000);
		}
	}

	// IOP hardware breakpoints (COP0 DCIC). In a delay slot, doBranch keeps the vector.
	if (psxRegs.CP0.r[7] & DCIC_DE)
	{
		if (psxCheckInstBreakpoint(iopIsDelaySlot) || psxCheckDataBreakpoint(iopMemRead32(psxRegs.pc), iopIsDelaySlot))
			return;
	}

	psxRegs.code = iopMemRead32(psxRegs.pc);

		PSXCPU_LOG("%s", disR3000AF(psxRegs.code, psxRegs.pc));

	if (!iopIsDelaySlot && psxIsPlainLoad(psxRegs.code) && psxLoadDelayHazard(psxRegs.code, iopMemRead32(psxRegs.pc + 4)))
	{
		const u32 pc = psxRegs.pc;
		const u32 n = psxLoadDelayRunLength(pc, 0xffffffff);
		psxRegs.pc += 4 * n;
		psxRegs.cycle += n;
		psxExecLoadDelayRun(pc, n);
		return;
	}

	psxRegs.pc+= 4;
	psxRegs.cycle++;

	psxBSC[psxRegs.code >> 26]();
}

static void doBranch(s32 tar) {
	if (tar == 0x0)
		DevCon.Warning("[R3000 Interpreter] Warning: Branch to 0x0!");

	// When upgrading the IOP, there are two resets, the second of which is a 'fake' reset
	// This second 'reset' involves UDNL calling SYSMEM and LOADCORE directly, resetting LOADCORE's modules
	// This detects when SYSMEM is called and clears the modules then
	if(tar == 0x890)
	{
		DevCon.WriteLn(Color_Gray, "R3000 Debugger: Branch to 0x890 (SYSMEM). Clearing modules.");
		R3000SymbolGuardian.ClearIrxModules();
	}

	// Override the memory size argument to IOPBOOT
	if(static_cast<u32>(tar) == 0xbfc4a000) {
		psxRegs.GPR.n.a0 = Ps2MemSize::ExposedIopRam >> 20;
	}

	branch2 = iopIsDelaySlot = true;
	branchPC = tar;
	const u32 slotpc = psxRegs.pc;
	execI();
	PSXCPU_LOG( "\n" );
	iopIsDelaySlot = false;
	// An exception in the delay slot (break, syscall, a DCIC breakpoint) has moved pc to its vector,
	// with EPC = the branch: keep it. (A branch in the slot has set pc already, to branchPC.)
	if (psxRegs.pc == slotpc + 4)
		psxRegs.pc = branchPC;

	iopEventTest();
}

static void intReserve() {
}

static void intAlloc() {
}

static void intReset() {
	intAlloc();
}

static s32 intExecuteBlock( s32 eeCycles )
{
	psxRegs.iopBreak = 0;
	psxRegs.iopCycleEE = eeCycles;
	u64 lastIOPCycle = 0;

	while (psxRegs.iopCycleEE > 0)
	{
		lastIOPCycle = psxRegs.cycle;
		if ((psxHu32(HW_ICFG) & 8) && ((psxRegs.pc & 0x1fffffffU) == 0xa0 || (psxRegs.pc & 0x1fffffffU) == 0xb0 || (psxRegs.pc & 0x1fffffffU) == 0xc0))
			psxBiosCall();

		branch2 = 0;
		while (!branch2)
			execI();

		
		if ((psxHu32(HW_ICFG) & (1 << 3)))
		{
			// F = gcd(PS2CLK, PSXCLK) = 230400
			const u32 cnum = 1280; // PS2CLK / F
			const u32 cdenom = 147; // PSXCLK / F

			//One of the Iop to EE delta clocks to be set in PS1 mode.
			const u32 t = ((cnum * (psxRegs.cycle - lastIOPCycle)) + psxRegs.iopCycleEECarry);
			psxRegs.iopCycleEE -= t / cdenom;
			psxRegs.iopCycleEECarry = t % cdenom;
		}
		else
		{ 
			//default ps2 mode value
			psxRegs.iopCycleEE -= (psxRegs.cycle - lastIOPCycle) * 8;
		}
	}

	return psxRegs.iopBreak + psxRegs.iopCycleEE;
}

static void intClear(u32 Addr, u32 Size) {
}

static void intShutdown() {
}

R3000Acpu psxInt = {
	intReserve,
	intReset,
	intExecuteBlock,
	intClear,
	intShutdown
};
