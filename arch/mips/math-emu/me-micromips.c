#include <asm/inst.h>

/* (microMIPS) Convert certain microMIPS instructions to MIPS32 format. */
static const int sd_format[] = {16, 17, 0, 0, 0, 0, 0, 0};
static const int sdps_format[] = {16, 17, 22, 0, 0, 0, 0, 0};
static const int dwl_format[] = {17, 20, 21, 0, 0, 0, 0, 0};
static const int swl_format[] = {16, 20, 21, 0, 0, 0, 0, 0};

/*
 * This functions translates a 32-bit microMIPS instruction
 * into a 32-bit MIPS32 instruction. Returns 0 on success
 * and SIGILL otherwise.
 */
int microMIPS32_to_MIPS32(union mips_instruction *insn_ptr)
{
	union mips_instruction insn = *insn_ptr;
	union mips_instruction mips32_insn = insn;
	int func, fmt, op;

	switch (insn.mm_i_format.opcode) {
	case mm_ldc132_op:
		mips32_insn.mm_i_format.opcode = ldc1_op;
		mips32_insn.mm_i_format.rt = insn.mm_i_format.rs;
		mips32_insn.mm_i_format.rs = insn.mm_i_format.rt;
		break;
	case mm_lwc132_op:
		mips32_insn.mm_i_format.opcode = lwc1_op;
		mips32_insn.mm_i_format.rt = insn.mm_i_format.rs;
		mips32_insn.mm_i_format.rs = insn.mm_i_format.rt;
		break;
	case mm_sdc132_op:
		mips32_insn.mm_i_format.opcode = sdc1_op;
		mips32_insn.mm_i_format.rt = insn.mm_i_format.rs;
		mips32_insn.mm_i_format.rs = insn.mm_i_format.rt;
		break;
	case mm_swc132_op:
		mips32_insn.mm_i_format.opcode = swc1_op;
		mips32_insn.mm_i_format.rt = insn.mm_i_format.rs;
		mips32_insn.mm_i_format.rs = insn.mm_i_format.rt;
		break;
	case mm_pool32i_op:
		/* NOTE: offset is << by 1 if in microMIPS mode. */
		if ((insn.mm_i_format.rt == mm_bc1f_op) ||
		    (insn.mm_i_format.rt == mm_bc1t_op)) {
			mips32_insn.fb_format.opcode = cop1_op;
			mips32_insn.fb_format.bc = bc_op;
			mips32_insn.fb_format.flag =
				(insn.mm_i_format.rt == mm_bc1t_op) ? 1 : 0;
		} else
			return SIGILL;
		break;
	case mm_pool32f_op:
		switch (insn.mm_fp0_format.func) {
		case mm_32f_01_op:
		case mm_32f_11_op:
		case mm_32f_02_op:
		case mm_32f_12_op:
		case mm_32f_41_op:
		case mm_32f_51_op:
		case mm_32f_42_op:
		case mm_32f_52_op:
			op = insn.mm_fp0_format.func;
			if (op == mm_32f_01_op)
				func = madd_s_op;
			else if (op == mm_32f_11_op)
				func = madd_d_op;
			else if (op == mm_32f_02_op)
				func = nmadd_s_op;
			else if (op == mm_32f_12_op)
				func = nmadd_d_op;
			else if (op == mm_32f_41_op)
				func = msub_s_op;
			else if (op == mm_32f_51_op)
				func = msub_d_op;
			else if (op == mm_32f_42_op)
				func = nmsub_s_op;
			else
				func = nmsub_d_op;
			mips32_insn.fp6_format.opcode = cop1x_op;
			mips32_insn.fp6_format.fr = insn.mm_fp6_format.fr;
			mips32_insn.fp6_format.ft = insn.mm_fp6_format.ft;
			mips32_insn.fp6_format.fs = insn.mm_fp6_format.fs;
			mips32_insn.fp6_format.fd = insn.mm_fp6_format.fd;
			mips32_insn.fp6_format.func = func;
			break;
		case mm_32f_10_op:
			func = -1;	/* Invalid */
			op = insn.mm_fp5_format.op & 0x7;
			if (op == mm_ldxc1_op)
				func = ldxc1_op;
			else if (op == mm_sdxc1_op)
				func = sdxc1_op;
			else if (op == mm_lwxc1_op)
				func = lwxc1_op;
			else if (op == mm_swxc1_op)
				func = swxc1_op;

			if (func != -1) {
				mips32_insn.r_format.opcode = cop1x_op;
				mips32_insn.r_format.rs =
					insn.mm_fp5_format.base;
				mips32_insn.r_format.rt =
					insn.mm_fp5_format.index;
				mips32_insn.r_format.rd = 0;
				mips32_insn.r_format.re = insn.mm_fp5_format.fd;
				mips32_insn.r_format.func = func;
			} else
				return SIGILL;
			break;
		case mm_32f_40_op:
			op = -1;	/* Invalid */
			if (insn.mm_fp2_format.op == mm_fmovt_op)
				op = 1;
			else if (insn.mm_fp2_format.op == mm_fmovf_op)
				op = 0;
			if (op != -1) {
				mips32_insn.fp0_format.opcode = cop1_op;
				mips32_insn.fp0_format.fmt =
					sdps_format[insn.mm_fp2_format.fmt];
				mips32_insn.fp0_format.ft =
					(insn.mm_fp2_format.cc<<2) + op;
				mips32_insn.fp0_format.fs =
					insn.mm_fp2_format.fs;
				mips32_insn.fp0_format.fd =
					insn.mm_fp2_format.fd;
				mips32_insn.fp0_format.func = fmovc_op;
			} else
				return SIGILL;
			break;
		case mm_32f_60_op:
			func = -1;	/* Invalid */
			if (insn.mm_fp0_format.op == mm_fadd_op)
				func = fadd_op;
			else if (insn.mm_fp0_format.op == mm_fsub_op)
				func = fsub_op;
			else if (insn.mm_fp0_format.op == mm_fmul_op)
				func = fmul_op;
			else if (insn.mm_fp0_format.op == mm_fdiv_op)
				func = fdiv_op;
			if (func != -1) {
				mips32_insn.fp0_format.opcode = cop1_op;
				mips32_insn.fp0_format.fmt =
					sdps_format[insn.mm_fp0_format.fmt];
				mips32_insn.fp0_format.ft =
					insn.mm_fp0_format.ft;
				mips32_insn.fp0_format.fs =
					insn.mm_fp0_format.fs;
				mips32_insn.fp0_format.fd =
					insn.mm_fp0_format.fd;
				mips32_insn.fp0_format.func = func;
			} else
				return SIGILL;
			break;
		case mm_32f_70_op:
			func = -1;	/* Invalid */
			if (insn.mm_fp0_format.op == mm_fmovn_op)
				func = fmovn_op;
			else if (insn.mm_fp0_format.op == mm_fmovz_op)
				func = fmovz_op;
			if (func != -1) {
				mips32_insn.fp0_format.opcode = cop1_op;
				mips32_insn.fp0_format.fmt =
					sdps_format[insn.mm_fp0_format.fmt];
				mips32_insn.fp0_format.ft =
					insn.mm_fp0_format.ft;
				mips32_insn.fp0_format.fs =
					insn.mm_fp0_format.fs;
				mips32_insn.fp0_format.fd =
					insn.mm_fp0_format.fd;
				mips32_insn.fp0_format.func = func;
			} else
				return SIGILL;
			break;
		case mm_32f_73_op:    /* POOL32FXF */
			switch (insn.mm_fp1_format.op) {
			case mm_movf0_op:
			case mm_movf1_op:
			case mm_movt0_op:
			case mm_movt1_op:
				if ((insn.mm_fp1_format.op & 0x7f) ==
				    mm_movf0_op)
					op = 0;
				else
					op = 1;
				mips32_insn.r_format.opcode = spec_op;
				mips32_insn.r_format.rs = insn.mm_fp4_format.fs;
				mips32_insn.r_format.rt =
					(insn.mm_fp4_format.cc << 2) + op;
				mips32_insn.r_format.rd = insn.mm_fp4_format.rt;
				mips32_insn.r_format.re = 0;
				mips32_insn.r_format.func = movc_op;
				break;
			case mm_fcvtd0_op:
			case mm_fcvtd1_op:
			case mm_fcvts0_op:
			case mm_fcvts1_op:
				if ((insn.mm_fp1_format.op & 0x7f) ==
				    mm_fcvtd0_op) {
					func = fcvtd_op;
					fmt = swl_format[insn.mm_fp3_format.fmt];
				} else {
					func = fcvts_op;
					fmt = dwl_format[insn.mm_fp3_format.fmt];
				}
				mips32_insn.fp0_format.opcode = cop1_op;
				mips32_insn.fp0_format.fmt = fmt;
				mips32_insn.fp0_format.ft = 0;
				mips32_insn.fp0_format.fs =
					insn.mm_fp3_format.fs;
				mips32_insn.fp0_format.fd =
					insn.mm_fp3_format.rt;
				mips32_insn.fp0_format.func = func;
				break;
			case mm_fmov0_op:
			case mm_fmov1_op:
			case mm_fabs0_op:
			case mm_fabs1_op:
			case mm_fneg0_op:
			case mm_fneg1_op:
				if ((insn.mm_fp1_format.op & 0x7f) ==
				    mm_fmov0_op)
					func = fmov_op;
				else if ((insn.mm_fp1_format.op & 0x7f) ==
					 mm_fabs0_op)
					func = fabs_op;
				else
					func = fneg_op;
				mips32_insn.fp0_format.opcode = cop1_op;
				mips32_insn.fp0_format.fmt =
					sdps_format[insn.mm_fp3_format.fmt];
				mips32_insn.fp0_format.ft = 0;
				mips32_insn.fp0_format.fs =
					insn.mm_fp3_format.fs;
				mips32_insn.fp0_format.fd =
					insn.mm_fp3_format.rt;
				mips32_insn.fp0_format.func = func;
				break;
			case mm_ffloorl_op:
			case mm_ffloorw_op:
			case mm_fceill_op:
			case mm_fceilw_op:
			case mm_ftruncl_op:
			case mm_ftruncw_op:
			case mm_froundl_op:
			case mm_froundw_op:
			case mm_fcvtl_op:
			case mm_fcvtw_op:
				if (insn.mm_fp1_format.op == mm_ffloorl_op)
					func = ffloorl_op;
				else if (insn.mm_fp1_format.op == mm_ffloorw_op)
					func = ffloor_op;
				else if (insn.mm_fp1_format.op == mm_fceill_op)
					func = fceill_op;
				else if (insn.mm_fp1_format.op == mm_fceilw_op)
					func = fceil_op;
				else if (insn.mm_fp1_format.op == mm_ftruncl_op)
					func = ftruncl_op;
				else if (insn.mm_fp1_format.op == mm_ftruncw_op)
					func = ftrunc_op;
				else if (insn.mm_fp1_format.op == mm_froundl_op)
					func = froundl_op;
				else if (insn.mm_fp1_format.op == mm_froundw_op)
					func = fround_op;
				else if (insn.mm_fp1_format.op == mm_fcvtl_op)
					func = fcvtl_op;
				else
					func = fcvtw_op;
				mips32_insn.fp0_format.opcode = cop1_op;
				mips32_insn.fp0_format.fmt =
					sd_format[insn.mm_fp1_format.fmt];
				mips32_insn.fp0_format.ft = 0;
				mips32_insn.fp0_format.fs =
					insn.mm_fp1_format.fs;
				mips32_insn.fp0_format.fd =
					insn.mm_fp1_format.rt;
				mips32_insn.fp0_format.func = func;
				break;
			case mm_frsqrt_op:
			case mm_fsqrt_op:
			case mm_frecip_op:
				if (insn.mm_fp1_format.op == mm_frsqrt_op)
					func = frsqrt_op;
				else if (insn.mm_fp1_format.op == mm_fsqrt_op)
					func = fsqrt_op;
				else
					func = frecip_op;
				mips32_insn.fp0_format.opcode = cop1_op;
				mips32_insn.fp0_format.fmt =
					sdps_format[insn.mm_fp1_format.fmt];
				mips32_insn.fp0_format.ft = 0;
				mips32_insn.fp0_format.fs =
					insn.mm_fp1_format.fs;
				mips32_insn.fp0_format.fd =
					insn.mm_fp1_format.rt;
				mips32_insn.fp0_format.func = func;
				break;
			case mm_mfc1_op:
			case mm_mtc1_op:
			case mm_cfc1_op:
			case mm_ctc1_op:
			case mm_mfhc1_op:
			case mm_mthc1_op:
				if (insn.mm_fp1_format.op == mm_mfc1_op)
					op = mfc_op;
				else if (insn.mm_fp1_format.op == mm_mtc1_op)
					op = mtc_op;
				else if (insn.mm_fp1_format.op == mm_cfc1_op)
					op = cfc_op;
				else if (insn.mm_fp1_format.op == mm_ctc1_op)
					op = ctc_op;
				else if (insn.mm_fp1_format.op == mm_mfhc1_op)
					op = mfhc_op;
				else
					op = mthc_op;
				mips32_insn.fp1_format.opcode = cop1_op;
				mips32_insn.fp1_format.op = op;
				mips32_insn.fp1_format.rt =
					insn.mm_fp1_format.rt;
				mips32_insn.fp1_format.fs =
					insn.mm_fp1_format.fs;
				mips32_insn.fp1_format.fd = 0;
				mips32_insn.fp1_format.func = 0;
				break;
			default:
				return SIGILL;
			}
			break;
		case mm_32f_74_op:	/* c.cond.fmt */
			mips32_insn.fp0_format.opcode = cop1_op;
			mips32_insn.fp0_format.fmt =
				sdps_format[insn.mm_fp4_format.fmt];
			mips32_insn.fp0_format.ft = insn.mm_fp4_format.rt;
			mips32_insn.fp0_format.fs = insn.mm_fp4_format.fs;
			mips32_insn.fp0_format.fd = insn.mm_fp4_format.cc << 2;
			mips32_insn.fp0_format.func =
				insn.mm_fp4_format.cond | MM_MIPS32_COND_FC;
			break;
		default:
			return SIGILL;
		}
		break;
	default:
		return SIGILL;
	}

	*insn_ptr = mips32_insn;
	return 0;
}
