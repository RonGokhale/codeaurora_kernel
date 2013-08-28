/*
 * Copyright 2013 Tilera Corporation. All Rights Reserved.
 *
 *   This program is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU General Public License
 *   as published by the Free Software Foundation, version 2.
 *
 *   This program is distributed in the hope that it will be useful, but
 *   WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE, GOOD TITLE or
 *   NON INFRINGEMENT.  See the GNU General Public License for
 *   more details.
 */

#include <linux/linkage.h>
#include <asm/switch_to.h>
#include <asm/processor.h>
#include <arch/spr_def.h>
#include <arch/abi.h>

#define FRAME_SIZE ((4 + CALLEE_SAVED_REGS_COUNT) * 8)
#define SAVE_REG(r) { st r12, r; addi r12, r12, 8 }
#define LOAD_REG(r) { ld r, r12; addi r12, r12, 8 }
#define FOR_EACH_CALLEE_SAVED_REG(f)					\
							f(r30); f(r31); \
	f(r32); f(r33); f(r34); f(r35);	f(r36); f(r37); f(r38); f(r39); \
	f(r40); f(r41); f(r42); f(r43); f(r44); f(r45); f(r46); f(r47); \
	f(r48); f(r49); f(r50); f(r51); f(r52);

/*
 * Called with interrupts disabled from kvm_tile_run() and is responsible
 * just for saving the callee-save registers and the stack pointer, then
 * resetting ksp0 so subsequent interrupts don't wipe the kernel stack.
 * It uses restore_all in intvec_64.S to jump back into the guest.
 * The kvm_vmexit function below undoes the stack manipulation.
 */
STD_ENTRY(kvm_vmresume)
	/* Do function prolog and save callee-saves on stack. */
	{
	  move r10, sp
	  st sp, lr
	}
	{
	  addli r11, sp, -FRAME_SIZE + 8
	  addli sp, sp, -FRAME_SIZE
	}
	{
	  st r11, r10
	  addi r12, sp, 16
	}
	FOR_EACH_CALLEE_SAVED_REG(SAVE_REG)
	SAVE_REG(tp)
	SAVE_REG(lr)

	/* Save frame pointer in thread_info so we can get it back later. */
	st r1, sp

	/* Set the ksp0 for this core to be below this frame. */
	mfspr r10, SPR_SYSTEM_SAVE_K_0
	bfins r10, sp, 0, CPU_SHIFT-1
	mtspr SPR_SYSTEM_SAVE_K_0, r10

	/* sp points to ABI save area below pt_regs for restore_all. */
	addli sp, r0, -C_ABI_SAVE_AREA_SIZE

	/* Execute an "interrupt return" to the guest. */
	{
	 movei r30, 0
	 j restore_all
	}
	STD_ENDPROC(kvm_vmresume)

/*
 * Called with interrupts disabled from kvm_trigger_vmexit(); returns with
 * interrupts still disabled to kvm_vmresume()'s caller, discarding all the
 * stack contents below the kvm_vmresume() frame.  kvm_vmresume()'s caller
 * is responsible for resetting SPR_SYSTEM_SAVE_K_0 to its previous value.
 */
STD_ENTRY(kvm_vmexit)
	{
	 move sp, r0
	 addi r12, r0, 16
	}
	FOR_EACH_CALLEE_SAVED_REG(LOAD_REG)
	LOAD_REG(tp)
	LOAD_REG(lr)
	{
	  addli sp, sp, FRAME_SIZE
	  jrp lr
	}
	STD_ENDPROC(kvm_vmexit)
