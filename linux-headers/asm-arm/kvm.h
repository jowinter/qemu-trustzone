/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2, as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */

#ifndef __ARM_KVM_H__
#define __ARM_KVM_H__

#include <asm/types.h>

#define __KVM_HAVE_GUEST_DEBUG
#define __KVM_HAVE_IRQ_LINE

/*
 * KVM_IRQ_LINE macros to set/read IRQ/FIQ for specific VCPU index.
 */
enum KVM_ARM_IRQ_LINE_TYPE {
	KVM_ARM_IRQ_LINE = 0,
	KVM_ARM_FIQ_LINE = 1,
};

/*
 * Modes used for short-hand mode determinition in the world-switch code and
 * in emulation code.
 *
 * Note: These indices do NOT correspond to the value of the CPSR mode bits!
 */
enum vcpu_mode {
	MODE_FIQ = 0,
	MODE_IRQ,
	MODE_SVC,
	MODE_ABT,
	MODE_UND,
	MODE_USR,
	MODE_SYS
};

struct kvm_regs {
	__u32 regs0_7[8];	/* Unbanked regs. (r0 - r7)	   */
	__u32 fiq_regs8_12[5];	/* Banked fiq regs. (r8 - r12)	   */
	__u32 usr_regs8_12[5];	/* Banked usr registers (r8 - r12) */
	__u32 reg13[6];		/* Banked r13, indexed by MODE_	   */
	__u32 reg14[6];		/* Banked r13, indexed by MODE_	   */
	__u32 reg15;
	__u32 cpsr;
	__u32 spsr[5];		/* Banked SPSR,  indexed by MODE_  */
	struct {
		__u32 c0_midr;
		__u32 c1_sys;
		__u32 c2_base0;
		__u32 c2_base1;
		__u32 c2_control;
		__u32 c3_dacr;
	} cp15;

};

struct kvm_sregs {
};

struct kvm_fpu {
};

struct kvm_guest_debug_arch {
};

struct kvm_debug_exit_arch {
};

struct kvm_sync_regs {
};

struct kvm_arch_memory_slot {
};

#endif /* __ARM_KVM_H__ */
