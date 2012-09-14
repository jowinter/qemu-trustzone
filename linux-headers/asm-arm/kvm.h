/*
 * Copyright (C) 2012 - Virtual Open Systems and Columbia University
 * Author: Christoffer Dall <c.dall@virtualopensystems.com>
 *
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
 */

#ifndef __ARM_KVM_H__
#define __ARM_KVM_H__

#include <asm/types.h>

#define __KVM_HAVE_GUEST_DEBUG
#define __KVM_HAVE_IRQ_LINE

#define KVM_REG_SIZE(id)						\
	(1U << (((id) & KVM_REG_SIZE_MASK) >> KVM_REG_SIZE_SHIFT))

struct kvm_regs {
	__u32 usr_regs[15];	/* R0_usr - R14_usr */
	__u32 svc_regs[3];	/* SP_svc, LR_svc, SPSR_svc */
	__u32 abt_regs[3];	/* SP_abt, LR_abt, SPSR_abt */
	__u32 und_regs[3];	/* SP_und, LR_und, SPSR_und */
	__u32 irq_regs[3];	/* SP_irq, LR_irq, SPSR_irq */
	__u32 fiq_regs[8];	/* R8_fiq - R14_fiq, SPSR_fiq */
	__u32 pc;		/* The program counter (r15) */
	__u32 cpsr;		/* The guest CPSR */
};

/* Supported Processor Types */
#define KVM_ARM_TARGET_CORTEX_A15	(0xC0F)

struct kvm_vcpu_init {
	__u32 target;
	__u32 features[7];
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

/* For KVM_VCPU_GET_REG_LIST. */
struct kvm_reg_list {
	__u64 n; /* number of regs */
	__u64 reg[0];
};

/* If you need to interpret the index values, here is the key: */
#define KVM_REG_ARM_COPROC_MASK		0x000000000FFF0000
#define KVM_REG_ARM_COPROC_SHIFT	16
#define KVM_REG_ARM_32_OPC2_MASK	0x0000000000000007
#define KVM_REG_ARM_32_OPC2_SHIFT	0
#define KVM_REG_ARM_OPC1_MASK		0x0000000000000078
#define KVM_REG_ARM_OPC1_SHIFT		3
#define KVM_REG_ARM_CRM_MASK		0x0000000000000780
#define KVM_REG_ARM_CRM_SHIFT		7
#define KVM_REG_ARM_32_CRN_MASK		0x0000000000007800
#define KVM_REG_ARM_32_CRN_SHIFT	11

/* KVM_IRQ_LINE irq field index values */
#define KVM_ARM_IRQ_TYPE_SHIFT		24
#define KVM_ARM_IRQ_TYPE_MASK		0xff
#define KVM_ARM_IRQ_VCPU_SHIFT		16
#define KVM_ARM_IRQ_VCPU_MASK		0xff
#define KVM_ARM_IRQ_NUM_SHIFT		0
#define KVM_ARM_IRQ_NUM_MASK		0xffff

/* irq_type field */
#define KVM_ARM_IRQ_TYPE_CPU		0
#define KVM_ARM_IRQ_TYPE_SPI		1
#define KVM_ARM_IRQ_TYPE_PPI		2

/* out-of-kernel GIC cpu interrupt injection irq_number field */
#define KVM_ARM_IRQ_CPU_IRQ		0
#define KVM_ARM_IRQ_CPU_FIQ		1

/* Highest supported SPI, from VGIC_NR_IRQS */
#define KVM_ARM_IRQ_GIC_MAX		127

/* Normal registers are mapped as coprocessor 16. */
#define KVM_REG_ARM_CORE		(0x0010 << KVM_REG_ARM_COPROC_SHIFT)
#define KVM_REG_ARM_CORE_REG(name)	(offsetof(struct kvm_regs, name) / 4)

#endif /* __ARM_KVM_H__ */
