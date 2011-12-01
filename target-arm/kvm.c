/*
 * ARM implementation of KVM hooks
 *
 * Copyright Christoffer Dall 2009-2010
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 *
 */

#include <stdio.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <linux/kvm.h>

#include "qemu-common.h"
#include "qemu-timer.h"
#include "sysemu.h"
#include "kvm.h"
#include "cpu.h"
#include "device_tree.h"
#include "hw/arm-misc.h"

const KVMCapabilityInfo kvm_arch_required_capabilities[] = {
    KVM_CAP_LAST_INFO
};

int kvm_arch_init(KVMState *s)
{
    return 0;
}

int kvm_arch_init_vcpu(CPUState *env)
{
    return 0;
}

int kvm_arch_put_registers(CPUState *env, int level)
{
    struct kvm_regs regs;
    int ret;

    ret = kvm_vcpu_ioctl(env, KVM_GET_REGS, &regs);
    if (ret < 0)
        return ret;

    memcpy(regs.regs0_7, env->regs, sizeof(uint32_t) * 8);
    memcpy(regs.usr_regs8_12, env->usr_regs, sizeof(uint32_t) * 5);
    memcpy(regs.fiq_regs8_12, env->fiq_regs, sizeof(uint32_t) * 5);
    regs.reg13[MODE_FIQ] = env->banked_r13[5];
    regs.reg13[MODE_IRQ] = env->banked_r13[4];
    regs.reg13[MODE_SVC] = env->banked_r13[1];
    regs.reg13[MODE_ABT] = env->banked_r13[2];
    regs.reg13[MODE_UND] = env->banked_r13[3];
    regs.reg13[MODE_USR] = env->banked_r13[0];
    regs.reg14[MODE_FIQ] = env->banked_r14[5];
    regs.reg14[MODE_IRQ] = env->banked_r14[4];
    regs.reg14[MODE_SVC] = env->banked_r14[1];
    regs.reg14[MODE_ABT] = env->banked_r14[2];
    regs.reg14[MODE_UND] = env->banked_r14[3];
    regs.reg14[MODE_USR] = env->banked_r14[0];
    regs.reg15 = env->regs[15];
    regs.cpsr = cpsr_read(env);
    regs.spsr[MODE_FIQ] = env->banked_spsr[5];
    regs.spsr[MODE_IRQ] = env->banked_spsr[4];
    regs.spsr[MODE_SVC] = env->banked_spsr[1];
    regs.spsr[MODE_ABT] = env->banked_spsr[2];
    regs.spsr[MODE_UND] = env->banked_spsr[3];

    regs.cp15.c0_midr = env->cp15.c0_cpuid;
    regs.cp15.c1_sys = env->cp15.c1_sys;

    ret = kvm_vcpu_ioctl(env, KVM_SET_REGS, &regs);

    return ret;
}

int kvm_arch_get_registers(CPUState *env)
{
    struct kvm_regs regs;
    int32_t ret;

    ret = kvm_vcpu_ioctl(env, KVM_GET_REGS, &regs);
    if (ret < 0)
        return ret;
    memcpy(env->regs, regs.regs0_7, sizeof(uint32_t) * 8);
    memcpy(env->usr_regs, regs.usr_regs8_12, sizeof(uint32_t) * 5);
    memcpy(env->fiq_regs, regs.fiq_regs8_12, sizeof(uint32_t) * 5);
    env->banked_r13[5] = regs.reg13[MODE_FIQ];
    env->banked_r13[4] = regs.reg13[MODE_IRQ];
    env->banked_r13[1] = regs.reg13[MODE_SVC];
    env->banked_r13[2] = regs.reg13[MODE_ABT];
    env->banked_r13[3] = regs.reg13[MODE_UND];
    env->banked_r13[0] = regs.reg13[MODE_USR];
    env->banked_r14[5] = regs.reg14[MODE_FIQ];
    env->banked_r14[4] = regs.reg14[MODE_IRQ];
    env->banked_r14[1] = regs.reg14[MODE_SVC];
    env->banked_r14[2] = regs.reg14[MODE_ABT];
    env->banked_r14[3] = regs.reg14[MODE_UND];
    env->banked_r14[0] = regs.reg14[MODE_USR];
    env->regs[15] = regs.reg15;
    cpsr_write(env, regs.cpsr, 0xFFFFFFFF);
    env->banked_spsr[5] = regs.spsr[MODE_FIQ];
    env->banked_spsr[4] = regs.spsr[MODE_IRQ];
    env->banked_spsr[1] = regs.spsr[MODE_SVC];
    env->banked_spsr[2] = regs.spsr[MODE_ABT];
    env->banked_spsr[3] = regs.spsr[MODE_UND];

    //env->cp15.c0_cpuid = regs.cp15.c0_midr;
    env->cp15.c1_sys = regs.cp15.c1_sys;
    env->cp15.c2_base0 = regs.cp15.c2_base0;
    env->cp15.c2_base1 = regs.cp15.c2_base1;
    env->cp15.c3 = regs.cp15.c3_dacr;

    return 0;
}

#define KVM_ARM_EXCEPTION_IRQ 0x02
#define KVM_ARM_EXCEPTION_FIQ 0x01
int kvm_arch_interrupt(CPUState *env, int irq, int level)
{
    struct kvm_irq_level irq_level;
    int vcpu_idx = env->cpu_index;
    KVMState *s = kvm_state;
    int ret;

    if (level)
        irq_level.level = 1;
    else
        irq_level.level = 0;

    switch (irq) {
    case ARM_PIC_CPU_IRQ:
        irq_level.irq = KVM_ARM_IRQ_LINE | (vcpu_idx << 1);
        break;
    case ARM_PIC_CPU_FIQ:
        irq_level.irq = KVM_ARM_FIQ_LINE | (vcpu_idx << 1);
        break;
    default:
        fprintf(stderr, "unsupported ARM irq injection\n");
        abort();
    }

    ret = kvm_vm_ioctl(s, KVM_IRQ_LINE, &irq_level);
    if (ret) {
        fprintf(stderr, "kvm_vm_ioctl(s, KVM_IRQ_LINE, &irq_level) failed\n");
        abort();
    }

    return 0;
}

void kvm_arch_pre_run(CPUState *env, struct kvm_run *run)
{
}

void kvm_arch_post_run(CPUState *env, struct kvm_run *run)
{
}

int kvm_arch_handle_exit(CPUState *env, struct kvm_run *run)
{
    int ret = 0;

    return ret;
}

void kvm_arch_reset_vcpu(CPUState *env)
{
}

bool kvm_arch_stop_on_emulation_error(CPUState *env)
{
    return true;
}

int kvm_arch_process_async_events(CPUState *env)
{
    return 0;
}

int kvm_arch_on_sigbus_vcpu(CPUState *env, int code, void *addr)
{
    return 1;
}

int kvm_arch_on_sigbus(int code, void *addr)
{
    return 1;
}
