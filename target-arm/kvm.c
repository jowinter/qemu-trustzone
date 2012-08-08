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
    /* For ARM interrupt delivery is always asynchronous,
     * whether we are using an in-kernel VGIC or not.
     */
    kvm_async_interrupts_allowed = true;
    return 0;
}

int kvm_arch_init_vcpu(CPUARMState *env)
{
    struct kvm_vcpu_init init;

    init.target = KVM_ARM_TARGET_CORTEX_A15;
    memset(init.features, 0, sizeof(init.features));
    return kvm_vcpu_ioctl(env, KVM_ARM_VCPU_INIT, &init);
}

#define MSR32_INDEX_OF(coproc, crn, opc1, crm, opc2) \
    (((coproc)<<16) | ((opc1)<<11) | ((crn)<<7) | ((opc2)<<4) | (crm))

int kvm_arch_put_registers(CPUARMState *env, int level)
{
    struct kvm_regs regs;
    int mode, bn;
    struct cp15 {
        struct kvm_msrs hdr;
        struct kvm_msr_entry e[2];
    } cp15;
    int ret;

    ret = kvm_vcpu_ioctl(env, KVM_GET_REGS, &regs);
    if (ret < 0) {
        return ret;
    }

    /* We make sure the banked regs are properly set */
    mode = env->uncached_cpsr & CPSR_M;
    bn = bank_number(env, mode);
    if (mode == ARM_CPU_MODE_FIQ) {
        memcpy(env->fiq_regs, env->regs + 8, 5 * sizeof(uint32_t));
    } else {
        memcpy(env->usr_regs, env->regs + 8, 5 * sizeof(uint32_t));
    }
    env->banked_r13[bn] = env->regs[13];
    env->banked_r14[bn] = env->regs[14];
    env->banked_spsr[bn] = env->spsr;

    /* Now we can safely copy stuff down to the kernel */
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

    cp15.hdr.nmsrs = ARRAY_SIZE(cp15.e);
    cp15.e[0].index = MSR32_INDEX_OF(15, 0, 0, 0, 0); /* MIDR */
    cp15.e[0].data = env->cp15.c0_cpuid;
    cp15.e[1].index = MSR32_INDEX_OF(15, 1, 0, 0, 0); /* SCTLR */
    cp15.e[1].data = env->cp15.c1_sys;

    ret = kvm_vcpu_ioctl(env, KVM_SET_REGS, &regs);
    if (ret == 0) {
        ret = kvm_vcpu_ioctl(env, KVM_SET_MSRS, &cp15);
    }
    return ret;
}

int kvm_arch_get_registers(CPUARMState *env)
{
    struct kvm_regs regs;
    int mode, bn;
    int32_t ret;
    struct cp15 {
        struct kvm_msrs hdr;
        struct kvm_msr_entry e[6];
    } cp15;


    ret = kvm_vcpu_ioctl(env, KVM_GET_REGS, &regs);
    if (ret < 0) {
        return ret;
    }

    /* First, let's transfer the banked state */
    cpsr_write(env, regs.cpsr, 0xFFFFFFFF);
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
    env->banked_spsr[5] = regs.spsr[MODE_FIQ];
    env->banked_spsr[4] = regs.spsr[MODE_IRQ];
    env->banked_spsr[1] = regs.spsr[MODE_SVC];
    env->banked_spsr[2] = regs.spsr[MODE_ABT];
    env->banked_spsr[3] = regs.spsr[MODE_UND];

    /* We make sure the current mode regs are properly set */
    mode = env->uncached_cpsr & CPSR_M;
    bn = bank_number(env, mode);
    if (mode == ARM_CPU_MODE_FIQ) {
        memcpy(env->regs + 8, env->fiq_regs, 5 * sizeof(uint32_t));
    } else {
        memcpy(env->regs + 8, env->usr_regs, 5 * sizeof(uint32_t));
    }
    env->regs[13] = env->banked_r13[bn];
    env->regs[14] = env->banked_r14[bn];
    env->spsr = env->banked_spsr[bn];

    /* TODO: investigate automatically getting all registers
     * we know about via the ARMCPU cp_regs hashtable.
     */
    cp15.hdr.nmsrs = ARRAY_SIZE(cp15.e);
    cp15.e[0].index = MSR32_INDEX_OF(15, 0, 0, 0, 0); /* MIDR */
    cp15.e[1].index = MSR32_INDEX_OF(15, 1, 0, 0, 0); /* SCTLR */
    cp15.e[2].index = MSR32_INDEX_OF(15, 2, 0, 0, 0); /* TTBR0 */
    cp15.e[3].index = MSR32_INDEX_OF(15, 2, 0, 0, 1); /* TTBR1 */
    cp15.e[4].index = MSR32_INDEX_OF(15, 2, 0, 0, 2); /* TTBCR */
    cp15.e[5].index = MSR32_INDEX_OF(15, 3, 0, 0, 0); /* DACR */

    ret = kvm_vcpu_ioctl(env, KVM_GET_MSRS, &cp15);
    if (ret < 0) {
        return ret;
    }

    env->cp15.c1_sys = cp15.e[1].data;
    env->cp15.c2_base0 = cp15.e[2].data;
    env->cp15.c2_base1 = cp15.e[3].data;

    /* This is ugly, but necessary for GDB compatibility
     * TODO: do this via an access function.
     */
    env->cp15.c2_control = cp15.e[4].data;
    env->cp15.c2_mask = ~(((uint32_t)0xffffffffu) >> cp15.e[4].data);
    env->cp15.c2_base_mask = ~((uint32_t)0x3fffu >> cp15.e[4].data);

    env->cp15.c3 = cp15.e[5].data;
    return 0;
}

void kvm_arch_pre_run(CPUARMState *env, struct kvm_run *run)
{
}

void kvm_arch_post_run(CPUARMState *env, struct kvm_run *run)
{
}

int kvm_arch_handle_exit(CPUARMState *env, struct kvm_run *run)
{
    int ret = 0;

    return ret;
}

void kvm_arch_reset_vcpu(CPUARMState *env)
{
}

bool kvm_arch_stop_on_emulation_error(CPUARMState *env)
{
    return true;
}

int kvm_arch_process_async_events(CPUARMState *env)
{
    return 0;
}

int kvm_arch_on_sigbus_vcpu(CPUARMState *env, int code, void *addr)
{
    return 1;
}

int kvm_arch_on_sigbus(int code, void *addr)
{
    return 1;
}

void kvm_arch_update_guest_debug(CPUARMState *env, struct kvm_guest_debug *dbg)
{
    fprintf(stderr, "%s: not implemented\n", __func__);
}

int kvm_arch_insert_sw_breakpoint(CPUARMState *env,
                                  struct kvm_sw_breakpoint *bp)
{
    fprintf(stderr, "%s: not implemented\n", __func__);
    return -EINVAL;
}

int kvm_arch_insert_hw_breakpoint(target_ulong addr,
                                  target_ulong len, int type)
{
    fprintf(stderr, "%s: not implemented\n", __func__);
    return -EINVAL;
}

int kvm_arch_remove_hw_breakpoint(target_ulong addr,
                                  target_ulong len, int type)
{
    fprintf(stderr, "%s: not implemented\n", __func__);
    return -EINVAL;
}

int kvm_arch_remove_sw_breakpoint(CPUARMState *env,
                                  struct kvm_sw_breakpoint *bp)
{
    fprintf(stderr, "%s: not implemented\n", __func__);
    return -EINVAL;
}

void kvm_arch_remove_all_hw_breakpoints(void)
{
    fprintf(stderr, "%s: not implemented\n", __func__);
}
