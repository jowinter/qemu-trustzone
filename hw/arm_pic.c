/*
 * Generic ARM Programmable Interrupt Controller support.
 *
 * Copyright (c) 2006 CodeSourcery.
 * Written by Paul Brook
 *
 * This code is licensed under the LGPL
 */

#include "hw.h"
#include "arm-misc.h"
#include "kvm.h"

/* Input 0 is IRQ and input 1 is FIQ.  */
static void arm_pic_cpu_handler(void *opaque, int irq, int level)
{
    ARMCPU *cpu = opaque;
    CPUARMState *env = &cpu->env;

    switch (irq) {
    case ARM_PIC_CPU_IRQ:
        if (level)
            cpu_interrupt(env, CPU_INTERRUPT_HARD);
        else
            cpu_reset_interrupt(env, CPU_INTERRUPT_HARD);
        break;
    case ARM_PIC_CPU_FIQ:
        if (level)
            cpu_interrupt(env, CPU_INTERRUPT_FIQ);
        else
            cpu_reset_interrupt(env, CPU_INTERRUPT_FIQ);
        break;
    default:
        hw_error("arm_pic_cpu_handler: Bad interrupt line %d\n", irq);
    }
}

#ifdef CONFIG_KVM
static void kvm_arm_pic_cpu_handler(void *opaque, int irq, int level)
{
    ARMCPU *cpu = opaque;
    CPUARMState *env = &cpu->env;
    int kvm_irq;

    switch (irq) {
    case ARM_PIC_CPU_IRQ:
        kvm_irq = KVM_ARM_IRQ_LINE;
        break;
    case ARM_PIC_CPU_FIQ:
        kvm_irq = KVM_ARM_FIQ_LINE;
        break;
    default:
        hw_error("kvm_arm_pic_cpu_handler: Bad interrupt line %d\n", irq);
    }
    kvm_irq |= (env->cpu_index << 1);
    kvm_set_irq(kvm_state, kvm_irq, level ? 1 : 0);
}
#endif

qemu_irq *arm_pic_init_cpu(ARMCPU *cpu)
{
#ifdef CONFIG_KVM
    if (kvm_enabled()) {
        return qemu_allocate_irqs(kvm_arm_pic_cpu_handler, cpu, 2);
    }
#endif
    return qemu_allocate_irqs(arm_pic_cpu_handler, cpu, 2);
}
