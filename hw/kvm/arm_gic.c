/*
 * ARM Generic Interrupt Controller using KVM in-kernel support
 *
 * Copyright (c) 2012 Linaro Limited
 * Written by Peter Maydell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "hw/sysbus.h"
#include "kvm.h"
#include "hw/arm_gic_internal.h"

#define TYPE_KVM_ARM_GIC "kvm-arm_gic"
#define KVM_ARM_GIC(obj) \
     OBJECT_CHECK(gic_state, (obj), TYPE_KVM_ARM_GIC)
#define KVM_ARM_GIC_CLASS(klass) \
     OBJECT_CLASS_CHECK(KVMARMGICClass, (klass), TYPE_KVM_ARM_GIC)
#define KVM_ARM_GIC_GET_CLASS(obj) \
     OBJECT_GET_CLASS(KVMARMGICClass, (obj), TYPE_KVM_ARM_GIC)

typedef struct KVMARMGICClass {
    ARMGICCommonClass parent_class;
    int (*parent_init)(SysBusDevice *dev);
    void (*parent_reset)(DeviceState *dev);
} KVMARMGICClass;

static void kvm_arm_gic_set_irq(void *opaque, int irq, int level)
{
    /* Meaning of the 'irq' parameter:
     *  [0..N-1] : external interrupts
     *  [N..N+31] : PPI (internal) interrupts for CPU 0
     *  [N+32..N+63] : PPI (internal interrupts for CPU 1
     *  ...
     */
    gic_state *s = (gic_state *)opaque;
    struct kvm_irq_level irq_level;

    irq_level.level = level ? 1 : 0;

    if (irq < (s->num_irq - GIC_INTERNAL)) {
        /* External interrupt number 'irq' */
        irq_level.irq = irq + GIC_INTERNAL;
        kvm_vm_ioctl(kvm_state, KVM_IRQ_LINE, &irq_level);
    } else {
        int cpu;
        irq -= (s->num_irq - GIC_INTERNAL);
        cpu = irq / GIC_INTERNAL;
        irq %= GIC_INTERNAL;
        /* Internal interrupt 'irq' for CPU 'cpu' */
        irq_level.irq = irq;
        kvm_vcpu_ioctl(qemu_get_cpu(cpu), KVM_IRQ_LINE, &irq_level);
    }
}

static void kvm_arm_gic_put(gic_state *s)
{
    /* TODO: there isn't currently a kernel interface to set the GIC state */
}

static void kvm_arm_gic_get(gic_state *s)
{
    /* TODO: there isn't currently a kernel interface to get the GIC state */
}

static void kvm_arm_gic_reset(DeviceState *dev)
{
    gic_state *s = ARM_GIC_COMMON(dev);
    KVMARMGICClass *kgc = KVM_ARM_GIC_GET_CLASS(s);
    kgc->parent_reset(dev);
    kvm_arm_gic_put(s);
}

static int kvm_arm_gic_init(SysBusDevice *dev)
{
    /* Device instance init function for the GIC sysbus device */
    int i;
    gic_state *s = FROM_SYSBUS(gic_state, dev);
    KVMARMGICClass *kgc = KVM_ARM_GIC_GET_CLASS(s);

    kgc->parent_init(dev);

    i = s->num_irq - GIC_INTERNAL;
    /* For the GIC, also expose incoming GPIO lines for PPIs for each CPU.
     * GPIO array layout is thus:
     *  [0..N-1] SPIs
     *  [N..N+31] PPIs for CPU 0
     *  [N+32..N+63] PPIs for CPU 1
     *   ...
     */
    i += (GIC_INTERNAL * s->num_cpu);
    qdev_init_gpio_in(&s->busdev.qdev, kvm_arm_gic_set_irq, i);
    /* We never use our outbound IRQ lines but provide them so that
     * we maintain the same interface as the non-KVM GIC.
     */
    for (i = 0; i < s->num_cpu; i++) {
        sysbus_init_irq(&s->busdev, &s->parent_irq[i]);
    }
    /* Distributor */
    memory_region_init_reservation(&s->iomem, "kvm-gic_dist", 0x1000);
    sysbus_init_mmio(dev, &s->iomem);
    /* CPU interface for current core. Unlike arm_gic, we don't
     * provide the "interface for core #N" memory regions, because
     * cores with a VGIC don't have those.
     */
    memory_region_init_reservation(&s->cpuiomem[0], "kvm-gic_cpu", 0x1000);
    sysbus_init_mmio(dev, &s->cpuiomem[0]);
    /* TODO: we should tell the kernel at some point the address
     * of the private peripheral base. However we don't currently have
     * any convenient infrastructure to do that, and in any case the
     * kernel doesn't yet implement an ioctl to let us tell it.
     */
    return 0;
}

static void kvm_arm_gic_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *sbc = SYS_BUS_DEVICE_CLASS(klass);
    ARMGICCommonClass *agcc = ARM_GIC_COMMON_CLASS(klass);
    KVMARMGICClass *kgc = KVM_ARM_GIC_CLASS(klass);
    agcc->pre_save = kvm_arm_gic_get;
    agcc->post_load = kvm_arm_gic_put;
    kgc->parent_init = sbc->init;
    kgc->parent_reset = dc->reset;
    sbc->init = kvm_arm_gic_init;
    dc->reset = kvm_arm_gic_reset;
    dc->no_user = 1;
}

static TypeInfo arm_gic_info = {
    .name = TYPE_KVM_ARM_GIC,
    .parent = TYPE_ARM_GIC_COMMON,
    .instance_size = sizeof(gic_state),
    .class_init = kvm_arm_gic_class_init,
};

static void arm_gic_register_types(void)
{
    type_register_static(&arm_gic_info);
}

type_init(arm_gic_register_types)
