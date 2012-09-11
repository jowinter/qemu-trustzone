/*
 * ARM Generic/Distributed Interrupt Controller
 *
 * Copyright (c) 2006-2007 CodeSourcery.
 * Written by Paul Brook
 *
 * This code is licensed under the GPL.
 */

/* This file contains implementation code for the RealView EB interrupt
 * controller, MPCore distributed interrupt controller and ARMv7-M
 * Nested Vectored Interrupt Controller.
 * It is compiled in two ways:
 *  (1) as a standalone file to produce a sysbus device which is a GIC
 *  that can be used on the realview board and as one of the builtin
 *  private peripherals for the ARM MP CPUs (11MPCore, A9, etc)
 *  (2) by being directly #included into armv7m_nvic.c to produce the
 *  armv7m_nvic device.
 */

#include "sysbus.h"
#include "arm_gic_internal.h"

/*#define DEBUG_GIC*/
#define GIC_SECURITY_EXTENSIONS

#ifdef DEBUG_GIC
#define DPRINTF(fmt, ...) \
    do { fprintf(stderr, "arm_gic: %s: " fmt , gic_is_secure_access(s)? "secure" : "normal" , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...) do {} while(0)
#endif

static const uint8_t gic_id[] = {
    0x90, 0x13, 0x04, 0x00, 0x0d, 0xf0, 0x05, 0xb1
};

#define NUM_CPU(s) ((s)->num_cpu)

static inline int gic_get_current_cpu(gic_state *s)
{
    if (s->num_cpu > 1) {
        return cpu_single_env->cpu_index;
    }
    return 0;
}

static inline int gic_is_secure_access(gic_state *s)
{
    int cpu = gic_get_current_cpu(s);
    return arm_current_secure(qemu_get_cpu(cpu));
}

#if !defined(GIC_SECURITY_EXTENSIONS)
/* TODO: Many places that call this routine could be optimized.  */
/* Update interrupt status after enabled or pending bits have been changed.  */
static void gic_update_simple(gic_state *s, int cpu)
{
    int best_irq;
    int best_prio;
    int irq;
    int level;
    int cm = 1 << cpu;

    s->current_pending[cpu] = 1023;
    if (!s->enabled || !(s->cpu_enabled[cpu] & 0x01)) {
        qemu_irq_lower(s->parent_irq[cpu]);
        return;
    }
    best_prio = 0x100;
    best_irq = 1023;

    for (irq = 0; irq < s->num_irq; irq++) {
        if (GIC_TEST_ENABLED(irq, cm) && GIC_TEST_PENDING(irq, cm)) {
            if (GIC_GET_PRIORITY(irq, cpu) < best_prio) {
                best_prio = GIC_GET_PRIORITY(irq, cpu);
                best_irq = irq;
            }
        }
    }

    level = 0;
    if (best_prio <= s->priority_mask[cpu]) {
        s->current_pending[cpu] = best_irq;
        DPRINTF("Raised pending IRQ %d\n", best_irq);
        level = 1;
    }

    qemu_set_irq(s->parent_irq[cpu], level);
}
#endif /* !GIC_SECURITY_EXTENSIONS */

#ifdef GIC_SECURITY_EXTENSIONS
/*
 * NOTE: TrustZone: GIC update logic for TrustZone enabled cores
 *
 * TODO: Check with pseudo-code snippet "3.6.3 Exception generation pseudocode
 * with the Security Extensions" in [ARM IHI 0048A].
 *
 * TODO: BUGS AHEAD, GO GRAB A FLY SWATTER! (ACTIVE IRQ HANDLING IS BROKEN)
 */
static void gic_update_trustzone(gic_state *s, int cpu)
{
    int cm = 1 << cpu;
    int next_irq = 0;
    int next_fiq = 0;
    int best_prio;
    int best_irq;
    int irq;

    s->current_pending[cpu] = 1023;
    if (!s->enabled || !(s->cpu_enabled[cpu] & 0x03)) {
        qemu_irq_lower(s->parent_irq[cpu]);
        qemu_irq_lower(s->parent_fiq[cpu]);
        return;
    }

    /* Find the highest pending IRQ */
    best_prio = 0x100;
    best_irq = 1023;
    
    for (irq = 0; irq < s->num_irq; irq++) {
        if (GIC_TEST_ENABLED(irq, cm) && GIC_TEST_PENDING(irq, cm)) {
            if (GIC_GET_PRIORITY(irq, cpu) < best_prio) {
                if (GIC_TEST_SECURE(best_irq, cm) &&
                    (s->cpu_enabled[cpu] & 0x01)) {
                    best_prio = GIC_GET_PRIORITY(irq, cpu);
                    best_irq = irq;
                } else if (!GIC_TEST_SECURE(best_irq, cm) &&
                           (s->cpu_enabled[cpu] & 0x02)) {
                    best_prio = GIC_GET_PRIORITY(irq, cpu);
                    best_irq = irq;
                }
            }
        }
    }

    if (best_irq != 1023) {
        int secure_int = GIC_TEST_SECURE(best_irq, cm);
        int enable_s  = (s->cpu_enabled[cpu] & 0x01);
        int enable_ns = (s->cpu_enabled[cpu] & 0x02);
        int fiq_en    = (s->cpu_enabled[cpu] & 0x08);

        if (s->running_irq[cpu] == 1023) {            
            /* No active interrupt */
            if (secure_int && enable_s) {
                if (fiq_en) {
                    DPRINTF("Signaling secure FIQ %d\n", best_irq);
                    s->current_pending[cpu] = best_irq;
                    next_fiq = 1;
                } else {
                    DPRINTF("Signaling secure IRQ %d\n", best_irq);
                    s->current_pending[cpu] = best_irq;
                    next_irq = 1; /* Secure IRQ signaled on IRQ */
                }

            } else if (!secure_int && enable_ns) {
                DPRINTF("Signaling normal IRQ %d\n", best_irq);
                s->current_pending[cpu] = best_irq;
                next_irq = 1;
            }
        } else {
            int active_prio = GIC_GET_PRIORITY(s->running_irq[cpu], cpu);

            /* Currently active interrupts */
            if (secure_int && enable_s) {
                if (best_prio < active_prio) {
                    if (fiq_en) {
                        DPRINTF("Prrempt normal IRQ%d by secure FIQ%d\n", 
                                s->running_irq[cpu], best_irq);

                        next_fiq = 1;
                        s->current_pending[cpu] = best_irq;
                    } else {
                        DPRINTF("Prrempt normal IRQ%d by secure IRQ%d\n", 
                                s->running_irq[cpu], best_irq);

                        next_irq = 1;
                        s->current_pending[cpu] = best_irq;
                    }
                }

            } else if (!secure_int && enable_ns) {
                if (best_prio < active_prio) {
                    DPRINTF("Normal preemption of active IRQ%d by IRQ%d\n", 
                            s->running_irq[cpu], best_irq);

                    next_irq = 1;
                    s->current_pending[cpu] = best_irq;
                }
            } else {
                DPRINTF("No preemption of active IRQ%d by IRQ%d\n", 
                        s->running_irq[cpu], best_irq);
            }
        }
    }

    qemu_set_irq(s->parent_irq[cpu], next_irq);
    qemu_set_irq(s->parent_fiq[cpu], next_fiq);
}
#endif

void gic_update(gic_state *s)
{
    int cpu;

    for (cpu = 0; cpu < NUM_CPU(s); cpu++) {
#ifdef GIC_SECURITY_EXTENSIONS
        gic_update_trustzone(s, cpu);       
#else
        gic_update_simple(s, cpu);
#endif
    }
}

void gic_set_pending_private(gic_state *s, int cpu, int irq)
{
    int cm = 1 << cpu;

    if (GIC_TEST_PENDING(irq, cm))
        return;

    DPRINTF("Set %d pending cpu %d\n", irq, cpu);
    GIC_SET_PENDING(irq, cm);
    gic_update(s);
}

/* Process a change in an external IRQ input.  */
static void gic_set_irq(void *opaque, int irq, int level)
{
    /* Meaning of the 'irq' parameter:
     *  [0..N-1] : external interrupts
     *  [N..N+31] : PPI (internal) interrupts for CPU 0
     *  [N+32..N+63] : PPI (internal interrupts for CPU 1
     *  ...
     */
    gic_state *s = (gic_state *)opaque;
    int cm, target;
    if (irq < (s->num_irq - GIC_INTERNAL)) {
        /* The first external input line is internal interrupt 32.  */
        cm = ALL_CPU_MASK;
        irq += GIC_INTERNAL;
        target = GIC_TARGET(irq);
    } else {
        int cpu;
        irq -= (s->num_irq - GIC_INTERNAL);
        cpu = irq / GIC_INTERNAL;
        irq %= GIC_INTERNAL;
        cm = 1 << cpu;
        target = cm;
    }

    if (level == GIC_TEST_LEVEL(irq, cm)) {
        return;
    }

    if (level) {
        GIC_SET_LEVEL(irq, cm);
        if (GIC_TEST_TRIGGER(irq) || GIC_TEST_ENABLED(irq, cm)) {
            DPRINTF("Set %d pending mask %x\n", irq, target);
            GIC_SET_PENDING(irq, target);
        }
    } else {
        GIC_CLEAR_LEVEL(irq, cm);
    }
    gic_update(s);
}

static void gic_set_running_irq(gic_state *s, int cpu, int irq)
{
    s->running_irq[cpu] = irq;
    if (irq == 1023) {
        s->running_priority[cpu] = 0x100;
    } else {
        s->running_priority[cpu] = GIC_GET_PRIORITY(irq, cpu);
    }
    gic_update(s);
}

uint32_t gic_acknowledge_irq(gic_state *s, int cpu)
{
    /* TODO: TrustZone: Determine correct IRQ */
    int new_irq;
    int cm = 1 << cpu;
    int secure_irq;
    int secure_access;
    new_irq = s->current_pending[cpu];
    if (new_irq == 1023
            || GIC_GET_PRIORITY(new_irq, cpu) >= s->running_priority[cpu]) {
        return 1023;
    }
    /* NOTE: TrustZone: ACK filter */
    secure_irq = GIC_TEST_SECURE(new_irq, cm);
    secure_access = gic_is_secure_access(s);
    if (!secure_irq) {
        if (s->cpu_enabled[cpu] & 0x04) {
            DPRINTF("NACK to pending normal IRQ %d\n", new_irq);
            return 1023;
        }
    } else if (!secure_access) {
        /* Can not acknowlegde a secure IRQ from normal world */
        DPRINTF("NACK to pending secure IRQ %d\n", new_irq);
        return 1023;
    }
    s->last_active[new_irq][cpu] = s->running_irq[cpu];
    /* Clear pending flags for both level and edge triggered interrupts.
       Level triggered IRQs will be reasserted once they become inactive.  */
    GIC_CLEAR_PENDING(new_irq, GIC_TEST_MODEL(new_irq) ? ALL_CPU_MASK : cm);
    gic_set_running_irq(s, cpu, new_irq);
    DPRINTF("ACK %d\n", new_irq);
    return new_irq;
}

void gic_complete_irq(gic_state *s, int cpu, int irq)
{
    int update = 0;
    int cm = 1 << cpu;
    DPRINTF("EOI %d\n", irq);
    if (irq >= s->num_irq) {
        /* This handles two cases:
         * 1. If software writes the ID of a spurious interrupt [ie 1023]
         * to the GICC_EOIR, the GIC ignores that write.
         * 2. If software writes the number of a non-existent interrupt
         * this must be a subcase of "value written does not match the last
         * valid interrupt value read from the Interrupt Acknowledge
         * register" and so this is UNPREDICTABLE. We choose to ignore it.
         */
        return;
    }
    if (s->running_irq[cpu] == 1023) {
        DPRINTF("EOI %d not pending\n", irq);
        return; /* No active IRQ.  */
    }
    /* Mark level triggered interrupts as pending if they are still
       raised.  */
    if (!GIC_TEST_TRIGGER(irq) && GIC_TEST_ENABLED(irq, cm)
        && GIC_TEST_LEVEL(irq, cm) && (GIC_TARGET(irq) & cm) != 0) {
        DPRINTF("Set %d pending mask %x\n", irq, cm);
        GIC_SET_PENDING(irq, cm);
        update = 1;
    }
    if (irq != s->running_irq[cpu]) {
        /* Complete an IRQ that is not currently running.  */
        int tmp = s->running_irq[cpu];
        while (s->last_active[tmp][cpu] != 1023) {
            if (s->last_active[tmp][cpu] == irq) {
                s->last_active[tmp][cpu] = s->last_active[irq][cpu];
                break;
            }
            tmp = s->last_active[tmp][cpu];
        }
        if (update) {
            gic_update(s);
        }
    } else {
        /* Complete the current running IRQ.  */
        gic_set_running_irq(s, cpu, s->last_active[s->running_irq[cpu]][cpu]);
    }
}

static uint32_t gic_dist_readb(void *opaque, target_phys_addr_t offset)
{
    gic_state *s = (gic_state *)opaque;
    uint32_t res;
    int irq;
    int i;
    int cpu;
    int cm;
    int mask;

    cpu = gic_get_current_cpu(s);
    cm = 1 << cpu;
    if (offset < 0x80) {
        if (offset == 0)
            return s->enabled;
        if (offset == 4)
            return ((s->num_irq / 32) - 1) | ((NUM_CPU(s) - 1) << 5);
        if (offset < 0x08)
            return 0;
        goto bad_reg;
    } else if (offset < 0x100) {
        /* Interrupt Security , RAZ/WI */
        irq = (offset - 0x080) * 8;
        if (irq >= s->num_irq)
            goto bad_reg;

        res = 0;

        /* NOTE: TrustZone: Read interrupt security status (or zero) */
        if (gic_is_secure_access(s)) {
            mask = (irq < GIC_INTERNAL) ? cm : ALL_CPU_MASK;
            for (i = 0; i < 8; i++) {
                if (!GIC_TEST_SECURE(irq + i, cm)) {
                    res |= (1 << i);
                }
            }
        }

        DPRINTF("READ ICDISR[IRQ%d..%d] -> 0x%02x\n", irq, irq + 7, res);

    } else if (offset < 0x200) {
        /* Interrupt Set/Clear Enable.  */
        if (offset < 0x180)
            irq = (offset - 0x100) * 8;
        else
            irq = (offset - 0x180) * 8;
        irq += GIC_BASE_IRQ;
        if (irq >= s->num_irq)
            goto bad_reg;
        res = 0;
        for (i = 0; i < 8; i++) {
            /* NOTE: TrustZone: Secure interrupt is RAZ/WI for normal world */
            if (!GIC_TEST_SECURE(irq + i, cm) || gic_is_secure_access(s)) {
                if (GIC_TEST_ENABLED(irq + i, cm)) {
                    res |= (1 << i);
                }
            }
        }
    } else if (offset < 0x300) {
        /* Interrupt Set/Clear Pending.  */
        if (offset < 0x280)
            irq = (offset - 0x200) * 8;
        else
            irq = (offset - 0x280) * 8;
        irq += GIC_BASE_IRQ;
        if (irq >= s->num_irq)
            goto bad_reg;
        res = 0;
        mask = (irq < GIC_INTERNAL) ?  cm : ALL_CPU_MASK;
        for (i = 0; i < 8; i++) {
            /* NOTE: TrustZone: Secure interrupt is RAZ/WI for normal world */
            if (!GIC_TEST_SECURE(irq + i, cm) || gic_is_secure_access(s)) {
                if (GIC_TEST_PENDING(irq + i, mask)) {
                    res |= (1 << i);
                }
            }
        }
    } else if (offset < 0x400) {
        /* Interrupt Active.  */
        irq = (offset - 0x300) * 8 + GIC_BASE_IRQ;
        if (irq >= s->num_irq)
            goto bad_reg;
        res = 0;
        mask = (irq < GIC_INTERNAL) ?  cm : ALL_CPU_MASK;
        for (i = 0; i < 8; i++) {
            /* NOTE: TrustZone: Secure interrupt is RAZ/WI for normal world */
            if (!GIC_TEST_SECURE(irq + i, cm) || gic_is_secure_access(s)) {
                if (GIC_TEST_ACTIVE(irq + i, mask)) {
                    res |= (1 << i);
                }
            }
        }
    } else if (offset < 0x800) {
        /* Interrupt Priority.  */
        irq = (offset - 0x400) + GIC_BASE_IRQ;
        if (irq >= s->num_irq)
            goto bad_reg;

        /* NOTE: TrustZone: Secure interrupt is RAZ/WI for normal world */
        res = GIC_GET_PRIORITY(irq, cpu);

        if (GIC_TEST_SECURE(irq, cm)) {
            
        }
        
    } else if (offset < 0xc00) {
        /* Interrupt CPU Target.  */
        if (s->num_cpu == 1 && s->revision != REV_11MPCORE) {
            /* For uniprocessor GICs these RAZ/WI */
            res = 0;
        } else {
            irq = (offset - 0x800) + GIC_BASE_IRQ;
            if (irq >= s->num_irq) {
                goto bad_reg;
            }

            if (GIC_TEST_SECURE(irq, cm) && !gic_is_secure_access(s)) {
                /* NOTE: TrustZone: Secure interrupt is RAZ/WI for normal world */
                res = 0;
            } else if (irq >= 29 && irq <= 31) {
                res = cm;
            } else {
                res = GIC_TARGET(irq);
            }
        }
    } else if (offset < 0xf00) {
        /* Interrupt Configuration.  */
        irq = (offset - 0xc00) * 2 + GIC_BASE_IRQ;
        if (irq >= s->num_irq)
            goto bad_reg;
        res = 0;
        for (i = 0; i < 4; i++) {
            /* NOTE: TrustZone: Secure interrupt is RAZ/WI for normal world */
            if (!GIC_TEST_SECURE(irq + i, cm) || gic_is_secure_access(s)) {
                if (GIC_TEST_MODEL(irq + i))
                    res |= (1 << (i * 2));
                if (GIC_TEST_TRIGGER(irq + i))
                    res |= (2 << (i * 2));
            }
        }
    } else if (offset < 0xfe0) {
        goto bad_reg;
    } else /* offset >= 0xfe0 */ {
        if (offset & 3) {
            res = 0;
        } else {
            res = gic_id[(offset - 0xfe0) >> 2];
        }
    }
    return res;
bad_reg:
    hw_error("gic_dist_readb: Bad offset %x\n", (int)offset);
    return 0;
}

static uint32_t gic_dist_readw(void *opaque, target_phys_addr_t offset)
{
    uint32_t val;
    val = gic_dist_readb(opaque, offset);
    val |= gic_dist_readb(opaque, offset + 1) << 8;
    return val;
}

static uint32_t gic_dist_readl(void *opaque, target_phys_addr_t offset)
{
    uint32_t val;

    if (offset == 0xF00) {
        /* NOTE: TrustZone: Certain software ... tries to read from this
         * register - we simply pretend 0 */
        DPRINTF("BUG EMULATION: Read from W/O ICDSGIR");
        val = 0;
    } else {
        val = gic_dist_readw(opaque, offset);
        val |= gic_dist_readw(opaque, offset + 2) << 16;
    }

    return val;
}

static void gic_dist_writeb(void *opaque, target_phys_addr_t offset,
                            uint32_t value)
{
    gic_state *s = (gic_state *)opaque;
    int irq;
    int i;
    int cpu;
    int cm;

    cpu = gic_get_current_cpu(s);
    cm = (1 << cpu);
    if (offset < 0x80) {
        if (offset == 0) {
            s->enabled = (value & 1);
            DPRINTF("Distribution %sabled\n", s->enabled ? "En" : "Dis");
        } else if (offset < 4) {
            /* ignored.  */
        } else {
            goto bad_reg;
        }
    } else if (offset < 0x100) {
        /* Interrupt Security Registers, RAZ/WI */
        irq = (offset - 0x80) * 8 + GIC_BASE_IRQ;
        if (irq >= s->num_irq)
            goto bad_reg;

        /* NOTE: TrustZone: Update interrupt security status */
        if (gic_is_secure_access(s)) {
            DPRINTF("WRITE ICDISR[IRQ%d..%d] <- 0x%02x\n", irq, irq + 7,
                    (unsigned) value);
            for (i = 0; i < 8; i++) {
                int is_secure = !(value & (1 << i));
                if (is_secure && !GIC_TEST_SECURE(irq + i, cm)) {
                    DPRINTF("Update security status of IRQ %d to %ssecure\n",
                            irq + i, is_secure ? "" : "non");
                }

                if (is_secure) {
                    GIC_SET_SECURE(irq + i, cm);
                } else {
                    GIC_CLEAR_SECURE(irq + i, cm);
                }

                DPRINTF(" IRQ%d is %ssecure\n", irq + i,
                        GIC_TEST_SECURE(irq + i, cm) ? "" : "non");
            }
        }

        /* TODO: TrustZone: Check interaction with pending interrupts */

    } else if (offset < 0x180) {
        /* Interrupt Set Enable.  */
        irq = (offset - 0x100) * 8 + GIC_BASE_IRQ;
        if (irq >= s->num_irq)
            goto bad_reg;
        if (irq < 16)
          value = 0xff;
        for (i = 0; i < 8; i++) {
            if (value & (1 << i)) {
                int mask = (irq < GIC_INTERNAL) ? (1 << cpu) : GIC_TARGET(irq);

                /* NOTE: TrustZone: Secure interrupt is RAZ/WI for
                 * normal world */
                if (!GIC_TEST_SECURE(irq + i, cm) || gic_is_secure_access(s)) {
                    if (!GIC_TEST_ENABLED(irq + i, cm)) {
                        DPRINTF("Enabled IRQ %d\n", irq + i);
                    }
                    GIC_SET_ENABLED(irq + i, cm);
                }

                /* If a raised level triggered IRQ enabled then mark
                   is as pending.  */
                if (GIC_TEST_LEVEL(irq + i, mask)
                    && !GIC_TEST_TRIGGER(irq + i)) {
                    DPRINTF("Set %d pending mask %x\n", irq + i, mask);
                    GIC_SET_PENDING(irq + i, mask);
                }                
            }
        }
    } else if (offset < 0x200) {
        /* Interrupt Clear Enable.  */
        irq = (offset - 0x180) * 8 + GIC_BASE_IRQ;
        if (irq >= s->num_irq)
            goto bad_reg;
        if (irq < 16)
          value = 0;
        for (i = 0; i < 8; i++) {
            if (value & (1 << i)) {
                /* NOTE: TrustZone: Secure interrupt is RAZ/WI for
                 * normal world */
                if (!GIC_TEST_SECURE(irq + i, cm) || gic_is_secure_access(s)) {
                    if (GIC_TEST_ENABLED(irq + i, cm)) {
                        DPRINTF("Disabled IRQ %d\n", irq + i);
                    }
                    GIC_CLEAR_ENABLED(irq + i, cm);
                } else if (GIC_TEST_ENABLED(irq + i, cm)) {
                    DPRINTF("Reject non-secure disable of IRQ %d\n",
                            irq + i);
                }
            }
        }
    } else if (offset < 0x280) {
        /* Interrupt Set Pending.  */
        irq = (offset - 0x200) * 8 + GIC_BASE_IRQ;
        if (irq >= s->num_irq)
            goto bad_reg;
        if (irq < 16)
          irq = 0;

        for (i = 0; i < 8; i++) {
            if (value & (1 << i)) {
                /* NOTE: TrustZone: Secure interrupt is RAZ/WI for
                 * normal world */
                if (!GIC_TEST_SECURE(irq + i, cm) || gic_is_secure_access(s)) {
                    GIC_SET_PENDING(irq + i, GIC_TARGET(irq));
                } else {
                    DPRINTF("Reject non-secure set pending of IRQ %d\n",
                            irq + i);
                }
            }
        }
    } else if (offset < 0x300) {
        /* Interrupt Clear Pending.  */
        irq = (offset - 0x280) * 8 + GIC_BASE_IRQ;
        if (irq >= s->num_irq)
            goto bad_reg;
        for (i = 0; i < 8; i++) {
            /* NOTE: TrustZone: Secure interrupt is RAZ/WI for normal world */
            if (!GIC_TEST_SECURE(irq + i, cm) || gic_is_secure_access(s)) {
                /* ??? This currently clears the pending bit for all CPUs, even
                   for per-CPU interrupts.  It's unclear whether this is the
                   corect behavior.  */
                if (value & (1 << i)) {
                    GIC_CLEAR_PENDING(irq + i, ALL_CPU_MASK);
                }
            } else {
                DPRINTF("Reject non-secure clear pending of IRQ %d\n",
                        irq + i);
            }
        }
    } else if (offset < 0x400) {
        /* Interrupt Active.  */
        goto bad_reg;
    } else if (offset < 0x800) {

        /* Interrupt Priority.  */
        irq = (offset - 0x400) + GIC_BASE_IRQ;
        if (irq >= s->num_irq)
            goto bad_reg;

        int secure_access = gic_is_secure_access(s);

        /* NOTE: TrustZone: Secure interrupt is RAZ/WI for normal world */
        if (!secure_access) {
            /* Translate to distributor view */
            value = 0x80 | ((value >> 1) & 0x7F);
            /* DPRINTF("Translated priority to %02x\n", value); */
        }

        if (!GIC_TEST_SECURE(irq, cm) || secure_access) {
            if (irq < GIC_INTERNAL) {
                s->priority1[irq][cpu] = value;
            } else {
                s->priority2[irq - GIC_INTERNAL] = value;
            }        
        } else {
            DPRINTF("Reject non-secure priority change of IRQ %d\n",
                    irq);
        }
    } else if (offset < 0xc00) {
        /* Interrupt CPU Target. RAZ/WI on uniprocessor GICs, with the
         * annoying exception of the 11MPCore's GIC.
         */
        if (s->num_cpu != 1 || s->revision == REV_11MPCORE) {
            irq = (offset - 0x800) + GIC_BASE_IRQ;
            if (irq >= s->num_irq) {
                goto bad_reg;
            }

            /* NOTE: TrustZone: Secure interrupt is RAZ/WI for normal world */
            if (!GIC_TEST_SECURE(irq, cm) || gic_is_secure_access(s)) {
                if (irq < 29) {
                    value = 0;
                } else if (irq < GIC_INTERNAL) {
                    value = ALL_CPU_MASK;
                }
                s->irq_target[irq] = value & ALL_CPU_MASK;
            } else {
                DPRINTF("Reject non-secure target change of IRQ %d\n",
                        irq);
            }
        }
    } else if (offset < 0xf00) {
        /* Interrupt Configuration.  */
        irq = (offset - 0xc00) * 4 + GIC_BASE_IRQ;
        if (irq >= s->num_irq)
            goto bad_reg;
        if (irq < GIC_INTERNAL)
            value |= 0xaa;
        for (i = 0; i < 4; i++) {
            /* NOTE: TrustZone: Secure interrupt is RAZ/WI for normal world */
            if (!GIC_TEST_SECURE(irq + i, cm) || gic_is_secure_access(s)) {
                if (value & (1 << (i * 2))) {
                    GIC_SET_MODEL(irq + i);
                } else {
                    GIC_CLEAR_MODEL(irq + i);
                }
                if (value & (2 << (i * 2))) {
                    GIC_SET_TRIGGER(irq + i);
                } else {
                    GIC_CLEAR_TRIGGER(irq + i);
                }
            } else {
                DPRINTF("Reject non-secure configuration change of IRQ %d\n",
                        irq + i);
            }
        }
    } else {
        /* 0xf00 is only handled for 32-bit writes.  */
        goto bad_reg;
    }
    gic_update(s);
    return;
bad_reg:
    hw_error("gic_dist_writeb: Bad offset %x\n", (int)offset);
}

static void gic_dist_writew(void *opaque, target_phys_addr_t offset,
                            uint32_t value)
{
    gic_dist_writeb(opaque, offset, value & 0xff);
    gic_dist_writeb(opaque, offset + 1, value >> 8);
}

static void gic_dist_writel(void *opaque, target_phys_addr_t offset,
                            uint32_t value)
{
    gic_state *s = (gic_state *)opaque;
    if (offset == 0xf00) {
        int cpu;
        int irq;
        int mask;

        cpu = gic_get_current_cpu(s);
        irq = value & 0x3ff;

        switch ((value >> 24) & 3) {
        case 0:
            mask = (value >> 16) & ALL_CPU_MASK;
            break;
        case 1:
            mask = ALL_CPU_MASK ^ (1 << cpu);
            break;
        case 2:
            mask = 1 << cpu;
            break;
        default:
            DPRINTF("Bad Soft Int target filter\n");
            mask = ALL_CPU_MASK;
            break;
        }
        GIC_SET_PENDING(irq, mask);
        gic_update(s);
        return;
    }
    gic_dist_writew(opaque, offset, value & 0xffff);
    gic_dist_writew(opaque, offset + 2, value >> 16);
}

static const MemoryRegionOps gic_dist_ops = {
    .old_mmio = {
        .read = { gic_dist_readb, gic_dist_readw, gic_dist_readl, },
        .write = { gic_dist_writeb, gic_dist_writew, gic_dist_writel, },
    },
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static uint32_t gic_cpu_read(gic_state *s, int cpu, int offset)
{
    switch (offset) {
    case 0x00: /* Control */
        /* NOTE: TrustZone: Show correct view of ICCICR, normal world
         * only sees ICCICR.EnableNS as bit 0. */
        if (gic_is_secure_access(s)) {
            return s->cpu_enabled[cpu];
        } else {
            return (s->cpu_enabled[cpu] & 0x02) >> 1;
        }
    case 0x04: /* Priority mask */
        /* TODO: TrustZone: Banking */
        return s->priority_mask[cpu];
    case 0x08: /* Binary Point */
        /* ??? Not implemented.  */
        return 0;
    case 0x0c: /* Acknowledge */
        return gic_acknowledge_irq(s, cpu);
    case 0x14: /* Running Priority */
        return s->running_priority[cpu];
    case 0x18: /* Highest Pending Interrupt */
        return s->current_pending[cpu];
    default:
        hw_error("gic_cpu_read: Bad offset %x\n", (int)offset);
        return 0;
    }
}

static void gic_cpu_write(gic_state *s, int cpu, int offset, uint32_t value)
{
    switch (offset) {
    case 0x00: /* Control */
        /* NOTE: TrustZone: Write to correct view of ICCICR, normal world
         * writes to ICCICR.EnableNS */
        if (gic_is_secure_access(s)) {
            s->cpu_enabled[cpu] = (value & 0x1F);
        } else {
            s->cpu_enabled[cpu] = (s->cpu_enabled[cpu] & ~0x02) |
                ((value & 0x01) << 1);
        }

        DPRINTF("CPU %d sec:%sabled nwd:%sabled\n",
                cpu, (s->cpu_enabled[cpu] & 1) ? "En" : "Dis",
                (s->cpu_enabled[cpu] & 2)? "En" : "Dis");
        break;
    case 0x04: /* Priority mask */
        value &= 0xff;

        if (!gic_is_secure_access(s)) {
            /* NOTE: TrustZone: Normal world can only write range 0x80-0xFF
             * (if current level is in that range. */
            if (value < 0x80 || s->priority_mask[cpu] < value) {
                value = s->priority_mask[cpu];
            }
        }

        s->priority_mask[cpu] = value;
        break;
    case 0x08: /* Binary Point */
        /* ??? Not implemented.  */
        break;
    case 0x10: /* End Of Interrupt */
        return gic_complete_irq(s, cpu, value & 0x3ff);
    default:
        hw_error("gic_cpu_write: Bad offset %x\n", (int)offset);
        return;
    }
    gic_update(s);
}

/* Wrappers to read/write the GIC CPU interface for the current CPU */
static uint64_t gic_thiscpu_read(void *opaque, target_phys_addr_t addr,
                                 unsigned size)
{
    gic_state *s = (gic_state *)opaque;
    return gic_cpu_read(s, gic_get_current_cpu(s), addr);
}

static void gic_thiscpu_write(void *opaque, target_phys_addr_t addr,
                              uint64_t value, unsigned size)
{
    gic_state *s = (gic_state *)opaque;
    gic_cpu_write(s, gic_get_current_cpu(s), addr, value);
}

/* Wrappers to read/write the GIC CPU interface for a specific CPU.
 * These just decode the opaque pointer into gic_state* + cpu id.
 */
static uint64_t gic_do_cpu_read(void *opaque, target_phys_addr_t addr,
                                unsigned size)
{
    gic_state **backref = (gic_state **)opaque;
    gic_state *s = *backref;
    int id = (backref - s->backref);
    return gic_cpu_read(s, id, addr);
}

static void gic_do_cpu_write(void *opaque, target_phys_addr_t addr,
                             uint64_t value, unsigned size)
{
    gic_state **backref = (gic_state **)opaque;
    gic_state *s = *backref;
    int id = (backref - s->backref);
    gic_cpu_write(s, id, addr, value);
}

static const MemoryRegionOps gic_thiscpu_ops = {
    .read = gic_thiscpu_read,
    .write = gic_thiscpu_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps gic_cpu_ops = {
    .read = gic_do_cpu_read,
    .write = gic_do_cpu_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

void gic_init_irqs_and_distributor(gic_state *s, int num_irq)
{
    int i;

    i = s->num_irq - GIC_INTERNAL;
    /* For the GIC, also expose incoming GPIO lines for PPIs for each CPU.
     * GPIO array layout is thus:
     *  [0..N-1] SPIs
     *  [N..N+31] PPIs for CPU 0
     *  [N+32..N+63] PPIs for CPU 1
     *   ...
     */
    if (s->revision != REV_NVIC) {
        i += (GIC_INTERNAL * s->num_cpu);
    }
    qdev_init_gpio_in(&s->busdev.qdev, gic_set_irq, i);
    /* NOTE: TrustZone: hook up the IRQ and FIQ lines */
    for (i = 0; i < NUM_CPU(s); i++) {
        sysbus_init_irq(&s->busdev, &s->parent_irq[i]);
        sysbus_init_irq(&s->busdev, &s->parent_fiq[i]);
    }
    memory_region_init_io(&s->iomem, &gic_dist_ops, s, "gic_dist", 0x1000);
}

static int arm_gic_init(SysBusDevice *dev)
{
    /* Device instance init function for the GIC sysbus device */
    int i;
    gic_state *s = FROM_SYSBUS(gic_state, dev);
    ARMGICClass *agc = ARM_GIC_GET_CLASS(s);

    agc->parent_init(dev);

    gic_init_irqs_and_distributor(s, s->num_irq);

    /* Memory regions for the CPU interfaces (NVIC doesn't have these):
     * a region for "CPU interface for this core", then a region for
     * "CPU interface for core 0", "for core 1", ...
     * NB that the memory region size of 0x100 applies for the 11MPCore
     * and also cores following the GIC v1 spec (ie A9).
     * GIC v2 defines a larger memory region (0x1000) so this will need
     * to be extended when we implement A15.
     */
    memory_region_init_io(&s->cpuiomem[0], &gic_thiscpu_ops, s,
                          "gic_cpu", 0x100);
    for (i = 0; i < NUM_CPU(s); i++) {
        s->backref[i] = s;
        memory_region_init_io(&s->cpuiomem[i+1], &gic_cpu_ops, &s->backref[i],
                              "gic_cpu", 0x100);
    }
    /* Distributor */
    sysbus_init_mmio(dev, &s->iomem);
    /* cpu interfaces (one for "current cpu" plus one per cpu) */
    for (i = 0; i <= NUM_CPU(s); i++) {
        sysbus_init_mmio(dev, &s->cpuiomem[i]);
    }
    return 0;
}

static void arm_gic_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *sbc = SYS_BUS_DEVICE_CLASS(klass);
    ARMGICClass *agc = ARM_GIC_CLASS(klass);
    agc->parent_init = sbc->init;
    sbc->init = arm_gic_init;
    dc->no_user = 1;
}

static TypeInfo arm_gic_info = {
    .name = TYPE_ARM_GIC,
    .parent = TYPE_ARM_GIC_COMMON,
    .instance_size = sizeof(gic_state),
    .class_init = arm_gic_class_init,
};

static void arm_gic_register_types(void)
{
    type_register_static(&arm_gic_info);
}

type_init(arm_gic_register_types)
