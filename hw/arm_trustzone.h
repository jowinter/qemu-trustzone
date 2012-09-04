/*
 * ARM TrustZone memory permission checks
 *
 * Copyright (c) 2011-2012 IAIK, Graz University of Technology
 * Written by Johannes Winter
 *
 * This code is licenced under the GPL.
 */
#ifndef ARM_TRUSTZONE_H
#define ARM_TRUSTZONE_H 1

#ifndef CONFIG_USER_ONLY

#include "cpu.h"
#include "memory.h"

/**
 * BP147DecprotInfo:
 *
 * TrustZone Protection Controller (BP147) bit definition. A
 * BP147 controller can affect SoC properties other than memory
 * decode protection, for example lockdown of other peripherals.
 * An optional user-defined callback function can be used to
 * implement such SoC specific behavior.
 *
 * @reg: Index of the DECPROT register.
 * @bit: Number of the DECPROT register bit.
 * @addr: Physical base address of the protected region.
 * @size: Physical size of the protected region. (can be zero to
 *        disable decode protection control if only the callback
 *        is needed.)
 * @update: User-callback to invoke on register updates.
 * @opaque: Parameter for the bit update callback.
 */
typedef struct BP147DecprotInfo {
    unsigned reg;
    unsigned bit;
    target_phys_addr_t addr;
    target_phys_addr_t size;
    void (*update)(void *opaque, int value);
    void *opaque;
} BP147DecprotInfo;

#define BP147_DECPROT(r,b,a,s) \
    { .reg = (r), .bit = (b), .addr = (a), .size = (s) }

/**
 * bp147_add_region() - Defines a new decode protection region of a BP147
 *                    controller.
 *
 * @d: Device state of the BP147 device.
 * @info: Definition of the new region.
 */
void bp147_add_region(DeviceState *d, const BP147DecprotInfo *info);

/**
 * bp147_create_simple() - Creates a TrustZone Protection Controller
 *   (BP147) peripheral and adds decode protection regions.
 *
 * This convenience function creates and initialized new BP147 instance,
 * adds the user-supplied region definitions establishes an MMIO mapping
 * for the TrustZone Protection Controller.
 *
 * @addr: Physical base address of the BP147 device.
 * @info: Array of decode protection region definitions.
 * @count: Number of regions to define initially.
 */
DeviceState* bp147_create_simple(target_phys_addr_t addr,
                                 const BP147DecprotInfo *info, unsigned count);

/* TODO: TrustZone: API for the decode protection regions */
/**
 * tzc380_create_simple() - Creates a TrustZone Address Space Controller
 *  (TZC380) peripheral and defines the protected physical memory region.
 *
 * This convenience function creates and initialized new TZC380 instance,
 * configures its protected region and establishes an MMIO mapping
 * for the TrustZone Address Space Controller.
 *
 * @addr: Physical base address of the BP147 device.
 */
DeviceState* tzc380_create_simple(target_phys_addr_t addr);

/**
 * arm_check_phys_access() - Check access permissions of a physical memory page.
 *
 * This function checks memory access permission enforced by the TrustZone
 * protection controllers and address space controllers in the system. It is
 * normally invoked by get_phys_addr() after the virtual-to-physical address
 * translation completed successfully.
 *
 * TODO: TrustZone: In theory we could hook the "accepts" method of the MemoryRegion
 * API and call this function to check validity - given that we could communicate the
 * the security status for the memory access and resulting faults in a clean way.
 * (It is not sufficient to check the calling CPU's SCR.NS bit since we might be handling
 * an access from secure world done with normal world privileges).
 *
 * @phys_addr: the physical address to be accessed.
 * @page_size: size of the page to be accessed.
 * @prot: page access permissions of the target page.
 * @is_user: 0 for privileged access, 1 for user
 * @is_secure: 0 for use normal world, 1 for use secue world
 * @return: Zero if access is granted or an MMU fault status code compatible
 *  with get_phys_addr().
 */
int arm_check_phys_access(target_phys_addr_t phys_addr, target_ulong page_size,
                          int is_user, int is_secure, int prot);

#endif /* !CONFIG_USER_ONLY */

#endif /* !ARM_TRUSTZONE_H */
