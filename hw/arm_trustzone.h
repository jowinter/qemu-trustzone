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
 * arm_check_phys_access - check access permissions of a physical memory page.
 *
 * Check the TrustZone access permissions of a physical memory page.
 *
 * @phys_addr: the physical address to be accessed.
 * @page_size: size of the page to be accessed.
 * @prot: page access permissions of the target page.
 * @is_user: 0 for privileged access, 1 for user
 * @is_secure: 0 for use normal world, 1 for use secue world
 */
int arm_check_phys_access(target_phys_addr_t phys_addr, target_ulong page_size,
                          int is_user, int is_secure, int prot);

#endif /* !CONFIG_USER_ONLY */

#endif /* !ARM_TRUSTZONE_H */
