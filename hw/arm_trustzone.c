/*
 * ARM TrustZone memory permission checks
 *
 * Copyright (c) 2011-2012 IAIK, Graz University of Technology
 * Written by Johannes Winter
 *
 * This code is licenced under the GPL.
 */
#include "arm_trustzone.h"

#ifndef CONFIG_USER_ONLY
int arm_check_phys_access(uint64_t phys_addr, target_ulong page_size,
                          int is_user, int is_secure, int prot)
{
    /* TODO: TrustZone: Check memory permissions */
    return 0;
}
#endif /* !CONFIG_USER_ONLY */
