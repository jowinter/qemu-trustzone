/*
 * ARM TrustZone simulator infrastructure.
 *
 * Copyright (c) 2011 IAIK, Graz University of Technology
 * Written by Johannes Winter
 *
 * This code is licenced under the GPL.
 */
#ifndef ARM_TRUSTZONE_H
#define ARM_TRUSTZONE_H 1

#include "cpu.h"
#include "qdev.h"
#include "sysbus.h"

/*----------------------------------------------------------------------
 * TrustZone Address Space model
 *--------------------------------------------------------------------*/
typedef struct ASCBus       ASCBus;
typedef struct ASCPartition ASCPartition;

/*
 * TrustZone address space partition
 */
struct ASCPartition {
  DeviceState qdev;

  target_phys_addr_t phys_start; /* Physical start address of this partition */
  target_phys_addr_t phys_end;   /* Physical end address of this partition */
  uint32_t n_prot; /* Static normal world PAGE_* permissions */
  uint32_t s_prot; /* Static secure world PAGE_* permissions */
};

#define ASC_PARTITION_FROM_QDEV(dev) DO_UPCAST(ASCPartition, qdev, dev)

/*
 * Initializes the virtual root address space controller for use
 * by the TrustZone address space model.
 *
 * @return: The new virtual root address space controller.
 */
ASCBus *arm_trustzone_init_vrootasc(void);

/*
 * Searches for the first partition covering the given address.
 *
 * @bus: Virtual address space control bus to search.
 * @addr: Address to search
 * @return: The first matching partition or NULL if no partition
 *   could be found.
 */
ASCPartition *arm_trustzone_get_partition(ASCBus *bus, target_phys_addr_t addr);

/*
 * Defines a new memory partition in a TrustZone enabled system.
 */
ASCPartition *arm_trustzone_create_partition(target_phys_addr_t phys_start,
                                             target_phys_addr_t phys_end,
                                             int n_prot, int s_prot);

void arm_trustzone_set_access(ASCPartition *partition, int n_prot, int s_prot);

/*----------------------------------------------------------------------
 * TrustZone Simulator Infrastructure
 *--------------------------------------------------------------------*/


void arm_trustzone_ext_dabort(CPUState *cpu, target_phys_addr_t addr,
                              int is_write);
int arm_trustzone_check_access(CPUState *cpu, target_phys_addr_t addr, int is_secure, int prot);

/*----------------------------------------------------------------------
 * TrustZone Protection Controller (BP147)
 *--------------------------------------------------------------------*/
void bp147_attach_slave(SysBusDevice *bp147, SysBusDevice *slave, 
                        int idx, int bit);

int bp147_attach_simple(SysBusDevice *bp147, int idx, int bit, const char *qname);

#endif /* ARM_TRUSTZONE_H */
