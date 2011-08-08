/*
 * ARM TrustZone simulator infrastructure.
 *
 * Copyright (c) 2011 IAIK, Graz University of Technology
 * Written by Johannes Winter
 *
 * This code is licenced under the GPL.
 */
#include "cpu.h"
#include "dyngen-exec.h"
#include "monitor.h"
#include "qdev.h"
#include "qdev-addr.h"

#include "arm_trustzone.h"

static ASCBus *arm_trustzone_get_subpartitions(ASCPartition* part);

/* Address space control root bus (TODO: get this from CPU env or similar) */
static ASCBus *asc_root_bus = NULL;

#define DEBUG_ARM_TRUSTZONE 
/* #define DEBUG_TRUSTZONE_ACCESS_CHECK */

#ifdef DEBUG_ARM_TRUSTZONE
#define armtz_dbg(...) do {                     \
    fprintf(stderr, "armtz: " __VA_ARGS__);     \
    fputc('\n', stderr);                        \
  } while(0)
#else
#define armtz_dbg(...) do {                     \
  } while(0)
#endif

#ifdef DEBUG_TRUSTZONE_ACCESS_CHECK
#define armtz_check_dbg(...) armtz_dbg(__VA_ARGS__)
#else
#define armtz_check_dbg(...)
#endif

/*
 * Computes the effective access permissions to a TrustZone protected
 * memory region.
 *
 * @cpu: CPU requesting the access
 * @addr: Target address to access
 * @is_secure: Assumed CPU state for the access
 * @prot: Requested access permissions (PAGE_WRITE, PAGE_READ, PAGE_EXEC)
 * @return: Granted access permissions (zero indicates fault)
 */
int arm_trustzone_check_access(CPUState *cpu, target_phys_addr_t addr,
                               int is_secure, int prot)
{
  int granted = prot;
  ASCBus *asc_bus = asc_root_bus;

  /* Search for the root partition */  
  while (granted && asc_bus) {
    ASCPartition *partition = arm_trustzone_get_partition(asc_bus, addr);
    if (!partition) {
      break;
    }

    /* Apply restrictions of this partition */
    granted &= is_secure? partition->s_prot : partition->n_prot;

    /* Check for sub-partitions */
    asc_bus = arm_trustzone_get_subpartitions(partition);
  }

  /* Show the access check results */
  armtz_check_dbg("cpu%d: access [%ssecure/%ssecure] %08lx %c%c%c -> grant %c%c%c",
                  cpu->cpu_index, is_secure? "" : "non", arm_is_secure(cpu, 1)? "" : "non",
                  (unsigned long)addr, 
                  (prot & PAGE_READ)? 'r' : '-', (prot & PAGE_WRITE)? 'w' : '-',
                  (prot & PAGE_EXEC)? 'x' : '-', (granted & PAGE_READ)? 'r' : '-', 
                  (granted & PAGE_WRITE)? 'w' : '-', (granted & PAGE_EXEC)? 'x' : '-');

  return granted;
}

/*
 * Sets the effective TrustZone access permissions for a given 
 * memory partition.
 *
 * @part: Partition to update
 * @n_prot: New effective normal world permissions.
 * @s_prot: New effective secure world permissions.
 */
void arm_trustzone_set_access(ASCPartition *part,
                              int n_prot, int s_prot)
{
  armtz_dbg("assigning new permissions of %08lx..%08lx n:%c%c%c s:%c%c%c",
            (unsigned long)part->phys_start, (unsigned long)part->phys_end,
            (n_prot & PAGE_READ)? 'r' : '-', (n_prot & PAGE_WRITE)? 'w' : '-',
            (n_prot & PAGE_EXEC)? 'x' : '-', (s_prot & PAGE_READ)? 'r' : '-', 
            (s_prot & PAGE_WRITE)? 'w' : '-', (s_prot & PAGE_EXEC)? 'x' : '-');

  // TODO: Queue MMU TLB flush
  part->n_prot = n_prot;
  part->s_prot = s_prot;
}

/*----------------------------------------------------------------------
 * Virtual TrustZone Address Space Control Bus
 *----------------------------------------------------------------------*/
struct ASCBus 
{
  BusState qbus;
};

static void arm_ascbus_dev_print(Monitor *mon, DeviceState *dev, int indent);

static struct BusInfo asc_bus_info = {
  .name = "trustzone_partitions",
  .size = sizeof(ASCBus),
  .print_dev = arm_ascbus_dev_print,
  .props = (Property[]) {
    DEFINE_PROP_TADDR("start", ASCPartition, phys_start, 0xFFFFFFFF),
    DEFINE_PROP_TADDR("end",   ASCPartition, phys_end,   0x00000000),
    DEFINE_PROP_UINT32("nprot", ASCPartition, n_prot, 0),
    DEFINE_PROP_UINT32("sprot", ASCPartition, s_prot, PAGE_READ|PAGE_WRITE|PAGE_EXEC),
    DEFINE_PROP_END_OF_LIST(),
  }
};

static const VMStateDescription vmstate_asc_bus = {
  .name = "trustzone_partitions",
  .version_id = 1,
  .minimum_version_id = 1,
  .minimum_version_id_old = 1,
  .pre_save = NULL,
  .post_load = NULL,
  .fields      = (VMStateField []) {
    VMSTATE_END_OF_LIST()
  }
};

static inline void print_prot(Monitor *mon, int indent, const char *name, int prot)
{
  monitor_printf(mon, "%*s%s %c%c%c\n",
                 indent, "", name,
                 prot & PAGE_READ? 'r' : '-',
                 prot & PAGE_WRITE? 'w' : '-',
                 prot & PAGE_EXEC? 'x' : '-');
}

static void arm_ascbus_dev_print(Monitor *mon, DeviceState *dev, int indent)
{
  ASCPartition *p = ASC_PARTITION_FROM_QDEV(dev);

  monitor_printf(mon, "%*smem " TARGET_FMT_plx "-" TARGET_FMT_plx "\n",
                 indent, "", p->phys_start, p->phys_end);
  print_prot(mon, indent, "nprot", p->n_prot);
  print_prot(mon, indent, "sprot", p->s_prot);
}

ASCPartition *arm_trustzone_get_partition(ASCBus *bus, target_phys_addr_t addr)
{
  ASCPartition *partition = NULL;
  DeviceState *qdev;

  if (!bus) {
    return NULL;
  }

  QLIST_FOREACH(qdev, &bus->qbus.children, sibling) {
    ASCPartition* candidate = ASC_PARTITION_FROM_QDEV(qdev);
    if (candidate->phys_start <= addr && addr <= candidate->phys_end) {
      partition = candidate;
      break;
    }
  }

  return partition;
}

static ASCBus *arm_trustzone_get_subpartitions(ASCPartition* part)
{
  return FROM_QBUS(ASCBus, qdev_get_child_bus(&part->qdev, "partitions"));
}

/*----------------------------------------------------------------------
 * Virtual TrustZone address space partition
 *----------------------------------------------------------------------*/
static const VMStateDescription vmstate_asc_partition = {
  .name = "trustzone-partition",
  .version_id = 1,
  .minimum_version_id = 1,
  .minimum_version_id_old = 1,
  .pre_save = NULL,
  .post_load = NULL,
  .fields = (VMStateField[]) {
    VMSTATE_UINT32(phys_start, ASCPartition), /* FIXME: Need VMSTATE_TADDR or similar */
    VMSTATE_UINT32(phys_end,   ASCPartition), /* FIXME: Need VMSTATE_TADDR or similar */
    VMSTATE_UINT32(n_prot, ASCPartition),
    VMSTATE_UINT32(s_prot, ASCPartition),
    VMSTATE_END_OF_LIST()
  }
};

static int asc_partition_init(DeviceState *dev, DeviceInfo *base) 
{
  return 0;
}

static DeviceInfo asc_partition_device = {
  .name = "trustzone-partition",
  .size = sizeof(ASCPartition),
  .vmsd = &vmstate_asc_partition,
  .init = asc_partition_init,
  .bus_info = &asc_bus_info,
};

ASCPartition *arm_trustzone_create_partition(target_phys_addr_t phys_start,
                                             target_phys_addr_t phys_end,
                                             int n_prot, int s_prot)
{
  DeviceState *dev;

  dev = qdev_create(&asc_root_bus->qbus, "trustzone-partition");
  qdev_prop_set_taddr(dev, "start", phys_start);
  qdev_prop_set_taddr(dev, "end", phys_end);
  qdev_prop_set_uint32(dev, "nprot", n_prot);
  qdev_prop_set_uint32(dev, "sprot", s_prot);
  qdev_init_nofail(dev);

  return ASC_PARTITION_FROM_QDEV(dev);
}

/*----------------------------------------------------------------------
 * Virtual TrustZone address space root controller
 *----------------------------------------------------------------------*/
static int asc_root_init(SysBusDevice *dev)
{
  ASCBus *bus;

  /* Create the adress space controller bus */  
  bus = FROM_QBUS(ASCBus, qbus_create(&asc_bus_info, &dev->qdev, "partitions"));
  vmstate_register(NULL, -1, &vmstate_asc_bus, bus);
  return 0;
}

ASCBus *arm_trustzone_init_vrootasc(void)
{
  DeviceState *dev;
  assert(asc_root_bus == NULL);
 
  /* Create the (virtual) root address space controller */
  dev = sysbus_create_simple("trustzone_vrootasc", -1, NULL);

  /* And grab the root bus */
  asc_root_bus = FROM_QBUS(ASCBus, qdev_get_child_bus(dev, "partitions"));

  assert(asc_root_bus != NULL);
  return asc_root_bus;
}


static void asc_register_devices(void)
{
  sysbus_register_dev("trustzone_vrootasc", sizeof(SysBusDevice), asc_root_init);
  qdev_register(&asc_partition_device);
}

device_init(asc_register_devices);
