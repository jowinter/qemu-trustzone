/**
 * secure.h - Secure world CRT helpers
 */
#ifndef SECURE_H
#define SECURE_H 1

/* defined CPSR and SPSR bits (selected parts) */
#define PSR_T       0x00000020
#define PSR_F       0x00000040
#define PSR_I       0x00000080
#define PSR_A       0x00000100
#define PSR_E       0x00000200

    /* defined for use with CPS fast mode change instruction */
#define USR_MODE    16
#define FIQ_MODE    17
#define IRQ_MODE    18
#define SVC_MODE    19
#define MON_MODE    22
#define ABT_MODE    23
#define UND_MODE    27
#define SYS_MODE    31

    /* Secure configuration register */
#define SCR_NS  (1 << 0)
#define SCR_IRQ (1 << 1)
#define SCR_FIQ (1 << 2)
#define SCR_EA  (1 << 3)
#define SCR_FW  (1 << 4)
#define SCR_AW  (1 << 5)


#ifdef __ASSEMBLER__
.macro secure_bootmap_begin
    .pushsection ".rodata", "a"
    .global secure_cpu_bootmap
secure_cpu_bootmap:
.endm

.macro secure_bootmap_entry mpidr_value,mpidr_mask,boot_addr,boot_stack
    .long \mpidr_value
    .long \mpidr_mask
    .long \boot_addr
    .long \boot_stack
.endm

.macro secure_bootmap_end default_boot_addr=secure_cpu_panic
    secure_bootmap_entry 0, 0, \default_boot_addr, 0
    .type secure_cpu_bootmap, "object"
    .size secure_cpu_bootmap, . - secure_cpu_bootmap
    .popsection
.endm

.macro secure_bootmap_anycpu boot_addr
    secure_bootmap_begin
    secure_bootmap_end \boot_addr
.endm
#endif

#endif /* SECURE_H */
