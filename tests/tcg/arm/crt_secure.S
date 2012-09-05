    /**
     * crt-secure.S - Secure-world startup code for the Versatile Express Cortex-A9 tests.
     */
#include "secure.h"

    .global secure_exception_vectors
    .global secure_monitor_vectors
    .global secure_cpu_panic
    .global secure_sys_exit
    .extern secure_cpu_bootmap


    /* Semihosting definitions from arm-semi.c */
#define TARGET_SYS_EXIT        0x18

    /* TARGET_SYS_EXIT (angel_SWIreason_ReportException) status codes */
#define ADP_Stopped_RunTimeErrorUnknown	0x20023 /* used for secure_cpu_panic */
#define ADP_Stopped_ApplicationExit     0x20026 /* used for secure_sys_exit  */

    /*----------------------------------------------------------------------
     * Secure-world exception vectors and secure monitor vectors
     *-----------------------------------------------------------------------*/
    .section ".vectors", "ax"
    .align 5
    .arm

    .macro define_exception_handler name, default_handler=secure_cpu_panic
    /* Create a weak-symbol alias for the default handler */
    .weak \name\()_handler
    .set  \name\()_handler, \default_handler

    /* Push the vector address to subsection 1 (with proper symbol information) */
    .subsection 1
\name\()_vector:
    .long \name\()_handler
    .type \name\()_vector, "object"
    .size \name\()_vector, . - \name\()_vector
    .previous

    /* Emit the PC load operation */
    LDR pc, \name\()_vector
    .endm

    .macro define_reserved_exception_handler default_handler=secure_cpu_panic
    /* Emit the unhandled exception report in subsection 0 */
    B \default_handler
    .endm

    /* Secure-world exception vectors */
    .fnstart
    .cantunwind
secure_exception_vectors:
    B secure_start
    define_exception_handler secure_undef
    define_exception_handler secure_swi
    define_exception_handler secure_pabt
    define_exception_handler secure_dabt
    define_reserved_exception_handler
    define_exception_handler secure_irq
    define_exception_handler secure_fiq
    .fnend
    .type secure_exception_vectors, "function"
    .size secure_exception_vectors, . - secure_exception_vectors

    /* Secure-monitor exception vectors */
    .align 5
    .fnstart
    .cantunwind
secure_monitor_vectors: /* Secure-monitor vectors */
    define_reserved_exception_handler
    define_reserved_exception_handler
    define_exception_handler monitor_smc
    define_exception_handler monitor_pabt
    define_exception_handler monitor_dabt
    define_reserved_exception_handler
    define_exception_handler monitor_irq
    define_exception_handler monitor_fiq
    .fnend
    .type secure_monitor_vectors, "function"
    .size secure_monitor_vectors, . - secure_monitor_vectors

     /*----------------------------------------------------------------------
      * Secure-world SMP-aware startup code.
      *-----------------------------------------------------------------------*/
    .text
    .arm

    /**
     * Common secure-world entry-point for all CPUs.
     *
     * We assume a Linux-compatible boot register layout on entry:
     *  r0 - Zero
     *  r1 - Linux Board ID (0x8e0 for Versatile Express)
     *  r2 - Pointer to kernel arguments (ATAGs)
     *
     * We disable interrupts, enter supvervisor mode and inspect the
     * CPU bootmap to find the next piece of code to run. No other
     * initialization is done here.
     *
     * The test application must provide the CPU bootmap (secure_cpu_bootmap)
     * defining the appropriate startup code.
     */
    .fnstart
    .cantunwind
secure_start:
    CPSID iaf, #SVC_MODE        /* Enter supervisor mode, no interrupts */

    LDR r0, =secure_exception_vectors
    MCR p15, 0, r0, c12, c0, 0  /* Setup Vector Base Address Register */

    LDR r1, =secure_monitor_vectors
    MCR p15, 0, r0, c12, c0, 1  /* Setup Monitor Vector Base Address Register */

    MRC p15, 0, r0, c0, c0, 5   /* Get the current CPU affinity (MPIDR) */
    TST   r0, #0x80000000       /* Force MPIDR==0 when SMP extension are unavailable */
    MOVEQ r0, #0x00000000

    /*
     * Scan the CPU bootmap for the next stage boot code.
     *
     * Identify the next stage boot code based on the MPIDR value of the current CPU.
     * The bootmap entries are defined as:
     *
     * struct bootmap_entry {
     *   uint32_t mpidr_mask;  [r6]
     *   uint32_t mpidr_value; [r7]
     *   uint32_t boot_addr;   [r8]
     *   uint32_t boot_stack;  [sp]
     * };
     *
     * The first bootmap entry satisfying (MPIDR & mpdir_mask) == mpdir_value
     * selects the code to boot. The last entry of the bootmap should must be
     * a catch-all entry handling otherwise unused CPUs.
     */
    LDR r5, =secure_cpu_bootmap

next_bootmap_entry:
    LDMIA r5!, {r6-r8, sp}
    AND r6, r6, r0
    CMP r6, r7
    BNE next_bootmap_entry

    /*
     * We found a suitable bootmap entry
     *
     * Register for the callee:
     *  r0 - MPIDR of calling CPU
     *  r1 - Linux board ID (received from bootloader)
     *  r2 - Linux ATAG pointer (received from bootloader)
     *  sp - SVC mode stack defined by the bootmap entry.
     */
    MOV fp, #0
    LDR lr, =secure_cpu_freeze /* Sane default for C handlers */
    BX r8

    .ltorg
    .fnend
    .type secure_start, "function"
    .size secure_start, . - secure_start

     /**
     * Default trap for unhandled secure world panic events on
     * the current CPU.
     *
     * TODO: Handle incoming IPIs to allow resurrection?
     */
    .fnstart
    .cantunwind
secure_cpu_panic:
    CPSID iaf
#ifdef USE_SEMIHOSTING
    MOV r0, #TARGET_SYS_EXIT
    LDR r1, =ADP_Stopped_RunTimeErrorUnknown
    SWI #0x123456
#endif
    B secure_cpu_freeze
    .ltorg
    .fnend
    .type secure_cpu_panic, "function"
    .size secure_cpu_panic, . - secure_cpu_panic

     /**
     * Default trap for unhandled secure world panic events on
     * the current CPU.
     *
     * TODO: Handle incoming IPIs to allow resurrection?
     */
    .fnstart
    .cantunwind
secure_sys_exit:
    CPSID iaf
#ifdef USE_SEMIHOSTING
    MOV r0, #TARGET_SYS_EXIT
    LDR r1, =ADP_Stopped_ApplicationExit
    SWI #0x123456
#endif
    B secure_cpu_freeze
    .ltorg
    .fnend
    .type secure_sys_exit, "function"
    .size secure_sys_exit, . - secure_sys_exit

     /**
     * Freezes the calling CPU with an endless loop.
     *
     * TODO: Handle incoming IPIs to allow resurrection?
     */
    .fnstart
    .cantunwind
secure_cpu_freeze:
    CPSID iaf
1:  WFI
    B 1b
    .ltorg
    .fnend
    .type secure_cpu_freeze, "function"
    .size secure_cpu_freeze, . - secure_cpu_freeze
