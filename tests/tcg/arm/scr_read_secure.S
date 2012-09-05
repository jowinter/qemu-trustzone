    /**
     * scr_read_secure.S - Simple secure world tests with the SCR register.
     *
     * We try to modify SCR in different processor modes, including manipulation
     * of the SCR.NS bit in secure monitor mode.
     *
     * We use the CP15 TPIDRURW (CP15 User Read/Write Thread ID) to set up one-shot
     * handlers for undefined instructions.
     */
    #include "secure.h"

    .extern secure_cpu_panic
    .extern secure_sys_exit

    /* Boot the read SCR test on all CPUs */
    secure_bootmap_anycpu test_read_scr

    /* Expect next instruction to be undefined */
    .macro expect_next_undef tmp=ip
    ADR \tmp, 1f + 4             /* Expected LR value of undef exception */
    MCR p15, 0, \tmp, c13, c0, 2 /* CP15 User Read/Write Thread ID */
1:
    .endm

    /* Assert that expect_next_undef fired. */
    .macro assert_prev_undef tmp=ip
    MRC p15, 0, \tmp, c13, c0, 2 /* CP15 User Read/Write Thread ID */
    CMN \tmp, #0
    BEQ secure_cpu_panic
    .endm

    /* Expect no undefined. */
    .macro expect_no_undef tmp=ip
    MVN \tmp, #0                 /* Set invalid expected LR value */
    MCR p15, 0, \tmp, c13, c0, 2 /* CP15 User Read/Write Thread ID */
    .endm

    .text
    .arm

    .fnstart
    .cantunwind
test_read_scr:
    expect_no_undef

    CPS #MON_MODE              /* Switch to secure monitor mode */
    MRC p15, 0, r0, c1, c1, 0  /* Read the initial SCR value */
    ORR r1, r0, #(SCR_AW | SCR_FW | SCR_FIQ | SCR_IRQ | SCR_NS)
    MCR p15, 0, r1, c1, c1, 0  /* Write new SCR value */
    BIC r1, #SCR_NS            /* Clear the SCR.NS bit again */
    MCR p15, 0, r1, c1, c1, 0

    CPS #SVC_MODE
    MRC p15, 0, r2, c1, c1, 0
    MCR p15, 0, r2, c1, c1, 0

    CPS #ABT_MODE
    MRC p15, 0, r2, c1, c1, 0
    MCR p15, 0, r2, c1, c1, 0

    CPS #UND_MODE
    MRC p15, 0, r2, c1, c1, 0
    MCR p15, 0, r2, c1, c1, 0

    CPS #FIQ_MODE
    MRC p15, 0, r2, c1, c1, 0
    MCR p15, 0, r2, c1, c1, 0

    CPS #IRQ_MODE
    MRC p15, 0, r2, c1, c1, 0
    MCR p15, 0, r2, c1, c1, 0

    CPS #SYS_MODE
    MRC p15, 0, r2, c1, c1, 0
    MCR p15, 0, r2, c1, c1, 0

    CPS #USR_MODE

    expect_next_undef
    MRC p15, 0, r2, c1, c1, 0 /* Expect: UNDEF trap */
    assert_prev_undef

    MOV r2, #0

    expect_next_undef
    MCR p15, 0, r2, c1, c1, 0 /* Expect: UNDEF trap */
    assert_prev_undef

    /* Trigger our SWI from user-mode */
    SWI #0x18
    B .

    .ltorg
    .fnend
    .type test_read_scr, "function"
    .size test_read_scr, . - test_read_scr

    /*
     * Undefined instruction handler (minimal version)
     */
    .global secure_undef_handler
    .fnstart
    .cantunwind
secure_undef_handler:
    MRC p15, 0, ip, c13, c0, 2 /* CP15 User Read/Write Thread ID */
    CMP ip, lr
    MCRNE p15, 0, lr, c13, c0, 2
    BNE secure_cpu_panic       /* Panic if wrong handler */

    MVN ip, #0                 /* Reset one-short handler */
    MCR p15, 0, ip, c13, c0, 2 /* CP15 User Read/Write Thread ID */
    MOVS pc, lr
    .fnend
    .type secure_undef_handler, "function"
    .size secure_undef_handler, . - secure_undef_handler

    /*
     * Poor man's semicall forwarding to QEMU
     */
    .global secure_swi_handler
    .fnstart
    .cantunwind
secure_swi_handler:
    LDR ip, [lr, #-4]
    SUB ip, ip, #0xef000000
    CMP ip, #0x18
    BEQ secure_sys_exit
    MOVS pc, lr
    .ltorg
    .fnend
    .type secure_swi_handler, "function"
    .size secure_swi_handler, . - secure_undef_handler
