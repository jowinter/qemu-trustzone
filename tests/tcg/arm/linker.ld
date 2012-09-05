/**
 * Linker map for QEMU TrustZone test setup with Versatile Express Cortex-A9
 * board setup.
 *
 * NOTE: We use object file names to handle secure and normal world association
 * in a simple manner:
 *
 * Object files with a name ending in "-secure.o" are assigned to secure world,
 * all other files go to normal world.
 */
OUTPUT_FORMAT("elf32-littlearm")
OUTPUT_ARCH("arm")
ENTRY(secure_exception_vectors)

MEMORY {
  /**
   * Our simple TrustZone tests partition the VE_SRAM
   * found at 0x48000000-0x49FFFFFF into a secure world
   * and a normal world partition.
   */
  secure.rom : ORIGIN = 0x48000000, LENGTH = 8M
  secure.ram : ORIGIN = 0x48800000, LENGTH = 8M
  normal.rom : ORIGIN = 0x49000000, LENGTH = 8M
  normal.ram : ORIGIN = 0x49800000, LENGTH = 8M
}

SECTIONS {
  /*
   * Secure-world code and data-segments
   */
  .secure.vectors : {
    crt_secure.o (.vectors);
  } >secure.rom

  .secure.text : {
    *_secure.o (.text .text.*);
  } >secure.rom

  .secure.rodata : {
    *_secure.o (.rodata .rodata.*);
    __secure_fixups_begin = .;
    *_secure.o (.fixups);
    __secure_fixups_end = .;
  } >secure.rom

  .secure.extab : {
    __secure_extab_start = .;
    *_secure.o (.ARM.extab*);
    __secure_extab_end = .;
   } >secure.rom

  .secure.exidx : {
    __secure_exidx_start = .;
    *_secure.o (.ARM.exidx*);
    __secure_exidx_end = .;
  } >secure.rom

  .secure.data : {
    *_secure.o (.data .data.*);
  } >secure.ram

  .secure.bss : {
    *_secure.o (.bss .bss.*);
  } >secure.ram

  /*
   * Normal-world code and data-segments
   */
  .normal.text : {
    *(.text .text.*);
  } >normal.rom

  .normal.rodata : {
    *(.rodata .rodata.*);
  } >normal.rom

  .normal.extab : {
    __normal_extab_start = .;
    *(.ARM.extab*);
    __normal_extab_end = .;
  } >normal.rom

  .normal.exidx : {
    __normal_exidx_start = .;
    *(.ARM.exidx*);
    __normal_exidx_end = .;
  } >normal.rom

  .normal.data : {
    *(.data .data.*);
  } >normal.ram

  .normal.bss : {
    *(.bss .bss.*);
  } >normal.ram
}
