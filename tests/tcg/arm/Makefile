-include ../../config-host.mak

SRC_PATH ?= ../../..
BIN_PATH ?= ../../..

CROSS=arm-none-eabi-

SIM      = $(BIN_PATH)/arm-softmmu/qemu-system-arm
SIMFLAGS = -M vexpress-a9 -nographic -semihosting $(EXTFLAGS) -kernel
SIMDEBUG = -s -S

CC      = $(CROSS)gcc
AS      = $(CROSS)gcc -x assembler
LD      = $(CROSS)ld


CFLAGS    = -march=armv7-a -mcpu=cortex-a9 -nodefaultlibs -ffreestanding -DUSE_SEMIHOSTING
ASFLAGS   = -march=armv7-a -mcpu=cortex-a9 -nodefaultlibs -ffreestanding -DUSE_SEMIHOSTING
LDFLAGS   = -nodefaultlibs -nostarfiles
LDFLAGS  += -Tlinker.ld

CRT     = crt_secure.o

TESTCASES += scr_read_secure.tst


all: build

%.o: $(SRC_PATH)/tests/arm/%.c
	$(CC) $(CFLAGS) -c $< -o $@

%.o: $(SRC_PATH)/tests/arm/%.S
	$(AS) $(ASFLAGS) -c $< -o $@

%.tst: %.o $(CRT) Makefile
	$(LD) $(LDFLAGS) $(NOSTDFLAGS) $(CRT) $< -o $@

build: $(TESTCASES)

check: $(addprefix run-, $(TESTCASES))

run-%.tst: %.tst
	$(SIM) $(SIMFLAGS) ./$<

run-test_fail.tst: test_fail.tst
	! $(SIM) $(SIMFLAGS) ./$<

debug-%.tst: %.tst
	$(SIM) $(SIMDEBUG) $(SIMFLAGS) ./$<

host-debug-%.tst: %.tst
	gdb --args $(SIM) $(SIMFLAGS) ./$<

clean:
	$(RM) -fr $(TESTCASES) $(CRT)
