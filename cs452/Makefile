FILENAME=a0
XDIR:=/u/cs452/public/xdev
TRIPLE=aarch64-none-elf
XBINDIR:=$(XDIR)/bin
CC:=$(XBINDIR)/$(TRIPLE)-gcc -ffreestanding
OBJCOPY:=$(XBINDIR)/$(TRIPLE)-objcopy
OBJDUMP:=$(XBINDIR)/$(TRIPLE)-objdump

MMU?=on
OPT?=-O3

# COMPILE OPTIONS
ifeq ($(MMU),on)
MMUFLAGS:=-DMMU
else
MMUFLAGS:=-mstrict-align -mgeneral-regs-only
endif
WARNINGS:=-Wall -Wextra -Wpedantic -Wno-unused-const-variable
CFLAGS:=-g -pipe -static -march=armv8-a -mcpu=cortex-a72 $(OPT) $(MMUFLAGS) $(WARNINGS)

# -Wl,option tells gcc to pass 'option' to the linker with commas replaced by spaces
# doing this rather than calling the linker directly simplifies the compilation procedure
LDFLAGS:=-Wl,-nmagic -Wl,-Tlinker.ld -Wl,--no-warn-rwx-segments -nostartfiles

# Source files and include dirs
SOURCES := $(wildcard *.S) $(wildcard *.c)
# Create .o and .d files for every .c and .S (hand-written assembly) file
OBJECTS := $(patsubst %.S, %.o, $(patsubst %.c, %.o, $(SOURCES)))
DEPENDS := $(patsubst %.S, %.d, $(patsubst %.c, %.d, $(SOURCES)))

.PHONY: all clean

all: $(FILENAME).img

clean:
	rm -f $(OBJECTS) $(DEPENDS) $(FILENAME).elf $(FILENAME).img

$(FILENAME).img: $(FILENAME).elf
	$(OBJCOPY) -S -O binary $< $@; sync

$(FILENAME).elf: $(OBJECTS) linker.ld
	$(CC) $(CFLAGS) $(filter-out %.ld, $^) -o $@ $(LDFLAGS)
ifneq ($(MMU),on)
	@$(OBJDUMP) -d $@ | grep -Fq q0 && printf "\n***** WARNING: SIMD DETECTED! *****\n\n" || true
endif

%.o: %.S Makefile
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

%.o: %.c Makefile
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

-include $(DEPENDS)
