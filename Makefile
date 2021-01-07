OBJECTS= auto_sail_trim.o
DEVICE  = msp430g2553
INSTALL_DIR=$(HOME)/ti/msp430_gcc

GCC_DIR =  $(INSTALL_DIR)/bin
SUPPORT_FILE_DIRECTORY = $(INSTALL_DIR)/include

CC      = $(GCC_DIR)/msp430-elf-gcc
GDB     = $(GCC_DIR)/msp430-elf-gdb

CFLAGS = -I $(SUPPORT_FILE_DIRECTORY) -mmcu=$(DEVICE) -Os -g
LFLAGS = -L $(SUPPORT_FILE_DIRECTORY) -T $(DEVICE).ld

all: ${OBJECTS}
	$(CC) $(CFLAGS) $(LFLAGS) $? -o auto_sail_trim.elf

debug: all
	$(GDB) auto_sail_trim.elf
