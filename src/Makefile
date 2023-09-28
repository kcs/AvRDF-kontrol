#####################################################################
#  Makefile for AVR
#  Kertész Csaba-Zoltán
#  csaba.kertesz@etc.unitbv.ro
#####################################################################

# avoid displayin commands
export MAKEFLAGS += --silent

#--------------------------------------------------------------------
#  Project specific settings
#--------------------------------------------------------------------

# project name
TARGET = AvRDF-kontrol

# mcu type
MCU = attiny461a

# cpu frequency
F_CPU = 4000000

# programmer type
ISP = usbasp

# programmer port
PORT = 

# fuse bits
LFUSE = 0xFE
HFUSE = 0xDF
EFUSE = 0xFF

# board variant
BOARD_VARIANT = 1


#--------------------------------------------------------------------
#  Directories
#--------------------------------------------------------------------
OBJDIR = obj
BINDIR = bin
LSTDIR = lst
SRCDIR = .

# insert source directories into the search path
vpath %c  $(SRCDIR) \


#--------------------------------------------------------------------
#  Source files
#--------------------------------------------------------------------

# C sources (*.c)
SRC = main.c

# assembly sources (*.S)
ASRC = 


#--------------------------------------------------------------------
#  Compiler settings
#--------------------------------------------------------------------

# optimization level
OPT = 2

# compile time definitions
CDEFS = BOARD_VERSION=$(BOARD_VARIANT) \

# include directories
CINC = \
       $(SRCDIR) \

# warnings
CWARN = -Wall -Wextra -Werror\
        -Wshadow -Wpointer-arith -Wbad-function-cast -Wcast-align \
        -Wsign-compare -Waggregate-return -Wstrict-prototypes \
        -Wmissing-prototypes -Wmissing-declarations -Wunused \

# tuning
CTUNING = -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums

# avr-gcc version 13 has bug, that identifies PORTB as an out-of-bounds array
# access, until this is fixed, a workaround is to specify the page size as 0
CTUNING += --param min-pagesize=0

# language standard
# use latest GNU extensions
CSTD = gnu17

# debug info
DEBUG = dwarf-2

#--------------------------------------------------------------------
#  Assembler settings
#--------------------------------------------------------------------

# include directories
AINC = 

#--------------------------------------------------------------------
#  Linker settings
#--------------------------------------------------------------------

# libraries
LIBS = 

#--------------------------------------------------------------------
#  avrdude settings
#--------------------------------------------------------------------

# other flags
AVRDUDE_FLAGS = 

#====================================================================

# commands
SHELL = sh
CC = avr-gcc
LD = avr-gcc
OBJCOPY = avr-objcopy
OBJDUMP = avr-objdump
SIZE = avr-size
NM = avr-nm
AVRDUDE = avrdude
REMOVE = rm -f
REMOVEDIR = rm -rf
MKDIR = mkdir -p
MOVE = mv -f

# object files
OBJ = $(SRC:%.c=$(OBJDIR)/%.o) $(ASRC:%.S=$(OBJDIR)/%.o)

# listing files
LST = $(SRC:%.c=$(LSTDIR)/%.lst) $(ASRC:%.S=$(LSTDIR)/%.lst)

# some systemwide extra flags
DEPFLAGS = -MD -MP -MF $(OBJDIR)/.dep/$(@F).d
# listings
CLST = -Wa,-adhlns=$(LSTDIR)/$(notdir $(<:.c=.lst))
ALST = -Wa,-adhlns=$(LSTDIR)/$(notdir $(<:.S=.lst))
LMAP = -Wl,-Map=$(LSTDIR)/$(TARGET).map,--cref

#compiler flags
CFLAGS = \
         -mmcu=$(MCU) \
         -g$(DEBUG) \
         $(addprefix -D,$(CDEFS)) -DF_CPU=$(F_CPU) \
         -O$(OPT) \
         $(CWARN) \
         $(CTUNING) \
         -std=$(CSTD) \
         $(addprefix -I,$(CINC)) \
         $(CLST) \
         $(DEPFLAGS)

AFLAGS = -mmcu=$(MCU) -x assembler-with-cpp $(AFLAGS) $(AINC) $(ALST)

LDFLAGS = -mmcu=$(MCU) $(LMAP) $(LIBS)

#install target: program flash using avrdude
AVRDUDE_FLASH = -U flash:w:$(BINDIR)/$(TARGET).hex
#AVRDUDE_FLASH = -U application:w:$(BINDIR)/$(TARGET).hex
#AVRDUDE_EEPROM = -U eeprom:w:$(BINDIR)/$(TARGET).eep

#====================================================================

#default target
all:  directories elf hex eep lss size

elf: $(BINDIR)/$(TARGET).elf
hex: $(BINDIR)/$(TARGET).hex
eep: $(BINDIR)/$(TARGET).eep
lss: $(LSTDIR)/$(TARGET).lss

directories:
	$(MKDIR) $(sort $(BINDIR) $(OBJDIR) $(LSTDIR))

size:
	@if test -f $(BINDIR)/$(TARGET).elf; \
	then \
		echo ; $(SIZE) $(BINDIR)/$(TARGET).elf; \
	fi

install:
	$(AVRDUDE) -p t461 -c $(ISP) $(addprefix -P ,$(PORT)) $(AVRDUDE_COUNTER) $(AVRDUDE_FLAGS) \
		 $(AVRDUDE_FLASH) \
		 $(AVRDUDE_EEPROM)

fuses:
	$(AVRDUDE) -p t461 -c $(ISP) $(addprefix -P ,$(PORT)) $(AVRDUDE_COUNTER) -u \
		-U lfuse:w:$(LFUSE):m \
		-U hfuse:w:$(HFUSE):m \
		-U efuse:w:$(EFUSE):m

$(BINDIR)/$(TARGET).hex: $(BINDIR)/$(TARGET).elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

$(BINDIR)/$(TARGET).eep: $(BINDIR)/$(TARGET).elf
	$(OBJCOPY) -j eeprom --set-section-flags=.eeprom="alloc,load" \
		--change-section-lma .eeprom=0 --no-change-warnings -O ihex $< $@

$(LSTDIR)/$(TARGET).lss: $(BINDIR)/$(TARGET).elf
	$(OBJDUMP) -h -S $< > $@

$(BINDIR)/$(TARGET).elf: $(OBJ)
	$(LD) $(LDFLAGS) $(OBJ) $(LIBS) -o $@

$(OBJDIR)/%.o: %.c
	echo "(CC) $<"
	$(CC) -c $(CFLAGS) $< -o $@

$(OBJDIR)/%.o: %.S
	echo "(AS) $<"
	$(CC) -S $(AFLAGS) $< -o $@

clean:
	$(REMOVE) $(BINDIR)/* $(OBJDIR)/* $(LSTDIR)/*
	$(REMOVEDIR) $(OBJDIR)/.dep

#include dependecies
-include $(shell $(MKDIR) $(OBJDIR)/.dep 2>/dev/null) $(wildcard $(OBJDIR)/.dep/*)

.PHONY : all directories elf hex eep lss install fuses clean
