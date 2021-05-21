BOARD ?= PotatoPi_PICo24

# If the build directory is not given, make it reflect the board name.
BUILD ?= build-$(BOARD)

BOARD_DIR ?= boards/$(BOARD)
ifeq ($(wildcard $(BOARD_DIR)/.),)
$(error Invalid BOARD specified: $(BOARD_DIR))
endif


include ../../py/mkenv.mk
include $(BOARD_DIR)/mpconfigboard.mk

# qstr definitions (must come before including py.mk)
QSTR_DEFS = qstrdefsport.h

# include py core make definitions
include $(TOP)/py/py.mk

XCVERSION ?= 1.70
XC16 ?= /opt/microchip/xc16/v$(XCVERSION)
CROSS_COMPILE ?= $(XC16)/bin/xc16-

INC += -I.
INC += -I$(BOARD_DIR)
INC += -I$(TOP)
INC += -I$(BUILD)
INC += -I$(XC16)/include
INC += -I$(XC16)/support/$(PARTFAMILY)/h

CFLAGS_PIC16BIT = -mcpu=$(PART) -mlarge-code
CFLAGS = $(INC) -Wall -Werror -std=gnu99 -nostdlib $(CFLAGS_PIC16BIT) $(COPT)

#Debugging/Optimization
ifeq ($(DEBUG), 1)
CFLAGS += -O0 -ggdb
else
CFLAGS += -O2 -DNDEBUG
endif

LDFLAGS = --heap=0 -nostdlib --script=$(BOARD_DIR)/linker_script.ld -Map=$@.map --cref -p$(PART) --check-sections --data-init --pack-data --handles --isr --gc-sections --fill-upper=0 --stackguard=16 --no-force-link --smart-io --report-mem
LIBS = -L$(XC16)/lib -L$(XC16)/lib/$(PARTFAMILY) -lc -lm -lpic30

SRC_C += \
	main.c \
	pic16bit_mphal.c \
	modpyb.c \
	modpybled.c \
	modpybswitch.c \
	lib/utils/pyexec.c \
	lib/utils/sys_stdio_mphal.c \
	lib/mp-readline/readline.c \
	$(wildcard $(BOARD_DIR)/*.c) \

SRC_S = \
#	gchelper.s \

OBJ = $(PY_O) $(addprefix $(BUILD)/, $(SRC_C:.c=.o) $(SRC_S:.s=.o))

# List of sources for qstr extraction
SRC_QSTR += $(SRC_C)
# Append any auto-generated sources that are needed by sources listed in
# SRC_QSTR
SRC_QSTR_AUTO_DEPS +=

all: $(BUILD)/firmware.hex

$(BUILD)/firmware.hex: $(BUILD)/firmware.elf
	$(ECHO) "Create $@"
	$(Q)$(CROSS_COMPILE)bin2hex $<

$(BUILD)/firmware.elf: $(OBJ)
	$(ECHO) "LINK $@"
	$(Q)$(LD) $(LDFLAGS) -o $@ $^ $(LIBS)
	$(Q)size $@

$(PY_BUILD)/gc.o: CFLAGS += -O1
$(PY_BUILD)/vm.o: CFLAGS += -O1

include $(TOP)/py/mkrules.mk
