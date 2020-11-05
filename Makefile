GCC_INSTALL_ROOT ?= /opt/gcc-arm-none-eabi/bin/

SRCS := common.c bsp/system_stm32f7xx.c bsp/startup_stm32f767xx.s
OBJS := $(addprefix build/,$(patsubst %.s,%.o,$(patsubst %.c,%.o,$(SRCS))))

INCS = \
  ./bsp/include \
  ./bsp/CMSIS/Core/include \
  ./bsp/CMSIS/Device/ST/STM32F7xx/include


all: build/ex1.out build/ex2.out build/ex3.out build/ex4.out # ex5



build/%.out: build/%.o $(OBJS)
	$(do-link)

ex1.o: common.h
ex2.o: common.h
ex3.o: common.h
ex4.o: common.h
common.o: common.h

.PHONY: flash-ex1 gdb-ex1

flash-ex1:
	openocd -f openocd.cfg -c "program build/ex1.out verify reset exit"

gdb-ex1:
	$(GDB) build/ex1.out



LINKER_SCRIPT := ./bsp/STM32F767ZITx_FLASH.ld

define do-link =
@echo $^ > $(@:.out=.in)
@$(CC) $(LDFLAGS) @$(@:.out=.in) -Wl,-Map=$(@:.out=.map) -o $@
@$(SIZE) $@
endef


OPT = -O3 -g3
#OPT = -g

COMMON_FLAGS = -DSTM32F767xx -DHSE_VALUE=8000000
COMMON_FLAGS += -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
COMMON_FLAGS += -Wall -Werror $(OPT)
CCOMMON_FLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CCOMMON_FLAGS += -fno-builtin -fshort-enums
INC_FLAGS := $(addprefix -I,$(INCS))

CFLAGS += $(OPT) $(COMMON_FLAGS) $(CCOMMON_FLAGS) $(INC_FLAGS)

ASMFLAGS += $(COMMON_FLAGS) -g3

LDFLAGS += -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
LDFLAGS += -specs=nano.specs -specs=nosys.specs -lc -lm -lnosys
LDFLAGS += -Wl,--gc-sections


GCC_PREFIX = arm-none-eabi
MK := mkdir
RM := rm -rf
CC      := $(GCC_INSTALL_ROOT)$(GCC_PREFIX)-gcc
SIZE    := $(GCC_INSTALL_ROOT)$(GCC_PREFIX)-size
GDB     := $(GCC_INSTALL_ROOT)$(GCC_PREFIX)-gdb


build/%.o: %.c
	$(info Compiling file: $(notdir $<))
	@$(MK) -p $(dir $@)
	@$(CC) -std=c99 -MP -MMD -c -o $@ $< $(CFLAGS) $(INC_PATHS)

build/%.o: %.s
	$(info Assembling file: $(notdir $<))
	@$(MK) -p $(dir $@)
	@$(CC) -x assembler-with-cpp -MP -MMD -c -o $@ $< $(ASMFLAGS) $(INC_PATHS)


.PHONY: clean
clean:
	$(RM) build
