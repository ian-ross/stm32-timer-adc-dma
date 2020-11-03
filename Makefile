TARGET ?= simple-adc

SRCS += \
  $(TARGET).cpp \
  adc.cpp \
  gpio.cpp \
  mcu.cpp \
  usart.cpp


# TOOL PATHS

GCC_INSTALL_ROOT ?= /opt/gcc-arm-none-eabi/bin/
GCC_PREFIX ?= arm-none-eabi
BSP=./bsp


# CMSIS INCLUDE DIRECTORIES

INCS += \
  $(BSP)/include \
  $(BSP)/CMSIS/Core/include \
  $(BSP)/CMSIS/Device/ST/STM32F7xx/include


# CMSIS STARTUP FILES

SRCS += \
  $(BSP)/system_stm32f7xx.c \
  $(BSP)/startup_stm32f767xx.s

# LINKER PLACEMENT

LINKER_SCRIPT := $(BSP)/STM32F767ZITx_FLASH.ld


# FLAGS AND COMPILER SETUP

# Optimisation
OPT = -O3 -g3
#OPT = -g

# Flags shared between tools.
COMMON_FLAGS = -DSTM32F767xx -DHSE_VALUE=8000000
COMMON_FLAGS += -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
COMMON_FLAGS += -Wall -Werror $(OPT)

# Flags shared between C and C++.
CCOMMON_FLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CCOMMON_FLAGS += -fno-builtin -fshort-enums
INC_FLAGS := $(addprefix -I,$(INCS))

# C flags common to all targets
CFLAGS += $(OPT) $(COMMON_FLAGS) $(CCOMMON_FLAGS) $(INC_FLAGS)

# C++ flags common to all targets
CXXFLAGS += $(OPT) $(COMMON_FLAGS) $(CCOMMON_FLAGS) $(INC_FLAGS)
CXXFLAGS += -fno-exceptions -fno-rtti

# Assembler flags common to all targets
ASMFLAGS += $(COMMON_FLAGS) -g3

# Linker flags
LDFLAGS += -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
LDFLAGS += -specs=nano.specs -specs=nosys.specs -lc -lm -lnosys
LDFLAGS += -Wl,--gc-sections


# TOOLS

MK := mkdir
RM := rm -rf
CC      := $(GCC_INSTALL_ROOT)$(GCC_PREFIX)-gcc
CXX     := $(GCC_INSTALL_ROOT)$(GCC_PREFIX)-c++
AS      := $(GCC_INSTALL_ROOT)$(GCC_PREFIX)-as
AR      := $(GCC_INSTALL_ROOT)$(GCC_PREFIX)-ar
LD      := $(GCC_INSTALL_ROOT)$(GCC_PREFIX)-ld
NM      := $(GCC_INSTALL_ROOT)$(GCC_PREFIX)-nm
OBJDUMP := $(GCC_INSTALL_ROOT)$(GCC_PREFIX)-objdump
OBJCOPY := $(GCC_INSTALL_ROOT)$(GCC_PREFIX)-objcopy
SIZE    := $(GCC_INSTALL_ROOT)$(GCC_PREFIX)-size


# ----------------------------------------------------------------------
#
#  MAIN BUILD GOALS
#

.PHONY: default

default: build/$(TARGET).out build/$(TARGET).hex build/$(TARGET).bin

# Per-target object files derived from source paths.
OBJS1 = $(patsubst %.s,%.o,$(patsubst %.cpp,%.o,$(patsubst %.c,%.o,$(SRCS))))
OBJS2 = $(patsubst ../../%,%,$(OBJS1))
OBJS = $(addprefix build/$(TARGET)/,$(OBJS2))

# Per-target dependency files derived from source paths.
DEPS1 = $(patsubst %.s,%.d,$(patsubst %.cpp,%.d,$(patsubst %.c,%.d,$(SRCS))))
DEPS = $(addprefix build/$(TARGET)/,$(DEPS1))


# Link rule.

build/$(TARGET).out: $(OBJS)
	$(info Linking target: $@)
	@echo $^ $(LIB_FILES) > $(@:.out=.in)
	@$(CC) $(LDFLAGS) @$(@:.out=.in) -Wl,-Map=$(@:.out=.map) -o $@
	@$(SIZE) $@


# Compilation rules.

build/$(TARGET)/%.o: %.c
	$(info Compiling file: $(notdir $<))
	@$(MK) -p $(dir $@)
	@$(CC) -std=c99 -MP -MMD -c -o $@ $< $(CFLAGS) $(INC_PATHS)

build/$(TARGET)/%.o: %.cpp
	$(info Compiling file: $(notdir $<))
	@$(MK) -p $(dir $@)
	@$(CXX) -MP -MMD -c -o $@ $< $(CXXFLAGS) $(INC_PATHS)

build/$(TARGET)/%.o: %.s
	$(info Assembling file: $(notdir $<))
	@$(MK) -p $(dir $@)
	@$(CC) -x assembler-with-cpp -MP -MMD -c -o $@ $< $(ASMFLAGS) $(INC_PATHS)

%.bin: %.out
	$(info Preparing: $(notdir $@))
	@$(OBJCOPY) -O binary $< $@

%.hex: %.out
	$(info Preparing: $(notdir $@))
	@$(OBJCOPY) -O ihex $< $@



# Utility rules.

.PHONY: clean
clean:
	$(RM) build version.h

.PHONY: show_inc show_flags
show_inc:
	@echo $(INCS)
show_flags:
	@echo $(CFLAGS)

cdb:
	$(MAKE) clean
	bear --use-cc $(CC) --use-c++ $(CXX) $(MAKE)

.PHONY: flash
flash:
	openocd -f openocd.cfg -c "program build/$(TARGET).out verify reset exit"

# Generated dependencies.

-include $(DEPS)
