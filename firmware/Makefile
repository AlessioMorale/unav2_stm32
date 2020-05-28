######################################
# target
######################################
TARGET = unavng_fw

######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og


#######################################
# paths
#######################################
# Build path
BUILD_DIR = build

HALPATH = libs/drivers/STM32F4xx_HAL_Driver
CMSIS = libs/drivers/CMSIS
USBDEVPATH = src/usb_device
MIDDLEWARESPATH = libs/middlewares
ROSSERIAL = libs/rosserial

BOARD_MODEL ?= board_v0.2

BOARD_BSP = src/${BOARD_MODEL}

$(info Building for BOARD_MODEL [${BOARD_MODEL}])
$(info )
######################################
# source
######################################
# C sources
C_SOURCES =  \
src/crc.c \
src/main.c \
src/rng.c \
src/stm32f4xx_it.c \
src/stm32f4xx_hal_msp.c \
src/system_stm32f4xx.c \
src/stm32f4xx_hal_timebase_tim.c \
src/timing.c \
src/mathutils.c \
src/leds.c \
src/leds_hal.c \
src/instrumentation/instrumentation.c \
src/counters.c \
src/freertos_support.c

CPP_SOURCES = \
src/freertos.cpp \
src/modules/basemodule.cpp \
src/modules/baserosmodule.cpp \
src/modules/rosnodemodule.cpp \
src/modules/motormanagermodule.cpp \
src/modules/motorcontrollermodule.cpp \
src/modules/systemmodule.cpp \
src/controls/pid.cpp \
src/drivers/encoder.cpp \
src/driver/lm75.cpp \
src/driver/ina219.cpp \
src/utils/timer.cpp \
src/messaging.cpp


# ASM sources
ASM_SOURCES =  \
startup_stm32f405xx.s

include $(HALPATH)/stm32hal.mk
include $(USBDEVPATH)/usbdev.mk
include $(MIDDLEWARESPATH)/middlewares.mk
include $(ROSSERIAL)/rosserial.mk
include ${BOARD_BSP}/bsp.mk
#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
ARMCC = $(GCC_PATH)/$(PREFIX)gcc
ARMCPPC = $(GCC_PATH)/$(PREFIX)g++
ARMAS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
ARMCP = $(GCC_PATH)/$(PREFIX)objcopy
ARMSZ = $(GCC_PATH)/$(PREFIX)size
else
ARMCC = $(PREFIX)gcc
ARMCPPC = $(PREFIX)g++
ARMAS = $(PREFIX)gcc -x assembler-with-cpp
ARMCP = $(PREFIX)objcopy
ARMSZ = $(PREFIX)size
endif
ARMHEX = $(ARMCP) -O ihex
ARMBIN = $(ARMCP) -O binary -S
 
#######################################
# ARM_CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI) -fsingle-precision-constant

# macros for gcc
# ARMAS defines
ARM_AS_DEFS = 

# C defines
ARM_C_DEFS =  \
-DUSE_HAL_DRIVER \
-DSTM32F407xx


# ARMAS includes
AS_INCLUDES = 

# C includes
C_INCLUDE_DIRS +=  \
include \
$(CMSIS)/Device/ST/STM32F4xx/Include \
$(CMSIS)/Include \
$(BOARD_INCLUDE_DIRS)

ARM_CFLAGS          += -Wall
ARM_CFLAGS 					+= -Wextra
ARM_CFLAGS 					+= -Wfatal-errors
ARM_CFLAGS 					+= -Wpacked
ARM_CFLAGS 					+= -Winline
ARM_CFLAGS 					+= -Wfloat-equal
ARM_CFLAGS 					+= -Wconversion
ARM_CFLAGS 					+= -Wlogical-op
ARM_CFLAGS 					+= -Wpointer-arith
ARM_CFLAGS 					+= -Wdisabled-optimization
ARM_CFLAGS          += -Wno-unused-parameter
ARM_CFLAGS          += -Wa,-alh=$(@:.o=.lst)

C_INCLUDE_DIRS_EXPANDED = $(wildcard $(C_INCLUDE_DIRS))
C_INCLUDES += $(foreach dir, $(C_INCLUDE_DIRS_EXPANDED), -I$(dir))
AS_INCLUDES_DIRS_EXPANDED = $(wildcard $(AS_INCLUDES_DIRS))
AS_INCLUDES += $(foreach dir, $(AS_INCLUDES_DIRS_EXPANDED), -I$(dir))

# compile gcc flags
ARM_ASFLAGS = $(MCU) $(ARM_AS_DEFS) $(AS_INCLUDES) $(OPT) -fdata-sections -ffunction-sections -fno-common

ARM_CFLAGS = $(MCU) $(ARM_C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections -fno-common -fstack-usage



ifeq ($(DEBUG), 1)
ARM_CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
ARM_CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)" -Wall 
ARM_CONLYFLAGS += -std=c11
# C++ Compiler -- Required & Optimization Flags

ARM_CXXFLAGS                += $(ARM_CFLAGS)
ARM_CXXFLAGS 				+= -fabi-version=0
ARM_CXXFLAGS                += -fno-rtti
ARM_CXXFLAGS                += -fno-exceptions
ARM_CXXFLAGS				+= -fno-use-cxa-atexit
ARM_CXXFLAGS 				+= -fno-threadsafe-statics
ARM_CXXFLAGS                += -std=c++17

# C++ -- Warnings
ARM_CXXFLAGS 				+= -Weffc++
ARM_CXXFLAGS 				+= -Wfloat-equal
ARM_CXXFLAGS 				+= -Wsign-promo
ARM_CXXFLAGS 				+= -Wzero-as-null-pointer-constant
ARM_CXXFLAGS 				+= -Wmissing-declarations 
ARM_CXXFLAGS 				+= -Woverloaded-virtual
ARM_CXXFLAGS 				+= -Wsuggest-final-types
ARM_CXXFLAGS 				+= -Wsuggest-final-methods
#ARM_CXXFLAGS 				+= -Wsuggest-override
ARM_CXXFLAGS 				+= -Wsuggest-attribute=const
ARM_CXXFLAGS 				+= -Wsuggest-attribute=noreturn
ARM_CXXFLAGS 				+= -Wsuggest-attribute=format
ARM_CXXFLAGS 				+= -Wmissing-format-attribute
#ARM_CXXFLAGS 				+= -Wold-style-cast
ARM_CXXFLAGS 				+= -Wshadow
#ARM_CXXFLAGS 				+= -Wuseless-cast
ARM_CXXFLAGS 				+= -Wctor-dtor-privacy
ARM_CXXFLAGS 				+= -Wstrict-null-sentinel

ARM_CFLAGS += $(ARM_CONLYFLAGS)
#######################################
# LDFLAGS
#######################################
# link script
ARM_LDSCRIPT = unav_firmware.ld

# libraries
ARM_LIBS = -lc -lm -lnosys -lstdc++
ARM_LIBDIR = 
ARM_LDFLAGS = $(MCU) -specs=nano.specs -T$(ARM_LDSCRIPT) $(ARM_LIBDIR) $(ARM_LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections -Wl,--undefined=uxTopUsedPriority

#######################################
# CppUTest
#######################################
#--- Inputs ----#
COMPONENT_NAME = uNav_CppUTest
CPPUTEST_HOME = ./testing/cpputest/
CPPUTEST_USE_MEM_LEAK_DETECTION = N
CPPUTEST_CXXFLAGS   += -std=c++17
C_DEFS +=  \
-DUSE_HAL_DRIVER \
-DSTM32F407xx

CPPUTEST_CXXFLAGS += $(C_DEFS) 

CPPUTEST_USE_EXTENSIONS = Y
CPP_PLATFORM = Gcc

SRC_DIRS = \
	src/CppUTest \
	src/Platforms/$(CPP_PLATFORM) 

SRC_FILES = \
	src/modules/system/systemhealthchecker.cpp \
	src/drivers/ina219.cpp \


TEST_SRC_DIRS = \
	testing/src \
	testing/src/mocks

INCLUDE_DIRS = \
	. \
	testing/include \
	$(CPPUTEST_HOME)/include
	
INCLUDE_DIRS +=	$(C_INCLUDE_DIRS) 


$(CPPUTEST_HOME)/lib/libCppUTest.a $(CPPUTEST_HOME)/lib/libCppUTestExt.a:
	@echo Building CppUTest library
	cd $(CPPUTEST_HOME) && ./autogen.sh
	cd $(CPPUTEST_HOME) && ./configure > /dev/null
	$(MAKE) -C $(CPPUTEST_HOME) -s clean
	$(MAKE) -C $(CPPUTEST_HOME) -s -j4
	$(MAKE) -C $(CPPUTEST_HOME) -s -j4 check
	@echo CppUTest library built

cpputest: $(CPPUTEST_HOME)/lib/libCppUTest.a

include $(CPPUTEST_HOME)/build/MakefileWorker.mk


# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin vtest

#######################################
# precompiled headers
#######################################
PRECOMPILED_H := precompiled.h
PRECOMPILED_H_OBJ := $(BUILD_DIR)/$(PRECOMPILED_H).gch

$(BUILD_DIR)/%.h.gch: $(PRECOMPILED_H) | $(BUILD_DIR)
	$(ARMCC) -c $(ARM_CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

PRECOMPILED_CPP_H := precompiledcpp.h
PRECOMPILED_CPP_H_OBJ := $(BUILD_DIR)/$(PRECOMPILED_CPP_H).gch

$(BUILD_DIR)/%cpp.h.gch: $(PRECOMPILED_CPP_H) | $(BUILD_DIR)
	$(ARMCPPC) -c $(ARM_CXXFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.lst)) $< -o $@

#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of cpp objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(CPP_SOURCES:.cpp=.o)))
vpath %.cpp $(sort $(dir $(CPP_SOURCES)))

# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(PRECOMPILED_H_OBJ) $(PRECOMPILED_CPP_H_OBJ)
	$(ARMCC) -c $(ARM_CFLAGS) -include $(PRECOMPILED_H)  -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.cpp Makefile | $(BUILD_DIR) 
	$(ARMCPPC) -c $(ARM_CXXFLAGS) -include $(PRECOMPILED_CPP_H) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(ARMAS) -c $(ARM_CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(ARMCC) $(OBJECTS) $(ARM_LDFLAGS) -o $@
	$(ARMSZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(ARMHEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(ARMBIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)
  
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

.PHONY: flash
flash:  $(BUILD_DIR)/$(TARGET).elf
	openocd -f openocd.cfg -c "program  $(BUILD_DIR)/$(TARGET).elf verify reset" -c "shutdown"
# *** EOF ***
.PHONY: format
format: 
	@echo formatting source code
	./format ./core
	./format ./testing/include
	./format ./testing/src
	@echo -
