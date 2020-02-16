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

HALPATH = drivers/STM32F4xx_HAL_Driver
USBDEVPATH = usb_device
MIDDLEWARESPATH = middlewares
FREERTOSPATH = FreeRTOS
######################################
# source
######################################
# C sources
C_SOURCES =  \
core/src/crc.c \
core/src/gpio.c \
core/src/main.c \
core/src/rng.c \
core/src/spi.c \
core/src/tim.c \
core/src/stm32f4xx_it.c \
core/src/stm32f4xx_hal_msp.c \
core/src/system_stm32f4xx.c \
core/src/stm32f4xx_hal_timebase_tim.c \
core/src/timing.c \
core/src/mathutils.c \
core/src/adc.c \
core/src/dma.c \
core/src/i2c.c \
core/src/usart.c \
core/src/leds.c \
core/src/leds_hal.c \
middlewares/Third_Party/FreeRTOS/Source/croutine.c \
middlewares/Third_Party/FreeRTOS/Source/event_groups.c \
middlewares/Third_Party/FreeRTOS/Source/list.c \
middlewares/Third_Party/FreeRTOS/Source/queue.c \
middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c \
middlewares/Third_Party/FreeRTOS/Source/tasks.c \
middlewares/Third_Party/FreeRTOS/Source/timers.c \
middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c \
middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c \
middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c \
core/src/instrumentation/instrumentation.c \
core/src/counters.c \
core/src/freertos_support.c


CPP_SOURCES = \
core/src/freertos.cpp \
rosserial/time.cpp \
core/src/modules/basemodule.cpp \
core/src/modules/baserosmodule.cpp \
core/src/modules/rosnodemodule.cpp \
core/src/modules/motormanagermodule.cpp \
core/src/modules/motorcontrollermodule.cpp \
core/src/modules/systemmodule.cpp \
core/src/controls/pid.cpp \
core/src/drivers/encoder.cpp \
core/src/driver/lm75.cpp \
core/src/utils/timer.cpp \
core/src/messaging.cpp



# ASM sources
ASM_SOURCES =  \
startup_stm32f405xx.s

include $(HALPATH)/stm32hal.mk
include $(USBDEVPATH)/usbdev.mk
include $(MIDDLEWARESPATH)/middlewares.mk
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
core/include \
drivers/CMSIS/Device/ST/STM32F4xx/Include \
drivers/CMSIS/Include \
middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/ \
middlewares/Third_Party/FreeRTOS/Source/include \
middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F \
rosserial

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
ARM_CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)" -Wall -std=c11

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

CPPUTEST_CXXFLAGS   += -std=c++17

#######################################
# LDFLAGS
#######################################
# link script
ARM_LDSCRIPT = unav_firmware.ld

# libraries
ARM_LIBS = -lc -lm -lnosys 
ARM_LIBDIR = 
ARM_LDFLAGS = $(MCU) -specs=nano.specs -T$(ARM_LDSCRIPT) $(ARM_LIBDIR) $(ARM_LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections -Wl,--undefined=uxTopUsedPriority

#######################################
# CppUTest
#######################################
#--- Inputs ----#
COMPONENT_NAME = uNav_CppUTest
CPPUTEST_HOME = ./testing/cpputest/

CPPUTEST_USE_EXTENSIONS = Y
CPP_PLATFORM = Gcc

SRC_DIRS = \
	src/CppUTest \
	src/Platforms/$(CPP_PLATFORM)

SRC_FILES = \
	core/src/modules/system/systemhealthchecker.cpp \


TEST_SRC_DIRS = \
	testing/src

INCLUDE_DIRS =\
	.\
	$(CPPUTEST_HOME)/include\
	core/include \
	drivers/CMSIS/makeDevice/ST/STM32F4xx/Include \
	drivers/CMSIS/Include \
	rosserial

$(CPPUTEST_HOME)/lib/libCppUTest.a $(CPPUTEST_HOME)/lib/libCppUTestExt.a:
	@echo Building CppUTest library
	cd $(CPPUTEST_HOME) && ./autogen.sh
	cd $(CPPUTEST_HOME) && ./configure > /dev/null
	$(MAKE) -C $(CPPUTEST_HOME) -s
	$(MAKE) -C $(CPPUTEST_HOME) -s check
	@echo CppUTest library built
	@echo -

cpputest: $(CPPUTEST_HOME)/lib/libCppUTest.a

include $(CPPUTEST_HOME)/build/MakefileWorker.mk


# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin vtest

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

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(ARMCC) -c $(ARM_CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.cpp Makefile | $(BUILD_DIR) 
	$(ARMCPPC) -c $(ARM_CXXFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.lst)) $< -o $@

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

