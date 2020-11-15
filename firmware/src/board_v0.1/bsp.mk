
C_SOURCES += \
${BOARD_BSP}/gpio.c \
${BOARD_BSP}/spi.c \
${BOARD_BSP}/tim.c \
${BOARD_BSP}/adc.c \
${BOARD_BSP}/dma.c \
${BOARD_BSP}/i2c.c \
${BOARD_BSP}/usart.c \
${BOARD_BSP}/board.c

BOARD_INCLUDE_DIRS = include/${BOARD_MODEL}

