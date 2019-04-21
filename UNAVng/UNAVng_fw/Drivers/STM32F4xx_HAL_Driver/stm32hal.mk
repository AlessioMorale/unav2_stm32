stm32libsrc = \
$(HALPATH)/Src/stm32f4xx_hal_pcd.c \
$(HALPATH)/Src/stm32f4xx_hal_pcd_ex.c \
$(HALPATH)/Src/stm32f4xx_ll_usb.c \
$(HALPATH)/Src/stm32f4xx_hal_i2c.c \
$(HALPATH)/Src/stm32f4xx_hal_i2c_ex.c \
$(HALPATH)/Src/stm32f4xx_hal_i2s.c \
$(HALPATH)/Src/stm32f4xx_hal_i2s_ex.c \
$(HALPATH)/Src/stm32f4xx_hal_spi.c \
$(HALPATH)/Src/stm32f4xx_hal_tim.c \
$(HALPATH)/Src/stm32f4xx_hal_tim_ex.c \
$(HALPATH)/Src/stm32f4xx_hal_rcc.c \
$(HALPATH)/Src/stm32f4xx_hal_rcc_ex.c \
$(HALPATH)/Src/stm32f4xx_hal_flash.c \
$(HALPATH)/Src/stm32f4xx_hal_flash_ex.c \
$(HALPATH)/Src/stm32f4xx_hal_flash_ramfunc.c \
$(HALPATH)/Src/stm32f4xx_hal_gpio.c \
$(HALPATH)/Src/stm32f4xx_hal_dma_ex.c \
$(HALPATH)/Src/stm32f4xx_hal_dma.c \
$(HALPATH)/Src/stm32f4xx_hal_pwr.c \
$(HALPATH)/Src/stm32f4xx_hal_pwr_ex.c \
$(HALPATH)/Src/stm32f4xx_hal_cortex.c \
$(HALPATH)/Src/stm32f4xx_hal.c \
$(HALPATH)/Src/stm32f4xx_hal_exti.c \
$(HALPATH)/Src/stm32f4xx_hal_adc.c \
$(HALPATH)/Src/stm32f4xx_hal_adc_ex.c \
$(HALPATH)/Src/stm32f4xx_hal_uart.c

C_SOURCES += $(stm32libsrc)

C_INCLUDES +=  \
-I$(HALPATH)/Inc \
-I$(HALPATH)/Inc/Legacy \
