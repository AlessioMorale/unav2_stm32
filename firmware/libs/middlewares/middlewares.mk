USBDEVLIBRARYPATH = $(MIDDLEWARESPATH)/ST/STM32_USB_Device_Library
usbdevlibsrc = $(USBDEVLIBRARYPATH)/Core/Src/usbd_core.c \
$(USBDEVLIBRARYPATH)/Core/Src/usbd_ctlreq.c \
$(USBDEVLIBRARYPATH)/Core/Src/usbd_ioreq.c \
$(USBDEVLIBRARYPATH)/Class/CDC/Src/usbd_cdc.c 

C_SOURCES += \
$(usbdevlibsrc)

C_INCLUDE_DIRS += \
$(USBDEVLIBRARYPATH)/Core/Inc \
$(USBDEVLIBRARYPATH)/Class/CDC/Inc

FREERTOSLIBRARYPATH = $(MIDDLEWARESPATH)/Third_Party/FreeRTOS

C_SOURCES += \
$(FREERTOSLIBRARYPATH)/Source/croutine.c \
$(FREERTOSLIBRARYPATH)/Source/event_groups.c \
$(FREERTOSLIBRARYPATH)/Source/list.c \
$(FREERTOSLIBRARYPATH)/Source/queue.c \
$(FREERTOSLIBRARYPATH)/Source/stream_buffer.c \
$(FREERTOSLIBRARYPATH)/Source/tasks.c \
$(FREERTOSLIBRARYPATH)/Source/timers.c \
$(FREERTOSLIBRARYPATH)/Source/CMSIS_RTOS/cmsis_os.c \
$(FREERTOSLIBRARYPATH)/Source/portable/MemMang/heap_4.c \
$(FREERTOSLIBRARYPATH)/Source/portable/GCC/ARM_CM4F/port.c

C_INCLUDE_DIRS += \
$(FREERTOSLIBRARYPATH)/Source/CMSIS_RTOS/ \
$(FREERTOSLIBRARYPATH)/Source/include \
$(FREERTOSLIBRARYPATH)/Source/portable/GCC/ARM_CM4F