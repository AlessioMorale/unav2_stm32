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


