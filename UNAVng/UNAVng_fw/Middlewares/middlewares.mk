USBDEVLIBRARYPATH = $(MIDDLEWARESPATH)/ST/STM32_USB_Device_Library
LITTLEFSLIBRARYPATH = $(MIDDLEWARESPATH)/littlefs
usbdevlibsrc = $(USBDEVLIBRARYPATH)/Core/Src/usbd_core.c \
$(USBDEVLIBRARYPATH)/Core/Src/usbd_ctlreq.c \
$(USBDEVLIBRARYPATH)/Core/Src/usbd_ioreq.c \
$(USBDEVLIBRARYPATH)/Class/CDC/Src/usbd_cdc.c 



littlefssrc = \
$(LITTLEFSLIBRARYPATH)/lfs.c \
$(LITTLEFSLIBRARYPATH)/lfs_util.c

C_SOURCES += \
$(usbdevlibsrc) \
$(littlefssrc) 

C_INCLUDES += -I$(USBDEVLIBRARYPATH)/Core/Inc \
-I$(USBDEVLIBRARYPATH)/Class/CDC/Inc \
-I$(LITTLEFSLIBRARYPATH)



