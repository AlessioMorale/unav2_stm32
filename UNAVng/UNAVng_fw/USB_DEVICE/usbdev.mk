usbsrcs = $(USBDEVPATH)/App/usb_device.c \
$(USBDEVPATH)/App/usbd_cdc_if.c \
$(USBDEVPATH)/App/usbd_desc.c \
$(USBDEVPATH)/Target/usbd_conf.c

C_SOURCES += $(usbsrcs)
C_INCLUDES +=  \
-I$(USBDEVPATH)/App \
-I$(USBDEVPATH)/Target