usbsrcs = $(USBDEVPATH)/App/usb_device.c \
$(USBDEVPATH)/App/usbd_cdc_if.c \
$(USBDEVPATH)/App/usbd_desc.c \
$(USBDEVPATH)/Target/usbd_conf.c

C_SOURCES += $(usbsrcs)
C_INCLUDE_DIRS +=  \
$(USBDEVPATH)/App \
$(USBDEVPATH)/Target