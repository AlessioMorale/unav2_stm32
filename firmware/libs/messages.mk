MESSAGESPATH = $(LIBSPATH)/messages

messagessrc = $(MESSAGESPATH)\from_unav.pb.c \
$(MESSAGESPATH)\to_unav.pb.c

C_INCLUDE_DIRS += \
$(MESSAGESPATH)/
