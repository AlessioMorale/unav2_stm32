freerstossrcs = $(wildcard $(FREERTOSPATH)/src/*.c)

C_SOURCES += $(freerstossrcs)
C_INCLUDES += -I$(FREERTOSPATH)/src