NANOPBLIBRARYPATH = $(LIBSPATH)/nanopb

nanopblibsrc = \
$(NANOPBLIBRARYPATH)/pb_decode.c \
$(NANOPBLIBRARYPATH)/pb_encode.c \
$(NANOPBLIBRARYPATH)/pb_common.c

C_SOURCES += \
$(nanopblibsrc)

C_INCLUDE_DIRS += \
$(NANOPBLIBRARYPATH)
