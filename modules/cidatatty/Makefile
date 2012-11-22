
#
# Set the driver file name.
#
TARGET= cidatatty.ko


#
#	You don't need to make change hereafter.
#
CC = $(CROSS_COMPILE)gcc
AR = $(CROSS_COMPILE)ar
LD = $(CROSS_COMPILE)ld
STRIP = $(CROSS_COMPILE)strip
M=$(shell pwd)


EXTRA_CFLAGS = $(COMMON_CFLAGS)
target := $(notdir $(TARGET))
obj-m  := $(subst .ko,.o,$(target))

all: $(target)

$(target): cidatatty.c
	$(MAKE) -C $(KERNELDIR) M=$(shell pwd) "-I../INC"modules
	$(STRIP) --strip-unneeded $@

clean:
	$(MAKE) -C $(KERNELDIR) M=$(shell pwd) clean

