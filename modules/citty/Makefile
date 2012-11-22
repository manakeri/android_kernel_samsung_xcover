
#
# Set the driver file name.
#
TARGET= citty.ko


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

$(target): citty.c
	$(MAKE) -C $(KERNELDIR) M=$(shell pwd) modules
	$(STRIP) --strip-unneeded $@

clean:
	rm -rf                      \
        *.o       \
        *~        \
        core      \
        .*.d      \
        .*.cmd    \
        *.ko      \
        *.mod.c   \
        .tmp_versions   \
        Module.symvers   \
        *.bak

