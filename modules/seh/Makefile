
#
# Set the driver file name.
#
TARGET= seh.ko


#
#	You don't need to make change hereafter.
#
CC = $(CROSS_COMPILE)gcc
AR = $(CROSS_COMPILE)ar
LD = $(CROSS_COMPILE)ld
STRIP = $(CROSS_COMPILE)strip
M=$(shell pwd)


EXTRA_CFLAGS = $(COMMON_CFLAGS)
EXTRA_CFLAGS+=-DOSA_LINUX\
	-DOSA_USE_ASSERT\
	-DUSE_OSA_SEMA\
	-DENV_LINUX

target := $(notdir $(TARGET))
#obj-m  := $(subst .ko,.o,$(target))
seh-objs := seh_linux.o
obj-m  := seh.o

all: $(target)

$(target): seh_linux.c
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

