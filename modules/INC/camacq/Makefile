obj-m := camacq.o
camacq-objs := camacq_core.o camacq_api.o camacq_ext.o

KDIR := ../../../../kernel/kernel
PWD  := $(shell pwd)

default:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules

clean:
	rm -rf *.ko
	rm -rf *.mod.*
	rm -rf .*.cmd
	rm -rf *.o

EXTRA_CFLAGS += -Idrivers/media/video
EXTRA_CFLAGS += -D_LINUX_

ifeq "$(TARGET_MODEL)" "Jetta"
EXTRA_CFLAGS += -D_MAKE_MODEL_JETTA_
else ifeq "$(TARGET_MODEL)" "Alkon"
EXTRA_CFLAGS += -D_MAKE_MODEL_ALKON_
else ifeq "$(TARGET_MODEL)" "Gforce"
EXTRA_CFLAGS += -D_MAKE_MODEL_GFORCE_
else 
EXTRA_CFLAGS += -D_MAKE_MODEL_ALKON_
endif

