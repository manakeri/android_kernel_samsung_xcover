#
# Makefile for bmm kernel module
#

ifneq ($(KERNELRELEASE),)

#CFLAGS += -DBMM_DEBUG 

obj-m    := bmm.o
bmm-objs := bmm_drv.o

else

ifeq ($(MAKERULES),)
include ../../../Rules.make
else
include $(MAKERULES)
endif

.EXPORT_ALL_VARIABLES:
KDIR := $(PXA_SRC_PVK_LNX_DIR)
PWD  := $(shell pwd)

.PHONY: all compile install-host install-target clean clean-local \
        uninstall-host uninstall-target

all: compile install-host install-target

compile:
	$(MAKE) -C $(KDIR) M=$(PWD) modules MAKERULES=../../Rules.make 

install-host:
install-target:
	if [ ! -e $(PXA_TARGET_MOD_DIR) ];then $(SUDO) mkdir -p $(PXA_TARGET_MOD_DIR);fi
	if [ ! -e $(PXA_TARGET_BIN_DIR) ];then $(SUDO) mkdir -p $(PXA_TARGET_BIN_DIR);fi
	$(SUDO) cp -f bmm.ko $(PXA_TARGET_MOD_DIR)

clean: clean-local uninstall-host uninstall-target

clean-local:
	rm -rf *.o *.ko *.mod.c .*.cmd ./.tmp* *~

uninstall-host:
uninstall-target:
	$(SUDO) rm -f $(PXA_TARGET_MOD_DIR)/bmm.ko

endif

