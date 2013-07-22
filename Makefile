#
# This file will be copied to <topdir>/kernel and change the name to Makefile by repo.
# If you don't use repo, please copy this file to <topdir>/kernel folder manually.
#

hide:=@
log=@echo [$(shell date "+%Y-%m-%d %H:%M:%S")]

MAKE_JOBS ?= 4
#KERNEL_TOOLCHAIN_PREFIX := /path/to/arm-eabi-4.4.3/bin/arm-eabi-
OUTDIR := out

MODULES_BUILT=
MODULES_CLEAN=
define add-module
	MODULES_BUILT+=$(1)
	MODULES_CLEAN+=clean_$(1)
endef

export ARCH := arm
#export CROSS_COMPILE := $(KERNEL_TOOLCHAIN_PREFIX)
export KERNELDIR := $(shell pwd)/common
export TARGET_PRODUCT := GT-S5690

.PHONY:help
help:
	$(hide)echo "======================================="
	$(hide)echo "= This file wraps the build of kernel and modules"
	$(hide)echo "= make all: to make  kernel, all modules and telephony. The kernel, modules and telephony will be output to 'out' directory."
	$(hide)echo "= make kernel: only make the kernel. Using KERNEL_CONFIG variable to specify the kernel config file to be used. By default it is: $(KERNEL_CONFIG)"
	$(hide)echo "= make modules: only make all the modules. The kernel should already be built. Otherwise building modules will fail."
	$(hide)echo "= make clean: clean the kernel, modules and telephony"
	$(hide)echo "======================================="

all: kernel modules
KERNEL_TGT := common/arch/arm/boot/zImage
.PHONY: kernel clean_kernel
kernel:
	$(log) "making kernel..."
	$(hide)cp defconfig common/.config
	$(hide)cd common && make -j$(MAKE_JOBS)
	$(hide)mkdir -p $(OUTDIR)
	$(hide)cat /dev/zero | head -c 4096 > $(OUTDIR)/header
	$(hide)cat $(OUTDIR)/header $(KERNEL_TGT) > $(OUTDIR)/zImage
	$(hide)cd $(OUTDIR) && ../pack.sh
	$(log) "kernel [$(KERNEL_CONFIG)] done"

.PHONY:clean_kernel clean_modules
clean_kernel:
	$(hide)cd common && make clean
	$(hide)rm -f $(OUTDIR)/zImage
	$(log) "Kernel cleaned."

clean: clean_kernel clean_modules
	$(hide)rm -fr $(OUTDIR)

SD8787_DRVSRC:= modules/sd8787/
.PHONY: sd8787_wifi clean_sd8787_wifi
sd8787_wifi:
	$(log) "making sd8787 wifi driver..."
	$(hide)cd $(SD8787_DRVSRC)/wlan_src && \
	make -j$(MAKE_JOBS) default
	$(hide)mkdir -p $(OUTDIR)/modules/
	$(hide)cp $(SD8787_DRVSRC)/wlan_src/sd8xxx.ko $(OUTDIR)/modules/sd8787.ko
	$(hide)cp $(SD8787_DRVSRC)/wlan_src/mlan.ko $(OUTDIR)/modules/mlan.ko
	$(log) "sd8787 wifi driver done."

clean_sd8787_wifi:
	$(hide)cd $(SD8787_DRVSRC)/wlan_src &&\
	make clean
	rm -f $(OUTDIR)/modules/sd8787.ko
	$(log) "sd8787 wifi driver cleaned."

$(eval $(call add-module,sd8787_wifi) )


.PHONY: sd8787_mbtchar clean_sd8787_mbtchar
sd8787_mbtchar:
	$(log) "making sd8787 mbtchar BT driver..."
	$(hide)cd $(SD8787_DRVSRC)/mbtchar_src && \
	make -j$(MAKE_JOBS) default
	$(hide)mkdir -p $(OUTDIR)/modules/
	$(hide)cp $(SD8787_DRVSRC)/mbtchar_src/mbtchar.ko $(OUTDIR)/modules/mbtchar.ko
	$(log) "sd8787 mbtchar bt driver done."

clean_sd8787_mbtchar:
	$(hide)cd $(SD8787_DRVSRC)/mbtchar_src &&\
	make clean
	$(hide)rm -f $(OUTDIR)/modules/mbtchar.ko
	$(log) "sd8787 mbtchar driver cleaned."

$(eval $(call add-module,sd8787_mbtchar) )

.PHONY: sd8787_mbt clean_sd8787_mbt
sd8787_mbt:
	$(log) "making sd8787 mbt BT driver..."
	$(hide)cd $(SD8787_DRVSRC)/mbt_src && \
	make -j$(MAKE_JOBS) default
	$(hide)mkdir -p $(OUTDIR)/modules/
	$(hide)cp $(SD8787_DRVSRC)/mbt_src/mbt8xxx.ko $(OUTDIR)/modules/mbt8787.ko
	$(log) "sd8787 mbt driver done."

clean_sd8787_mbt:
	$(hide)cd $(SD8787_DRVSRC)/mbt_src &&\
	make clean
	$(hide)rm -f $(OUTDIR)/modules/mbt8787.ko
	$(log) "sd8787 mbt driver cleaned."

$(eval $(call add-module,sd8787_mbt) )

.PHONY: sd8787_bt clean_sd8787_bt
sd8787_bt:
	$(log) "making sd8787 bt BT driver..."
	$(hide)cd $(SD8787_DRVSRC)/bt_src && \
	make -j$(MAKE_JOBS) default
	$(hide)mkdir -p $(OUTDIR)/modules/
	$(hide)cp $(SD8787_DRVSRC)/bt_src/bt8xxx.ko $(OUTDIR)/modules/bt8787.ko
	$(log) "sd8787 bt bt driver done."

clean_sd8787_bt:
	$(hide)cd $(SD8787_DRVSRC)/bt_src &&\
	make clean
	$(hide)rm -f $(OUTDIR)/modules/bt8xxx.ko
	$(log) "sd8787 bt driver cleaned."

$(eval $(call add-module,sd8787_bt) )


GC800_DRVSRC:= modules/graphics/galcore_src
export KERNEL_DIR:=$(KERNELDIR)
.PHONY: gc800 clean_gc800
gc800:
	$(log) "make gc800 driver..."
	$(hide)cd $(GC800_DRVSRC) &&\
	make -j$(MAKE_JOBS)
	$(hide)mkdir -p $(OUTDIR)/modules/
	$(hide)cp $(GC800_DRVSRC)/hal/driver/galcore.ko $(OUTDIR)/modules
	$(log) "gc800 driver done."

clean_gc800:
	$(hide)cd $(GC800_DRVSRC) &&\
	make clean
	$(hide)rm -f $(OUTDIR)/modules/galcore.ko
	$(log) "gc800 driver cleaned."

$(eval $(call add-module,gc800) )

BMM_DRVSRC:= modules/bmm-lib
.PHONY: bmm clean_bmm
bmm:
	$(log) "make bmm driver..."
	$(hide)cd $(BMM_DRVSRC)/drv &&\
	make -f Makefile_Android
	$(hide)mkdir -p $(OUTDIR)/modules/
	$(hide)cp $(BMM_DRVSRC)/drv/bmm.ko $(OUTDIR)/modules
	$(log) "bmm driver done."

clean_bmm:
	$(hide)cd $(BMM_DRVSRC)/drv &&\
	make clean -f Makefile_Android
	$(hide)rm -f $(OUTDIR)/modules/bmm.ko
	$(log) "bmm driver cleaned."

$(eval $(call add-module,bmm) )

PMEM_DRVSRC:=./common/arch/arm/plat-pxa
.PHONY: pmem clean_pmem
pmem:
	$(log) "make pmem driver..."
	$(hide)cp $(PMEM_DRVSRC)/pmem.ko $(OUTDIR)/modules
	$(log) "pmem driver done."

clean_pmem:
	$(hide)rm -f $(PMEM_DRVSRC)/pmem.ko
	$(log) "pmem driver cleaned."

MPDC_DRVSRC:= modules/mpdc/
export PXALINUX:=$(KERNELDIR)
export CPU_TYPE:=PJ4
export SOC_TYPE:=PXA968
.PHONY: mpdc clean_mpdc
mpdc:
	$(log) "make mpdc driver..."
	$(hide)cd $(MPDC_DRVSRC) &&\
	make -C src/driver/ PXALINUX=$(PXALINUX) CROSS_COMPILE=$(CROSS_COMPILE) CPU_TYPE=$(CPU_TYPE) SOC_TYPE=$(SOC_TYPE)
	$(hide)mkdir -p $(OUTDIR)/modules/
	$(hide)cp $(MPDC_DRVSRC)/src/driver/out/mpdc_cm.ko $(OUTDIR)/modules
	$(hide)cp $(MPDC_DRVSRC)/src/driver/out/mpdc_css.ko $(OUTDIR)/modules
	$(hide)cp $(MPDC_DRVSRC)/src/driver/out/mpdc_hs.ko $(OUTDIR)/modules
	$(log) "mpdc driver done."

clean_mpdc:
	$(hide)cd $(MPDC_DRVSRC)/src/driver &&\
	make clean
	$(hide)rm -f $(OUTDIR)/modules/mpdc_cm.ko
	$(hide)rm -f $(OUTDIR)/modules/mpdc_css.ko
	$(hide)rm -f $(OUTDIR)/modules/mpdc_hs.ko
	$(log) "mpdc driver cleaned."

$(eval $(call add-module,mpdc) )

#insert any module declaration above

.PHONY: modules
modules:$(MODULES_BUILT)

TOUCH_DRVSRC:= modules/touch
.PHONY: touch clean_touch
touch:
	$(log) "make touch driver..."
	$(hide)mkdir -p $(OUTDIR)/modules/
	$(hide)cd $(TOUCH_DRVSRC) &&\
	make OUTDIR=$(PWD)/$(OUTDIR)
	$(hide)cp $(TOUCH_DRVSRC)/*.ko $(OUTDIR)/modules

	$(log) "touch driver done."

clean_touch:
	$(hide)cd $(TOUCH_DRVSRC) &&\
	make clean OUTDIR=$(PWD)/$(OUTDIR)
	$(log) "touch driver cleaned."
$(eval $(call add-module,touch) )

#insert any module declaration above

.PHONY: modules
modules:$(MODULES_BUILT)

CITTY_DRVSRC:= modules/citty
.PHONY: citty clean_citty
citty:
	$(log) "make citty driver..."
	$(hide)mkdir -p $(OUTDIR)/modules/
	$(hide)cd $(CITTY_DRVSRC) &&\
        make all OUTDIR=$(PWD)/$(OUTDIR)
	$(hide)cp $(CITTY_DRVSRC)/*.ko $(OUTDIR)/modules
	$(log) "citty driver done."

clean_citty:
	$(hide)cd $(CITTY_DRVSRC) &&\
        make clean OUTDIR=$(PWD)/$(OUTDIR)
	$(log) "citty driver cleaned."
$(eval $(call add-module,citty) )


.PHONY: modules
modules:$(MODULES_BUILT)

CIDATATTY_DRVSRC:= modules/cidatatty
.PHONY: cidatatty clean_cidatatty
cidatatty:
	$(log) "make cidatatty driver..."
	$(hide)mkdir -p $(OUTDIR)/modules/
	$(hide)cd $(CIDATATTY_DRVSRC) &&\
        make all OUTDIR=$(PWD)/$(OUTDIR)
	$(hide)cp $(CIDATATTY_DRVSRC)/*.ko $(OUTDIR)/modules
	$(log) "cidatatty driver done."

clean_cidatatty:
	$(hide)cd $(CIDATATTY_DRVSRC) &&\
        make clean OUTDIR=$(PWD)/$(OUTDIR)
	$(log) "cidatatty driver cleaned."
$(eval $(call add-module,cidatatty) )


.PHONY: modules
modules:$(MODULES_BUILT)
	$(hide)mkdir -p $(OUTDIR)/modules/

clean_modules: $(MODULES_CLEAN)

