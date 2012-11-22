#  (C) Copyright 2007 Marvell International Ltd.
#             All Rights Reserved
#  This software file (the "File") is distributed by Marvell International Ltd. 
#  under the terms of the GNU General Public License Version 2, June 1991 (the "License"). 
#  You may use, redistribute and/or modify this File in accordance with the terms and 
#  conditions of the License, a copy of which is available along with the File in the 
#  license.txt file or by writing to the Free Software Foundation, Inc., 59 Temple Place, 
#  Suite 330, Boston, MA 02111-1307 or on the worldwide web at http://www.gnu.org/licenses/gpl.txt.
#  THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED WARRANTIES 
#  OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY DISCLAIMED.  
#  The License provides additional details about this warranty disclaimer.

ifneq ($(KERNELRELEASE),)
	obj-m += mpdc_hs.o

	mpdc_hs-objs := dsa.o oshooks.o sample_buf.o module_buf.o \
                    hotspot_drv.o pmu.o sample_rec.o \
                    ../common/ring_buffer.o \
                    ../common/common.o

# set the default cpu type if CPU_TYPE is not defined
ifndef CPU_TYPE
	CPU_TYPE = PXA2
endif

# set flags for PJ4	
ifeq ($(CPU_TYPE), PJ4)

	EXTRA_CFLAGS += -DPX_CPU_PJ4

ifeq ($(SOC_TYPE), PXA688)
	EXTRA_CFLAGS += -DPX_SOC_PXA688
endif

ifeq ($(SOC_TYPE), PXA510)
	EXTRA_CFLAGS += -DPX_SOC_PXA510
endif

ifeq ($(SOC_TYPE), PXA968)
	EXTRA_CFLAGS += -DPX_SOC_PXA968
endif

	mpdc_hs-objs += ../common/pmu_pj4.o
	mpdc_hs-objs += pj4/isr.o pj4/timer.o
ifdef PJ4_TIMER_GROUP
	EXTRA_CFLAGS += -DPJ4_TIMER_GROUP=$(PJ4_TIMER_GROUP)
endif

ifdef PJ4_TIMER_NO
	EXTRA_CFLAGS += -DPJ4_TIMER_NO=$(PJ4_TIMER_NO)
endif

endif


# set flags for PJ1	
ifeq ($(CPU_TYPE), PJ1)
	EXTRA_CFLAGS += -DPX_CPU_PJ1

ifeq ($(SOC_TYPE), PXA168)
	EXTRA_CFLAGS += -DPX_SOC_PXA168
endif

ifeq ($(SOC_TYPE), PXA91x)
	EXTRA_CFLAGS += -DPX_SOC_PXA91x
endif
	mpdc_hs-objs += ../common/pmu_pj1.o
	mpdc_hs-objs += pj1/isr.o pj1/timer.o
ifdef PJ1_TIMER_GROUP
	EXTRA_CFLAGS += -DPJ1_TIMER_GROUP=$(PJ1_TIMER_GROUP)
endif

ifdef PJ1_TIMER_NO
	EXTRA_CFLAGS += -DPJ1_TIMER_NO=$(PJ1_TIMER_NO)
endif

endif

# set flags for PXA2	
ifeq ($(CPU_TYPE), PXA2)
	EXTRA_CFLAGS += -DPX_CPU_PXA2
	mpdc_hs-objs += ../common/pmu_pxa2.o
	mpdc_hs-objs += pxa2/isr.o
endif

ifdef CONFIG_PRM
	EXTRA_CFLAGS += -DPRM_SUPPORT
endif

ifdef CONFIG_PXA3xx_PRM
	EXTRA_CFLAGS += -DPRM_SUPPORT
endif

	EXTRA_CFLAGS += -I$(PWD) -I$(PWD)/inc -I$(PWD)/../common -DSYS_CALL_TABLE_ADDRESS=$(SYS_CALL_TABLE_ADDRESS) 
	#-DDEBUG_DCD -DDEBUG_MTRK -g
	EXTRA_CFLAGS += -fno-omit-frame-pointer -mapcs -mno-sched-prolog
else
ifneq ($(MAKERULES),)
include $(MAKERULES)
endif
	KDIR := $(PXALINUX)
	PWD := $(shell pwd)
all:
	@export SYS_CALL_TABLE_ADDRESS=0;\
	if [ -f $(KDIR)/System.map ]; then \
                SYS_CALL_TABLE_ADDRESS=0x`awk '/sys_call_table/{print $$1}' $(KDIR)/System.map`; \
        fi;\
	if [ -f $(KDIR)/vmlinux ]; then \
                SYS_CALL_TABLE_ADDRESS=0x`nm $(KDIR)/vmlinux | awk '/sys_call_table/{print $$1}'`; \
        fi; \
        if [ "$$SYS_CALL_TABLE_ADDRESS" = "0" ]; then \
                echo "warning: couldn't find sys_call_table address in file System.map and vmlinux!";\
        fi; \
        $(MAKE) -C $(KDIR) SUBDIRS=$(PWD) CROSS_COMPILE=$$CROSS_COMPILE ARCH=arm modules
	$(CROSS_COMPILE)strip -g *.ko
	$(hide)mkdir -p ../out
	$(hide)cp *.ko ../out

.PHONY: clean
clean:
	@rm -f *.o *.ko *.mod.c \.*.cmd
	@rm -f pxa2/*.o pxa2/*.ko pxa2/*.mod.c pxa2/\.*.cmd
	@rm -f pj1/*.o pj1/*.ko pj1/*.mod.c pj1/\.*.cmd
	@rm -f pj4/*.o pj4/*.ko pj4/*.mod.c pj4/\.*.cmd
	@rm -rf .tmp_versions
	@rm -f *.ko
	@rm -f Module.symvers modules.order
	@rm -f $(PWD)/../common/*.o $(PWD)/../common/*.mod.c $(PWD)/../common/.*.o.cmd
endif
