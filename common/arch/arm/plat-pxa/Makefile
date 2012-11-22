#
# Makefile for code common across different PXA processor families
#

obj-y	:= dma.o

obj-$(CONFIG_ARCH_PXA)		+= pmu.o
obj-$(CONFIG_GENERIC_GPIO)	+= gpio.o
obj-$(CONFIG_PXA3xx)		+= mfp.o
obj-$(CONFIG_PXA93x)		+= mfp.o
obj-$(CONFIG_PXA95x)		+= mfp.o
obj-$(CONFIG_ARCH_MMP)		+= mfp.o misc.o

obj-$(CONFIG_HAVE_PWM)		+= pwm.o
obj-$(CONFIG_PXA_SSP)		+= ssp.o
obj-$(CONFIG_PXA_VBUS)		+= vbus.o

# Support for PMEM
ifeq ($(CONFIG_ANDROID_PMEM),y)
obj-$(CONFIG_ANDROID_PMEM)      += pmem.o
endif
obj-y				+= reg_rw.o
