KDR ?= /home/nie/git/pxa910-linux-2.6.32
#KDR ?= /home/nie/git/pxa168-linux-2.6.29

all: bt wlan install

wlan:
	cd wlan_src; \
	make -j4 CROSS_COMPILE=arm-eabi-4.4.0- KERNELDIR=${KDR}

bt:
	cd bt_src; \
	make -j4 CROSS_COMPILE=arm-eabi-4.4.0- KERNELDIR=${KDR} default

install:
	adb push wlan_src/mlan.ko /system/lib/modules/mlan.ko
	adb push wlan_src/sd8xxx.ko /system/lib/modules/sd8787.ko
	adb push bt_src/bt8xxx.ko /system/lib/modules/bt8787.ko
	adb push FwImage/sd8787_uapsta.bin /system/etc/firmware/mrvl/sd8787_uapsta.bin

clean: wlan_clean bt_clean

wlan_clean:
	make -C wlan_src clean
bt_clean:
	make -C bt_src clean
