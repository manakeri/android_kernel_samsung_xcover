LOCAL_PATH := $(call my-dir)

wifi_firmware1 := $(TARGET_OUT_ETC)/firmware/mrvl/sd8787_uapsta.bin
$(wifi_firmware1) : $(LOCAL_PATH)/sd8787_uapsta.bin | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(wifi_firmware1)

# Description of WLAN CCX Feature
# SD8XXX_CCX : To enable the CCX feature
# Should enable the option in Makefile in Makefile in kernel folder

SD8XXX_CCX = false

ifeq ($(SD8XXX_CCX), true)
wifi_firmware_ccx := $(TARGET_OUT_ETC)/firmware/mrvl/sd8787.bin
$(wifi_firmware_ccx) : $(LOCAL_PATH)/sd8787.bin | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(wifi_firmware_ccx)

wifi_ccx1 := $(TARGET_OUT_ETC)/ccx/libcrypto.so
$(wifi_ccx1) : $(LOCAL_PATH)/ccx_bin/libcrypto.so | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(wifi_ccx1)

wifi_ccx2 := $(TARGET_OUT_ETC)/ccx/libssl.so
$(wifi_ccx2) : $(LOCAL_PATH)/ccx_bin/libssl.so | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(wifi_ccx2)

wifi_ccx3 := $(TARGET_OUT_ETC)/ccx/run.sh
$(wifi_ccx3) : $(LOCAL_PATH)/ccx_bin/run.sh | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(wifi_ccx3)

wifi_ccx4 := $(TARGET_OUT_ETC)/ccx/wpa_cli
$(wifi_ccx4) : $(LOCAL_PATH)/ccx_bin/wpa_cli | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(wifi_ccx4)

wifi_ccx5 := $(TARGET_OUT_ETC)/ccx/wpa_cli_ent
$(wifi_ccx5) : $(LOCAL_PATH)/ccx_bin/wpa_cli_ent | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(wifi_ccx5)

wifi_ccx6 := $(TARGET_OUT_ETC)/ccx/wpa_supplicant
$(wifi_ccx6) : $(LOCAL_PATH)/ccx_bin/wpa_supplicant | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(wifi_ccx6)

wifi_ccx7 := $(TARGET_OUT_ETC)/ccx/wpa_supplicant.conf
$(wifi_ccx7) : $(LOCAL_PATH)/ccx_bin/wpa_supplicant.conf | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(wifi_ccx7)

wifi_ccx8 := $(TARGET_OUT_ETC)/ccx/wpa_supplicant_ent
$(wifi_ccx8) : $(LOCAL_PATH)/ccx_bin/wpa_supplicant_ent | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(wifi_ccx8)

wifi_ccx9 := $(TARGET_OUT_ETC)/ccx/chmod.sh
$(wifi_ccx9) : $(LOCAL_PATH)/ccx_bin/chmod.sh | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(wifi_ccx9)
endif
