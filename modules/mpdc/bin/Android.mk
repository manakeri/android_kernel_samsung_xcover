LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)
TOP_PATH:=$(LOCAL_PATH)

MPDC_BIN_DIR := $(TARGET_OUT_EXECUTABLES)

file := $(MPDC_BIN_DIR)/mpdc
$(file) : $(LOCAL_PATH)/mpdc | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(file)

file := $(MPDC_BIN_DIR)/mpdc_d
$(file) : $(LOCAL_PATH)/mpdc_d | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(file)

file := $(MPDC_BIN_DIR)/mpdc_svr
$(file) : $(LOCAL_PATH)/mpdc_svr | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(file)

file := $(MPDC_BIN_DIR)/pxidle
$(file) : $(LOCAL_PATH)/pxidle | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(file)

file := $(MPDC_BIN_DIR)/load_mpdc.sh
$(file) : $(LOCAL_PATH)/load_mpdc.sh | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(file)

file := $(MPDC_BIN_DIR)/unload_mpdc.sh
$(file) : $(LOCAL_PATH)/unload_mpdc.sh | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(file)

file := $(MPDC_BIN_DIR)/pxksymaddr
$(file) : $(LOCAL_PATH)/pxksymaddr | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(file)

include $(TOP_PATH)/config/Android.mk
include $(TOP_PATH)/lib/Android.mk
include $(TOP_PATH)/help/Android.mk
