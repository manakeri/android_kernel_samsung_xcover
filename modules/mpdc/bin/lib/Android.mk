LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

MPDC_LIB_DIR := $(TARGET_OUT_SHARED_LIBRARIES)

file := $(MPDC_LIB_DIR)/libpxcmprofiler.so
$(file) : $(LOCAL_PATH)/libpxcmprofiler.so | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(file)

file := $(MPDC_LIB_DIR)/libpxhsprofiler.so
$(file) : $(LOCAL_PATH)/libpxhsprofiler.so | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(file)

file := $(MPDC_LIB_DIR)/libpxserial.so
$(file) : $(LOCAL_PATH)/libpxserial.so | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(file)

file := $(MPDC_LIB_DIR)/libpxcmext.so
$(file) : $(LOCAL_PATH)/libpxcmext.so | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(file)

file := $(MPDC_LIB_DIR)/libpxcssprofiler.so
$(file) : $(LOCAL_PATH)/libpxcssprofiler.so | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(file)

file := $(MPDC_LIB_DIR)/libpxsdk.so
$(file) : $(LOCAL_PATH)/libpxsdk.so | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(file)

file := $(MPDC_LIB_DIR)/libpxutils.so
$(file) : $(LOCAL_PATH)/libpxutils.so | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(file)
