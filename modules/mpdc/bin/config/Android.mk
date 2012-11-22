LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

MPDC_CONFIG_DIR := $(TARGET_OUT_ETC)/mpdc/config

file := $(MPDC_CONFIG_DIR)/config_internal.txt
$(file) : $(LOCAL_PATH)/config_internal.txt | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(file)

file := $(MPDC_CONFIG_DIR)/pxa610.xml
$(file) : $(LOCAL_PATH)/pxa610.xml | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(file)

file := $(MPDC_CONFIG_DIR)/system_info.xml
$(file) : $(LOCAL_PATH)/system_info.xml | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(file)

file := $(MPDC_CONFIG_DIR)/manzano.xml
$(file) : $(LOCAL_PATH)/manzano.xml | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(file)

file := $(MPDC_CONFIG_DIR)/mohawk.xml
$(file) : $(LOCAL_PATH)/mohawk.xml | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(file)

file := $(MPDC_CONFIG_DIR)/pxa168.xml
$(file) : $(LOCAL_PATH)/pxa168.xml | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(file)

file := $(MPDC_CONFIG_DIR)/pxa3xx.xml
$(file) : $(LOCAL_PATH)/pxa3xx.xml | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(file)

file := $(MPDC_CONFIG_DIR)/pxa510.xml
$(file) : $(LOCAL_PATH)/pxa510.xml | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(file)

file := $(MPDC_CONFIG_DIR)/pxa93x.xml
$(file) : $(LOCAL_PATH)/pxa93x.xml | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(file)

file := $(MPDC_CONFIG_DIR)/pxa940.xml
$(file) : $(LOCAL_PATH)/pxa940.xml | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(file)

file := $(MPDC_CONFIG_DIR)/pxa955.xml
$(file) : $(LOCAL_PATH)/pxa955.xml | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(file)