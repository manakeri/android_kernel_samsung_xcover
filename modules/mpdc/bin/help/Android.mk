LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

MPDC_HELP_DIR := $(TARGET_OUT_ETC)/mpdc/help

file := $(MPDC_HELP_DIR)/all
$(file) : $(LOCAL_PATH)/all | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(file)

file := $(MPDC_HELP_DIR)/cm_options
$(file) : $(LOCAL_PATH)/cm_options | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(file)

file := $(MPDC_HELP_DIR)/css_options
$(file) : $(LOCAL_PATH)/css_options | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(file)

file := $(MPDC_HELP_DIR)/hs_options
$(file) : $(LOCAL_PATH)/hs_options | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(file)

file := $(MPDC_HELP_DIR)/pt_list
$(file) : $(LOCAL_PATH)/pt_list | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(file)

file := $(MPDC_HELP_DIR)/mohawk/event
$(file) : $(LOCAL_PATH)/mohawk/event | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(file)

file := $(MPDC_HELP_DIR)/pxa168/event
$(file) : $(LOCAL_PATH)/pxa168/event | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(file)

file := $(MPDC_HELP_DIR)/pxa3xx/event
$(file) : $(LOCAL_PATH)/pxa3xx/event | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(file)

file := $(MPDC_HELP_DIR)/pxa510/event
$(file) : $(LOCAL_PATH)/pxa510/event | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(file)

file := $(MPDC_HELP_DIR)/pxa610/event
$(file) : $(LOCAL_PATH)/pxa610/event | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(file)

file := $(MPDC_HELP_DIR)/pxa93x/event
$(file) : $(LOCAL_PATH)/pxa93x/event | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(file)

file := $(MPDC_HELP_DIR)/pxa940/event
$(file) : $(LOCAL_PATH)/pxa940/event | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(file)

file := $(MPDC_HELP_DIR)/pxa955/event
$(file) : $(LOCAL_PATH)/pxa955/event | $(ACP)
	$(transform-prebuilt-to-target)
ALL_PREBUILT += $(file)