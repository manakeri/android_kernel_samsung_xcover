#  (C) Copyright 2010 Marvell International Ltd.
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

export CPU_TYPE SOC_TYPE
export PXALINUX CROSS_COMPILE

HS_DRIVER_DIR ?= HSProfiler
CM_DRIVER_DIR ?= CMProfiler
CSS_DRIVER_DIR ?= CSSProfiler

all: driver

driver: hs_driver cm_driver css_driver

hs_driver:
	make -C $(HS_DRIVER_DIR) PXALINUX=$(PXALINUX) CROSS_COMPILE=$(CROSS_COMPILE) CPU_TYPE=$(CPU_TYPE) SOC_TYPE=$(SOC_TYPE)
cm_driver:
	make -C $(CM_DRIVER_DIR) PXALINUX=$(PXALINUX) CROSS_COMPILE=$(CROSS_COMPILE) CPU_TYPE=$(CPU_TYPE) SOC_TYPE=$(SOC_TYPE)
css_driver:
	make -C $(CSS_DRIVER_DIR) PXALINUX=$(PXALINUX) CROSS_COMPILE=$(CROSS_COMPILE) CPU_TYPE=$(CPU_TYPE) SOC_TYPE=$(SOC_TYPE)

.PHONY: clean
clean:
	make -C $(HS_DRIVER_DIR) clean
	make -C $(CM_DRIVER_DIR) clean
	make -C $(CSS_DRIVER_DIR) clean
	rm -rf out

