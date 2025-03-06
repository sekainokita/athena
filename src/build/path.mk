#******************************************************************************
#
# Copyright (C) 2023 - 2028 KETI, All rights reserved.
#                           (Korea Electronics Technology Institute)
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# Use of the Software is limited solely to applications:
# (a) running for Korean Government Project, or
# (b) that interact with KETI project/platform.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# KETI BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
# WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
# OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
# Except as contained in this notice, the name of the KETI shall not be used
# in advertising or otherwise to promote the sale, use or other dealings in
# this Software without prior written authorization from KETI.
#
#******************************************************************************

include .config

export BUILD_TOP_DIR = $(shell pwd)

export TOP_DIR = $(BUILD_TOP_DIR)/..
export APP_DIR = $(TOP_DIR)/apps
export BUILD_DIR = $(TOP_DIR)/build
export DI_DIR = $(TOP_DIR)/drivers
export FRAMEWORK_DIR = $(TOP_DIR)/framework
export INCLUDE_DIR = $(TOP_DIR)/include
export PACKAGE_DIR = $(TOP_DIR)/packages
export PLATFORM_DIR = $(TOP_DIR)/platform
export TOOL_DIR = $(TOP_DIR)/tools
export RELEASE_DIR = $(TOP_DIR)/../release/athena

export OUT_DIR = $(BUILD_DIR)/output
export OUT_BIN_DIR = $(OUT_DIR)/bin
export OUT_LIB_DIR = $(OUT_DIR)/lib

# Source path
export LIB_DB_PATH = $(FRAMEWORK_DIR)/db
export LIB_MSG_PATH = $(FRAMEWORK_DIR)/msg
export LIB_TIME_PATH = $(FRAMEWORK_DIR)/time
export LIB_FRAMEWORK_PATH = $(FRAMEWORK_DIR)
export LIB_PLATFORM_PATH = $(PLATFORM_DIR)
export LIB_DI_GPS_PATH = $(DI_DIR)/gps
export LIB_DI_CAMERA_PATH = $(DI_DIR)/camera
export LIB_DI_VIDEO_PATH = $(DI_DIR)/video
export LIB_DI_CAN_PATH = $(DI_DIR)/can
export LIB_DI_PATH = $(DI_DIR)
export APP_PATH = $(APP_DIR)
export APP_CLI_PATH = $(APP_DIR)/cli
export APP_SVC_PATH = $(APP_DIR)/services
export APP_SVC_PLATOONING_PATH = $(APP_SVC_PATH)/platooning
export APP_SVC_CP_PATH = $(APP_SVC_PATH)/cp

# Include path
export INC_PATH = $(TOP_DIR)/include
export APP_INC_PATH = $(APP_DIR)/include
export FRAMEWORK_INC_PATH = $(FRAMEWORK_DIR)/include
export PLATFORM_INC_PATH = $(PLATFORM_DIR)/include
export DI_INC_PATH = $(DI_DIR)/include
export LIB_PATH = $(TOP_MAKE_DIR)/lib

# Objs
export FRAMEWORK_OBJS_DIR=$(FRAMEWORK_DIR)/objs
export FRAMEWORK_OBJS = $(FRAMEWORK_OBJS_DIR)/*.o
export PLATFORM_OBJS_DIR=$(PLATFORM_DIR)/objs
export PLATFORM_OBJS = $(PLATFORM_OBJS_DIR)/*.o
export DI_OBJS_DIR=$(DI_DIR)/objs
export DI_OBJS = $(DI_OBJS_DIR)/*.o
export APP_OBJS_DIR=$(APP_DIR)/objs
export APP_OBJS = $(APP_OBJS_DIR)/*.o

# Chemtronics
ifeq ($(CONFIG_PLATFORM_OBU_CHEMTRONICS),y)
export LIB_PLAT_CHEM_PATH = $(PLATFORM_DIR)/chemtronics
ifeq ($(CONFIG_PLATFORM_OBU_CHEMTRONICS_COMPACT),y)
export LIB_PLAT_CHEM_OBU_PATH = $(LIB_PLAT_CHEM_PATH)/obu-compact
export LIB_PLAT_CHEM_OBU_INC_PATH = $(PLATFORM_DIR)/chemtronics/obu-compact/include
else
export LIB_PLAT_CHEM_OBU_PATH = $(LIB_PLAT_CHEM_PATH)/obu
export LIB_PLAT_CHEM_OBU_INC_PATH = $(PLATFORM_DIR)/chemtronics/obu/include
endif
endif

# XSENS
export XSENS_MTI680G_SDK_PATH = $(PLATFORM_DIR)/movella/xsens/MTi-680g/mtsdk-2022.0-xda_public_cpp/xspublic
export XSENS_MTI680G_AARCH64_LIB_PATH = $(PLATFORM_DIR)/movella/xsens/MTi-680g/lib/arm_aarch64
export XSENS_MTI680G_NANO_LIB_PATH = $(PLATFORM_DIR)/movella/xsens/MTi-680g/lib/arm_nano
export XSENS_MTI680G_X64_LIB_PATH = $(PLATFORM_DIR)/movella/xsens/MTi-680g/lib/ubuntu_x64
ifeq ($(CONFIG_UBUNTU),y)
export XSENS_MTI680G_LIB_PATH = $(XSENS_MTI680G_X64_LIB_PATH)
else ifeq ($(CONFIG_NVIDIA_NANO),y)
export XSENS_MTI680G_LIB_PATH = $(XSENS_MTI680G_NANO_LIB_PATH)
else
export XSENS_MTI680G_LIB_PATH = $(XSENS_MTI680G_AARCH64_LIB_PATH)
endif
