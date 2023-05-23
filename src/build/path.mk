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

CUR_DIR = $(shell pwd)

TOP_DIR = $(CUR_DIR)/..
APP_DIR = $(TOP_DIR)/apps
BUILD_DIR = $(TOP_DIR)/build
DRIVER_DIR = $(TOP_DIR)/drivers
FRAMEWORK_DIR = $(TOP_DIR)/framework
INCLUDE_DIR = $(TOP_DIR)/include
PACKAGE_DIR = $(TOP_DIR)/packages
PLATFORM_DIR = $(TOP_DIR)/platform
TOOL_DIR = $(TOP_DIR)/tools

OUT_DIR = $(BUILD_DIR)/output
OUT_BIN_DIR = $(OUT_DIR)/bin
OUT_LIB_DIR = $(OUT_DIR)/lib

# Source path
LIB_DB_PATH = $(FRAMEWORK_DIR)/db
LIB_MSG_PATH = $(FRAMEWORK_DIR)/msg
LIB_FRAMEWORK_PATH = $(FRAMEWORK_DIR)
APP_PATH = $(APP_DIR)
APP_CLI_PATH = $(APP_DIR)/cli

# Include path
INC_PATH = $(TOP_DIR)/include
APP_INC_PATH = $(APP_DIR)/include
FRAMEWORK_INC_PATH = $(FRAMEWORK_DIR)/include
PLATFORM_INC_PATH = $(PLATFORM_DIR)/include
LIB_PATH = $(TOP_MAKE_DIR)/lib

# Objs
FRAMEWORK_OBJS_DIR=$(FRAMEWORK_DIR)/objs
FRAMEWORK_OBJS = $(FRAMEWORK_OBJS_DIR)/*.o
PLATFORM_OBJS_DIR=$(PLATFORM_DIR)/objs
PLATFORM_OBJS = $(PLATFORM_OBJS_DIR)/*.o
APP_OBJS_DIR=$(APP_DIR)/objs
APP_OBJS = $(APP_OBJS_DIR)/*.o

# Platform path
LIB_PLATFORM_PATH = $(PLATFORM_DIR)

# Chemtronics
LIB_PLAT_CHEM_PATH = $(PLATFORM_DIR)/chemtronics
LIB_PLAT_CHEM_OBU_PATH = $(LIB_PLAT_CHEM_PATH)/obu
LIB_PLAT_CHEM_OBU_INC_PATH = $(PLATFORM_DIR)/chemtronics/obu/include
