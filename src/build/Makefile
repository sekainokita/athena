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

# PATH
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
APP_PATH = $(APP_DIR)

# Include path
INC_PATH = $(TOP_DIR)/include
LIB_PATH = $(TOP_MAKE_DIR)/lib

LIB_DB = db

# CFLAGS / LDFLAGS
TARGET_CFLAGS = -I$(INC_PATH)
TARGET_CFLAGS += -D_GNU_SOURCE=1
TARGET_CFLAGS += -std=gnu99
TARGET_CFLAGS += -Wall -Wstrict-prototypes -O3 -fomit-frame-pointer -pipe

#TARGET_LDFLAGS = -lpthread
TARGET_LDFLAGS ?= -L$(OUT_LIB_DIR) -l$(LIB_DB)

#
# Compile Flags
#

# Cross Compiler
#CROSS_COMPILER = powerpc-linux-gnu-
#CROSS_COMPILER = /opt/Xilinx/Vitis/2020.2/gnu/aarch64/lin/aarch64-linux/bin/aarch64-linux-gnu-
#TARGET_CC=$(CROSS_COMPILER)gcc

# GCC (x86)
TARGET_CC=gcc
TARGET_LD=ld
TARGET_AR=ar
TARGET_RANLIB=ranlib

TARGET_CONFIGS = \
	CC="$(TARGET_CC)" \
	LD="$(TARGET_LD)" \
	AR="$(TARGET_AR)" \
	RANLIB="$(TARGET_RANLIB)" \
	CFLAGS="$(TARGET_CFLAGS)" \
	LDFLAGS="$(TARGET_LDFLAGS)"
#
# Library
#
lib-db:
	@$(MAKE) -C $(LIB_DB_PATH) $(TARGET_CONFIGS) all

lib-db-clean:
	$(MAKE) -C $(LIB_DB_PATH) clean

lib-db-distclean:
	$(MAKE) -C $(LIB_DB_PATH) distclean

.PHONY: lib-db lib-db-clean lib-db-distclean

#
# Application
#
app:
	@$(MAKE) -C $(APP_PATH) $(TARGET_CONFIGS) all

app-clean:
	$(MAKE) -C $(APP_PATH) clean

app-distclean:
	$(MAKE) -C $(APP_PATH) distclean

.PHONY: app app-clean app-distclean

#
# Build All
#

all: lib-db app

#
# Clean
#

clean: distclean

distclean: lib-db-distclean app-distclean
	@$(RM) -f $(OUT_BIN_DIR)/* $(OUT_LIB_DIR)/*
