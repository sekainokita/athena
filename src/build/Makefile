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

include path.mk

LIB_DB = db
LIB_MSG = msg
LIB_TIME = time
LIB_FRAMEWORK = framework
LIB_PLATFORM = platform

# CFLAGS / LDFLAGS
TARGET_CFLAGS = -I$(INC_PATH)
TARGET_CFLAGS += -I$(APP_INC_PATH)
TARGET_CFLAGS += -I$(FRAMEWORK_INC_PATH)
TARGET_CFLAGS += -I$(PLATFORM_INC_PATH)
TARGET_CFLAGS += -I$(LIB_PLAT_CHEM_OBU_INC_PATH)
TARGET_CFLAGS += -D_GNU_SOURCE=1
TARGET_CFLAGS += -std=gnu99 -g
TARGET_CFLAGS += -Wall -Wextra -Wstrict-prototypes -O3 -fomit-frame-pointer -pipe

TARGET_LDFLAGS ?= -L$(OUT_LIB_DIR) -l$(LIB_FRAMEWORK) -l$(LIB_PLATFORM)
TARGET_LDFLAGS += -lrt -pthread
# To add Library : -l$(LIB_DB) -l$(LIB_MSG) -l$(LIB_TIME)

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

lib-msg:
	@$(MAKE) -C $(LIB_MSG_PATH) $(TARGET_CONFIGS) all

lib-msg-clean:
	$(MAKE) -C $(LIB_MSG_PATH) clean

lib-msg-distclean:
	$(MAKE) -C $(LIB_MSG_PATH) distclean

.PHONY: lib-msg lib-msg-clean lib-msg-distclean

lib-time:
	@$(MAKE) -C $(LIB_TIME_PATH) $(TARGET_CONFIGS) all

lib-time-clean:
	$(MAKE) -C $(LIB_TIME_PATH) clean

lib-time-distclean:
	$(MAKE) -C $(LIB_TIME_PATH) distclean

.PHONY: lib-time lib-time-clean lib-time-distclean

lib-framework:
	@$(MAKE) -C $(LIB_FRAMEWORK_PATH) $(TARGET_CONFIGS) all

lib-framework-clean:
	$(MAKE) -C $(LIB_FRAMEWORK_PATH) clean

lib-framework-distclean:
	$(MAKE) -C $(LIB_FRAMEWORK_PATH) distclean

.PHONY: lib-framework lib-framework-clean lib-framework-distclean

lib-platform:
	@$(MAKE) -C $(LIB_PLATFORM_PATH) $(TARGET_CONFIGS) all

lib-platform-clean:
	$(MAKE) -C $(LIB_PLATFORM_PATH) clean

lib-platform-distclean:
	$(MAKE) -C $(LIB_PLATFORM_PATH) distclean

.PHONY: lib-platform lib-platform-clean lib-platform-distclean

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

# Platform
plat-chem-obu:
	@$(MAKE) -C $(LIB_PLAT_CHEM_OBU_PATH) $(TARGET_CONFIGS) all

plat-chem-obu-clean:
	$(MAKE) -C $(LIB_PLAT_CHEM_OBU_PATH) clean

plat-chem-obu-distclean:
	$(MAKE) -C $(LIB_PLAT_CHEM_OBU_PATH) distclean

.PHONY: plat-chem-obu plat-chem-obu-clean plat-chem-obu-distclean

#
# Build All
#

lib-all:lib-db lib-msg lib-time
all: lib-platform lib-framework app lib-all

#
# Clean
#

clean: distclean

distclean: lib-db-distclean lib-msg-distclean lib-time-distclean lib-platform-distclean lib-framework-distclean app-distclean
	@$(RM) -f $(OUT_BIN_DIR)/* $(OUT_LIB_DIR)/*
