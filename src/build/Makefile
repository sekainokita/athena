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
include .config

LIB_DB = db
LIB_MSG = msg
LIB_TIME = time
LIB_FRAMEWORK = framework
LIB_PLATFORM = platform
LIB_DI = di

APP_BIN_NAME = athena
PLAT_APP_BIN_NAME = plat-chem-obu-compact-app

ifeq ($(CONFIG_MODEL_NNCO1XG),y)
APP_MODEL_NAME = nnco1xg
else ifeq ($(CONFIG_MODEL_NNCO2OG),y)
APP_MODEL_NAME = nnco2og
else ifeq ($(CONFIG_MODEL_NNCO2XG),y)
APP_MODEL_NAME = nnco2xg
else ifeq ($(CONFIG_MODEL_NOCO1XG),y)
APP_MODEL_NAME = noco1xg
else ifeq ($(CONFIG_MODEL_NOCO2OG),y)
APP_MODEL_NAME = noco2og
else ifeq ($(CONFIG_MODEL_NOCO2XG),y)
APP_MODEL_NAME = noco2xg
else ifeq ($(CONFIG_MODEL_NOCO2XGR),y)
APP_MODEL_NAME = noco2xgr
else ifeq ($(CONFIG_MODEL_NOCO2OGM),y)
APP_MODEL_NAME = noco2ogm
else ifeq ($(CONFIG_MODEL_UICO1XG),y)
APP_MODEL_NAME = uico1xg
else ifeq ($(CONFIG_MODEL_UIDR2RG),y)
APP_MODEL_NAME = uidr2rg
else ifeq ($(CONFIG_MODEL_UIDR2RGM),y)
APP_MODEL_NAME = uidr2rgm
else ifeq ($(CONFIG_MODEL_XZCO1XGC),y)
APP_MODEL_NAME = xzco1xgc
else
APP_MODEL_NAME = keti
endif

APP_MODEL_BIN_NAME = $(APP_MODEL_NAME)-$(APP_BIN_NAME)

# CFLAGS / LDFLAGS
TARGET_CFLAGS = -I$(INC_PATH)
TARGET_CFLAGS += -I$(APP_INC_PATH)
TARGET_CFLAGS += -I$(FRAMEWORK_INC_PATH)
TARGET_CFLAGS += -I$(PLATFORM_INC_PATH)
TARGET_CFLAGS += -I$(DI_INC_PATH)
TARGET_CFLAGS += -D_GNU_SOURCE=1
TARGET_CFLAGS += -std=gnu99 -g
TARGET_CFLAGS += -Wall -Wextra -Wstrict-prototypes -O3 -fomit-frame-pointer -pipe
TARGET_CFLAGS += -std=c11

TARGET_CFLAGS += -DCONFIG_MODEL_NAME=\"\\\"$(APP_MODEL_NAME)\\\"\"
TARGET_CXXFLAGS += -DCONFIG_MODEL_NAME=\"\\\"$(APP_MODEL_NAME)\\\"\"

ifeq ($(CONFIG_NVIDIA_JETSON_UTILS),y)
TARGET_CFLAGS += -I/usr/local/include/jetson-inference
TARGET_CFLAGS += -I/usr/local/include/jetson-utils
TARGET_CFLAGS += -I/usr/local/cuda-11.4/targets/aarch64-linux/include
endif

ifeq ($(CONFIG_CAN_PEAK),y)
TARGET_CFLAGS += -I$(PCANBASIC_INC_PATH) -I$(PCAN_DRV_PATH)
endif

TARGET_LDFLAGS ?= -L$(OUT_LIB_DIR) -l$(LIB_FRAMEWORK) -l$(LIB_PLATFORM) -l$(LIB_DI)
TARGET_LDFLAGS += -lrt -ldl
TARGET_LDFLAGS += -pthread
TARGET_LDFLAGS += -lstdc++
ifeq ($(CONFIG_SQLITE),y)
TARGET_LDFLAGS += -lsqlite3
endif
# To add Library : -l$(LIB_DB) -l$(LIB_MSG) -l$(LIB_TIME)

ifeq ($(CONFIG_NVIDIA_JETSON_UTILS),y)
TARGET_LDFLAGS += -L/usr/local/lib -ljetson-utils
endif

ifeq ($(CONFIG_WEBSOCKET),y)
TARGET_LDFLAGS += -lwebsockets -lssl -lcrypto -luv
endif

ifeq ($(CONFIG_CAN_PEAK),y)
TARGET_LDFLAGS += -L$(PCANBASIC_LIB_PATH) -Wl,-rpath,$(PCANBASIC_LIB_PATH) -lpcanbasic
endif

TARGET_CXXFLAGS = -I$(INC_PATH)
TARGET_CXXFLAGS += -I$(APP_INC_PATH)
TARGET_CXXFLAGS += -I$(FRAMEWORK_INC_PATH)
TARGET_CXXFLAGS += -I$(PLATFORM_INC_PATH)
TARGET_CXXFLAGS += -I$(DI_INC_PATH)
TARGET_CXXFLAGS += -D_GNU_SOURCE=1
TARGET_CXXFLAGS += -std=c++11 -g
TARGET_CXXFLAGS += -Wall -Wextra -O3 -fomit-frame-pointer -pipe

ifeq ($(CONFIG_NVIDIA_JETSON_UTILS),y)
TARGET_CXXFLAGS += -I/usr/local/include/jetson-inference
TARGET_CXXFLAGS += -I/usr/local/include/jetson-utils
TARGET_CXXFLAGS += -I/usr/local/cuda-11.4/targets/aarch64-linux/include
endif

ifeq ($(CONFIG_ROS),y)
TARGET_CFLAGS += -DCONFIG_ROS
TARGET_CXXFLAGS += -DCONFIG_ROS
endif

ifeq ($(CONFIG_WEBSOCKET),y)
TARGET_CFLAGS += -DCONFIG_WEBSOCKET
TARGET_CXXFLAGS += -DCONFIG_WEBSOCKET
endif

ifeq ($(CONFIG_GPS_XSENS),y)
TARGET_CFLAGS += -DCONFIG_GPS_XSENS
TARGET_CXXFLAGS += -DCONFIG_GPS_XSENS
endif

ifeq ($(CONFIG_CAN),y)
TARGET_CFLAGS += -DCONFIG_CAN
TARGET_CXXFLAGS += -DCONFIG_CAN
endif

ifeq ($(CONFIG_CAN_PEAK),y)
TARGET_CFLAGS += -DCONFIG_CAN_PEAK
TARGET_CXXFLAGS += -DCONFIG_CAN_PEAK
endif

ifeq ($(CONFIG_OBU),y)
TARGET_CFLAGS += -DCONFIG_OBU
TARGET_CXXFLAGS += -DCONFIG_OBU
endif

ifeq ($(CONFIG_RSU),y)
TARGET_CFLAGS += -DCONFIG_RSU
TARGET_CXXFLAGS += -DCONFIG_RSU
endif

ifeq ($(CONFIG_GPS_OBU),y)
TARGET_CFLAGS += -DCONFIG_GPS_OBU
TARGET_CXXFLAGS += -DCONFIG_GPS_OBU
endif

ifeq ($(CONFIG_GPS_RSU),y)
TARGET_CFLAGS += -DCONFIG_GPS_RSU
TARGET_CXXFLAGS += -DCONFIG_GPS_RSU
endif

ifdef CONFIG_RSU_MAX_DEV
TARGET_CFLAGS += -DCONFIG_RSU_MAX_DEV=$(CONFIG_RSU_MAX_DEV)
TARGET_CXXFLAGS += -DCONFIG_RSU_MAX_DEV=$(CONFIG_RSU_MAX_DEV)
endif

ifeq ($(CONFIG_SQLITE),y)
TARGET_CFLAGS += -DCONFIG_SQLITE
TARGET_CXXFLAGS += -DCONFIG_SQLITE
endif

ifeq ($(CONFIG_PLATFORM_OBU_CHEMTRONICS),y)
TARGET_CFLAGS += -DCONFIG_PLATFORM_OBU_CHEMTRONICS
TARGET_CXXFLAGS += -DCONFIG_PLATFORM_OBU_CHEMTRONICS
endif

ifeq ($(CONFIG_PLATFORM_OBU_CHEMTRONICS_COMPACT),y)
TARGET_CFLAGS += -DCONFIG_PLATFORM_OBU_CHEMTRONICS_COMPACT
TARGET_CXXFLAGS += -DCONFIG_PLATFORM_OBU_CHEMTRONICS_COMPACT
endif

ifeq ($(CONFIG_EXT_DATA_FORMAT),y)
TARGET_CFLAGS += -DCONFIG_EXT_DATA_FORMAT
TARGET_CXXFLAGS += -DCONFIG_EXT_DATA_FORMAT
endif

ifeq ($(CONFIG_RTC),y)
TARGET_CFLAGS += -DCONFIG_RTC
TARGET_CXXFLAGS += -DCONFIG_RTC
endif

ifdef CONFIG_OBU_MAX_DEV
TARGET_CFLAGS += -DCONFIG_OBU_MAX_DEV=$(CONFIG_OBU_MAX_DEV)
TARGET_CXXFLAGS += -DCONFIG_OBU_MAX_DEV=$(CONFIG_OBU_MAX_DEV)
endif

TARGET_CFLAGS += -DCONFIG_SW_VERSION=$(CONFIG_SW_VERSION)
TARGET_CXXFLAGS += -DCONFIG_SW_VERSION=$(CONFIG_SW_VERSION)

TARGET_LFLAGS = -lpthread -lrt

ifeq ($(CONFIG_GPS_XSENS),y)
TARGET_LFLAGS += -L$(XSENS_MTI680G_LIB_PATH) -lxscontroller -lxscommon -lxstypes -ldl
endif

TARGET_CC=gcc
TARGET_LD=ld
TARGET_AR=ar
TARGET_RANLIB=ranlib

TARGET_CONFIGS = \
	BUILD_TOP="$(TOP_DIR)" \
	CC="$(TARGET_CC)" \
	LD="$(TARGET_LD)" \
	AR="$(TARGET_AR)" \
	RANLIB="$(TARGET_RANLIB)" \
	CFLAGS="$(TARGET_CFLAGS)" \
	CXXFLAGS="$(TARGET_CXXFLAGS)" \
	LDFLAGS="$(TARGET_LDFLAGS)" \
	LFLAGS="$(TARGET_LFLAGS)"
#
# Library
#
lib-db:
	@$(MAKE) -C $(LIB_DB_PATH) $(TARGET_CONFIGS) all

lib-db-clean:
	$(MAKE) -C $(LIB_DB_PATH) $(TARGET_CONFIGS) clean

lib-db-distclean:
	$(MAKE) -C $(LIB_DB_PATH) $(TARGET_CONFIGS) distclean

.PHONY: lib-db lib-db-clean lib-db-distclean

lib-msg:
	@$(MAKE) -C $(LIB_MSG_PATH) $(TARGET_CONFIGS) all

lib-msg-clean:
	$(MAKE) -C $(LIB_MSG_PATH) $(TARGET_CONFIGS) clean

lib-msg-distclean:
	$(MAKE) -C $(LIB_MSG_PATH) $(TARGET_CONFIGS) distclean

.PHONY: lib-msg lib-msg-clean lib-msg-distclean

lib-time:
	@$(MAKE) -C $(LIB_TIME_PATH) $(TARGET_CONFIGS) all

lib-time-clean:
	$(MAKE) -C $(LIB_TIME_PATH) $(TARGET_CONFIGS) clean

lib-time-distclean:
	$(MAKE) -C $(LIB_TIME_PATH) $(TARGET_CONFIGS) distclean

.PHONY: lib-time lib-time-clean lib-time-distclean

lib-framework:
	@$(MAKE) -C $(LIB_FRAMEWORK_PATH) $(TARGET_CONFIGS) all

lib-framework-clean:
	$(MAKE) -C $(LIB_FRAMEWORK_PATH) $(TARGET_CONFIGS) clean

lib-framework-distclean:
	$(MAKE) -C $(LIB_FRAMEWORK_PATH) $(TARGET_CONFIGS) distclean

.PHONY: lib-framework lib-framework-clean lib-framework-distclean

lib-platform:
	@$(MAKE) -C $(LIB_PLATFORM_PATH) $(TARGET_CONFIGS) all

lib-platform-clean:
	$(MAKE) -C $(LIB_PLATFORM_PATH) $(TARGET_CONFIGS) clean

lib-platform-distclean:
	$(MAKE) -C $(LIB_PLATFORM_PATH) $(TARGET_CONFIGS) distclean

.PHONY: lib-platform lib-platform-clean lib-platform-distclean

lib-di:
	@$(MAKE) -C $(LIB_DI_PATH) $(TARGET_CONFIGS) all

lib-di-clean:
	$(MAKE) -C $(LIB_DI_PATH) $(TARGET_CONFIGS) clean

lib-di-distclean:
	$(MAKE) -C $(LIB_DI_PATH) $(TARGET_CONFIGS) distclean

.PHONY: lib-di lib-di-clean lib-di-distclean
#
# Application
#
app:
	@$(MAKE) -C $(APP_PATH) $(TARGET_CONFIGS) all

app-clean:
	$(MAKE) -C $(APP_PATH) $(TARGET_CONFIGS) clean

app-distclean:
	$(MAKE) -C $(APP_PATH) $(TARGET_CONFIGS) distclean

.PHONY: app app-clean app-distclean

# Platform
plat-chem-obu:
	@$(MAKE) -C $(LIB_PLAT_CHEM_OBU_PATH) $(TARGET_CONFIGS) all

plat-chem-obu-clean:
	$(MAKE) -C $(LIB_PLAT_CHEM_OBU_PATH) $(TARGET_CONFIGS) clean

plat-chem-obu-distclean:
	$(MAKE) -C $(LIB_PLAT_CHEM_OBU_PATH) $(TARGET_CONFIGS) distclean

.PHONY: plat-chem-obu plat-chem-obu-clean plat-chem-obu-distclean

#
# Build All
#
lib-all:lib-db lib-msg lib-time
all: lib-di lib-platform lib-framework app lib-all
.PHONY: lib-all all

#
# Release
#
release:
	@echo release $(RELEASE_DIR)/$(CONFIG_SW_VERSION)
	rm -rf $(RELEASE_DIR)/$(CONFIG_SW_VERSION)
	mkdir -p $(RELEASE_DIR)/$(CONFIG_SW_VERSION)
	mkdir -p $(RELEASE_DIR)/$(CONFIG_SW_VERSION)/bin
	cp -af $(OUT_BIN_DIR)/$(APP_BIN_NAME) $(RELEASE_DIR)/$(CONFIG_SW_VERSION)/bin/$(APP_MODEL_BIN_NAME)
	cp -af $(OUT_BIN_DIR)/$(PLAT_APP_BIN_NAME) $(RELEASE_DIR)/$(CONFIG_SW_VERSION)/bin/$(PLAT_APP_BIN_NAME)
	cp -a $(OUT_LIB_DIR) $(RELEASE_DIR)/$(CONFIG_SW_VERSION)/lib

#
# Clean
#
clean: distclean

distclean: lib-db-distclean lib-msg-distclean lib-time-distclean lib-di-distclean lib-platform-distclean lib-framework-distclean app-distclean
	@$(RM) -f $(OUT_BIN_DIR)/* $(OUT_LIB_DIR)/*
	@$(RM) -f *.txt *.csv *.db *.log
