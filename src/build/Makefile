#============================================================================
# Name        : Top Makefile
# Author      : Byoungman An
# Version     :
# Copyright   : Your copyright notice
# Description : Hello World in C, Ansi-style
#============================================================================

# Project path
CUR_DIR = $(shell pwd)
TOP_DIR = $(CUR_DIR)/..
TOP_MAKE_DIR = $(TOP_DIR)/make

# Source path
CLIB_PATH = ../src/clib
MAIN_PATH = ../src/main

# Include path
INC_PATH = $(TOP_DIR)/include
LIB_PATH = $(TOP_MAKE_DIR)/lib

# CFLAGS / LDFLAGS
TARGET_CFLAGS = -I$(INC_PATH)
TARGET_CFLAGS += -D_GNU_SOURCE=1
TARGET_CFLAGS += -std=gnu99
TARGET_LDFLAGS ?= -L$(LIB_PATH) -lclib -lpthread

#
# Compiler
#

#CROSS_COMPILER = powerpc-linux-gnu-
CROSS_COMPILER = /opt/Xilinx/Vitis/2020.2/gnu/aarch64/lin/aarch64-linux/bin/aarch64-linux-gnu-

#TARGET_CC=gcc
TARGET_CC=$(CROSS_COMPILER)gcc
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
# Common Library
#

clib:
	@$(MAKE) -C $(CLIB_PATH) $(TARGET_CONFIGS) all

clib-clean:
	$(MAKE) -C $(CLIB_PATH) clean

clib-distclean:
	$(MAKE) -C $(CLIB_PATH) distclean
	rm -f ./lib/*.a

.PHONY: clib clib-clean clib-distclean

#
# Main
#

main:
	@$(MAKE) -C $(MAIN_PATH) $(TARGET_CONFIGS) all

main-clean:
	$(MAKE) -C $(MAIN_PATH) clean

main-distclean:
	$(MAKE) -C $(MAIN_PATH) distclean
	rm -f ./main ./keti-nrip-app

.PHONY: main main-clean main-distclean

#
# Build All
#

all: clib main

#
# Build and Run
#

keti-run: distclean clib main
	./keti-nrip-app

#
# Clean
#

clean: distclean

distclean: clib-distclean main-distclean

