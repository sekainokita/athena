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

TARGET = app

APP_BIN = app-v2x

SRCS = $(wildcard *.c)
SRCS += $(wildcard $(APP_CLI_PATH)/*.c)
SRCS += $(wildcard $(APP_SVC_PLATOONING_PATH)/*.c)
SRCS += $(wildcard $(APP_SVC_CP_PATH)/*.c)

OBJS = ${SRCS:.c=.o}

OUT_BIN_DIR = ../build/output/bin

%.o:%.c
	@echo "Compiling..."
	$(CC) $(CFLAGS) -c $^
	mkdir -p $(APP_OBJS_DIR)
	mv *.o $(APP_OBJS_DIR)

$(TARGET):$(OBJS)
	@echo "Linking..."
	@echo "Check SRCS:$(SRCS), OBJS:$(OBJS)"
	$(CC) -o $@ $(APP_OBJS) $(CFLAGS) $(LDFLAGS) $(LFLAGS) -lm
	mv $(TARGET) $(APP_BIN)
	cp -f $(APP_BIN) $(OUT_BIN_DIR)/$(APP_BIN)

all: $(TARGET)

clean:
	@$(RM) -f $(OBJS) $(APP_BIN) $(OUT_BIN_DIR)/$(APP_BIN)
	@$(RM) -rf $(APP_OBJS_DIR)

distclean: clean

.PHONY: clean distclean

