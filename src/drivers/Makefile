#============================================================================
# Name        : Makefile for clib
# Author      : Byoungman An
# Version     :
# Copyright   : Your copyright notice
# Description : Hello World in C, Ansi-style
#============================================================================

TARGET = clib
SRCS = $(wildcard *.c)
TARGET_AR = ar
OBJS = ${SRCS:.c=.o}

%.o:%.c
	@echo "Compiling..."
	$(CC) $(CFLAGS) -c $^ -pthread

$(TARGET):$(OBJS)
	@echo "Linking..."
	@$(TARGET_AR) rcs libclib.a $(OBJS)
	@echo "Make and copy the common library..."
	@mv libclib.a ../../make/lib

all: $(TARGET)

clean:
	@$(RM) $(TARGET)

distclean:
	@$(RM) $(OBJS) $(TARGET)

.PHONY: clean distclean

