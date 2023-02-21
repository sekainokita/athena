#============================================================================
# Name        : Makefile for main
# Author      : Byoungman An
# Version     :
# Copyright   : Your copyright notice
# Description : Hello World in C, Ansi-style
#============================================================================
TARGET = keti-nrip-app

SRCS = $(wildcard *.c)

OBJS = ${SRCS:.c=.o}

%.o:%.c
	@echo "Compiling..."
	$(CC) $(CFLAGS) -c $^

$(TARGET):$(OBJS)
	@echo "Linking..."
	$(CC) -o $@ $^ $(CFLAGS) $(LDFLAGS) -pthread -lm
	cp -f keti-nrip-app ../../make

all: $(TARGET)

clean:
	@$(RM) $(TARGET)

distclean:
	@$(RM) $(OBJS) $(TARGET)

.PHONY: clean distclean

