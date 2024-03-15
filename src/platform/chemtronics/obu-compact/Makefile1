APP_TARGET = nr_v2x_interface
CC := gcc

LDFLAGS = -lpthread

SRCS := $(wildcard *.c)
OBJS := $(SRCS:%.c=%.o)

all: $(APP_TARGET)

$(APP_TARGET): $(OBJS)
	$(CC) -o $@ $(OBJS) $(LDFLAGS)

$(OBJS): %o : %c
	$(CC) $(LDFLAGS) -c $< -o $@

clean:
	rm -rf *.o $(APP_TARGET)
	