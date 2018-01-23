BIN = scheduler


SRC = \
	benchmark.c \
	Queue.c \
	main.c \
	task.c \
	MLF.c \
	FIFO.c \
	SRTN.c \

CC	     = cc
CFLAGS   = -g -Wall -Wextra -O2 #-Werror
LFLAGS   = -lm

GDB    = gdb
RM     = rm -f

OBJS = $(SRC:%.c=%.o)

all: $(BIN)

$(BIN): $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) $(LFLAGS) -o $(BIN)
-include $(OBJS:.o=.d)

%.o: %.c
	$(CC) $(CFLAGS) -c $<
	$(CC) -MM $(CFLAGS) $*.c > $*.d

debug: $(BIN)
	$(GDB) ./$(BIN)

run: $(BIN)
	./$(BIN)

clean:
	$(RM) $(BIN) $(OBJS) *.d

.PHONY: clean debug run
