BIN = scheduler


SRC = \
	#[LSG]
	AllTests.cpp \
	benchmark.c \
	Queue.c \
	#[/LSG]
	#[TUT]
	main.c \
	#[/TUT]
	task.c \
	LCFS.c \
	RR.c \
	SJN.c \
	MLF.c

CC	     = cc
CFLAGS   = -g -Wall -Wextra -Werror -O2
LFLAGS   = -lm

GDB    = gdb
RM     = rm -f

OBJS = $(SRC:%.c=%.o)

all: $(BIN)

$(BIN): $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) $(LFLAGS) -o $@

# pull in dependencies for .o files
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
