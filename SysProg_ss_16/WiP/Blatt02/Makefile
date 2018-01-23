BIN = bitcoin
SRC = \
	util.c \
	blockheader.c \
	main.c

CC = cc
CFLAGS = -g -std=c99  
LFLAGS = -lm
LDFLAGS = libgcrypt.a libgpg-error.a

GDB = gdb
RM = rm -f

OBJS = $(SRC:%.c=%.o)

all: $(BIN)

$(BIN): $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) $(LFLAGS) -o $@ $(LDFLAGS)

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
