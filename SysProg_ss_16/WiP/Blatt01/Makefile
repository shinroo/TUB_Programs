BIN_VORGABE = heap
BIN_LOESUNG = loesung
BIN_TEST_VORGABE = test_vorgabe
BIN_TEST_LOESUNG = test_loesung
SRC_VORGABE = \
	heap_vorgabe.c
SRC_LOESUNG = \
	heap_loesung.c
SRC_TEST_VORGABE = \
	heap_vorgabe.c \
	test.c
SRC_TEST_LOESUNG = \
	heap_loesung.c \
	test.c

CC	     = gcc
CFLAGS   = -g -Wall -Wextra -O2
LFLAGS   = -lm

GDB    = gdb
RM     = rm -f

OBJS_VORGABE = $(SRC_VORGABE:%.c=%.o)

OBJS_LOESUNG = $(SRC_LOESUNG:%.c=%.o)

OBJS_TEST_VORGABE = $(SRC_TEST_VORGABE:%.c=%.o)

OBJS_TEST_LOESUNG = $(SRC_TEST_LOESUNG:%.c=%.o)

all: $(BIN_VORGABE)

$(BIN_VORGABE): $(OBJS_VORGABE)
	$(CC) $(CFLAGS) $(OBJS_VORGABE) $(LFLAGS) -o $@

loesung: $(BIN_LOESUNG)

$(BIN_LOESUNG): $(OBJS_LOESUNG)
	$(CC) $(CFLAGS) $(OBJS_LOESUNG) $(LFLAGS) -o $@

test_vorgabe: $(BIN_TEST_VORGABE)

$(BIN_TEST_VORGABE): $(OBJS_TEST_VORGABE)
	$(CC) $(CFLAGS) $(OBJS_TEST_VORGABE) $(LFLAGS) -o $@

test_loesung: $(BIN_TEST_LOESUNG)

$(BIN_TEST_LOESUNG): $(OBJS_TEST_LOESUNG)
	$(CC) $(CFLAGS) $(OBJS_TEST_LOESUNG) $(LFLAGS) -o $@

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
	$(RM) $(BIN_TEST_VORGABE) $(BIN_TEST_LOESUNG) $(BIN_LOESUNG) $(BIN_VORGABE) $(OBJS_TEST_VORGABE) $(OBJS_TEST_LOESUNG) $(OBJS_LOESUNG) $(OBJS_VORGABE) *.d *-e

.PHONY: clean debug runBIN_VORGABE = heap
BIN_LOESUNG = loesung
BIN_TEST_VORGABE = test_vorgabe
BIN_TEST_LOESUNG = test_loesung
SRC_VORGABE = \
	heap_vorgabe.c
SRC_LOESUNG = \
	heap_loesung.c
SRC_TEST_VORGABE = \
	heap_vorgabe.c \
	test.c
SRC_TEST_LOESUNG = \
	heap_loesung.c \
	test.c

CC	     = gcc
CFLAGS   = -g -Wall -Wextra -O2
LFLAGS   = -lm

GDB    = gdb
RM     = rm -f

OBJS_VORGABE = $(SRC_VORGABE:%.c=%.o)

OBJS_LOESUNG = $(SRC_LOESUNG:%.c=%.o)

OBJS_TEST_VORGABE = $(SRC_TEST_VORGABE:%.c=%.o)

OBJS_TEST_LOESUNG = $(SRC_TEST_LOESUNG:%.c=%.o)

all: $(BIN_VORGABE)

$(BIN_VORGABE): $(OBJS_VORGABE)
	$(CC) $(CFLAGS) $(OBJS_VORGABE) $(LFLAGS) -o $@

loesung: $(BIN_LOESUNG)

$(BIN_LOESUNG): $(OBJS_LOESUNG)
	$(CC) $(CFLAGS) $(OBJS_LOESUNG) $(LFLAGS) -o $@

test_vorgabe: $(BIN_TEST_VORGABE)

$(BIN_TEST_VORGABE): $(OBJS_TEST_VORGABE)
	$(CC) $(CFLAGS) $(OBJS_TEST_VORGABE) $(LFLAGS) -o $@

test_loesung: $(BIN_TEST_LOESUNG)

$(BIN_TEST_LOESUNG): $(OBJS_TEST_LOESUNG)
	$(CC) $(CFLAGS) $(OBJS_TEST_LOESUNG) $(LFLAGS) -o $@

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
	$(RM) $(BIN_TEST_VORGABE) $(BIN_TEST_LOESUNG) $(BIN_LOESUNG) $(BIN_VORGABE) $(OBJS_TEST_VORGABE) $(OBJS_TEST_LOESUNG) $(OBJS_LOESUNG) $(OBJS_VORGABE) *.d *-e

.PHONY: clean debug run