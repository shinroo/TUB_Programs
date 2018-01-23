BIN = GameOfLife
SRC = \
	game_field.c \
	gamer_thread.c \
	printer_thread.c \
	utils.c \
	game_of_life.c	

ARCH = $(shell getconf LONG_BIT)

SDL_CFLAGS_64 = $(shell libsdl_64/bin/sdl-config --cflags)
SDL_LFLAGS_64 = $(shell libsdl_64/bin/sdl-config --static-libs)

SDL_CFLAGS_32 = $(shell libsdl_32/bin/sdl-config --cflags)
SDL_LFLAGS_32 = $(shell libsdl_32/bin/sdl-config --static-libs)

CC	 = gcc
CFLAGS   = -g -Wall -Wextra -O3 -std=c99 $(SDL_CFLAGS_$(ARCH))
LFLAGS   += $(SDL_LFLAGS_$(ARCH))


GDB    = gdb
RM     = rm -f

OBJS = $(SRC:%.c=%.o)

all: $(BIN)


$(BIN): $(OBJS)
	$(CC) $(CFLAGS) -o $(BIN) $(OBJS) $(LFLAGS)

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
