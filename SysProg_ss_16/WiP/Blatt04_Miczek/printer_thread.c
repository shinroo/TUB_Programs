#include <stdbool.h>
#include <unistd.h>
#include "SDL.h"
#include "printer_thread.h"
#include "utils.h"
#include <sys/time.h>
#include "config.h"

#define RGBA_MASK 0, 0, 0, 0 // DO NOT CHANGE THIS!
struct timeval stop, start;


void update_matrix(SDL_Surface* screen, game_field* field, int** last_field) { 
	SDL_Surface* cell = SDL_CreateRGBSurface(SDL_HWSURFACE, CELLWIDTH-BORDER_WIDTH, CELLHEIGHT-BORDER_WIDTH, 32, RGBA_MASK);

	// Die verwendeten Farben
	Uint32 white = SDL_MapRGB(cell->format, 0xFF, 0xFF, 0xFF);
    Uint32 green = SDL_MapRGB(cell->format, 0x00, 0xA0, 0x00);

	// Iteriere über alle Zellen des Spielfelds
    for (int x = 0; x < field->width; x++) {
        for (int y = 0; y < field->height; y++) {
			// Zeichne nur seit dem letzten Aufruf veränderte Zellen neu
            if (field->field[y][x] != last_field[y][x]) {
                if (field->field[y][x] == 1) {
					// Zelle lebt => grün einfärben
                	SDL_FillRect(cell, 0, green);
                	SDL_Rect pos;
					pos.x = x*CELLWIDTH;
					pos.y = y*CELLHEIGHT;
        			SDL_BlitSurface(cell, 0, screen, &pos);
        			last_field[y][x] = 1;
                } else {
					// Zelle ist tot => weiß einfärben
                	SDL_FillRect(cell, 0, white);
                	SDL_Rect pos;
					pos.x = x*CELLWIDTH;
					pos.y = y*CELLHEIGHT;
        			SDL_BlitSurface(cell, 0, screen, &pos);
        			last_field[y][x] = 0;
                }
            }
        }
    }
}


void draw_grid(SDL_Surface* screen, game_field* game) {
	Uint32 black = SDL_MapRGB(screen->format, 0x00, 0x00, 0x00);

	SDL_Rect vertical_line, horizontal_line;

	// Zeichne vertikale Gitterlinien
	vertical_line.y = 0;
	vertical_line.w = BORDER_WIDTH;
	vertical_line.h = game->height*CELLHEIGHT;

    for (int i = 0; i < game->width; i++) {
        vertical_line.x = CELLWIDTH - BORDER_WIDTH + CELLWIDTH*i;
        SDL_FillRect(screen,&vertical_line,black);
    }

	// Zeichne horizontale Gitterlinien
    horizontal_line.x = 0;
    horizontal_line.w = game->width*CELLWIDTH;
    horizontal_line.h = BORDER_WIDTH;

    for (int i = 0; i < game->height; i++) {
        horizontal_line.y = CELLHEIGHT - BORDER_WIDTH + CELLHEIGHT*i;
        SDL_FillRect(screen,&horizontal_line,black);
	}
}


void *print_game(void *args) {
	game_field* game = (game_field*) args;

	if (GUI) {
		// Graphische Ausgabe via SDL, falls GUI in der config.h auf '1' gesetzt wurde
		SDL_Init(SDL_INIT_VIDEO);
		SDL_WM_SetCaption("Game Of Life", 0);

		SDL_Surface* screen = SDL_SetVideoMode(game->width*CELLWIDTH, game->height*CELLHEIGHT, 32, SDL_HWSURFACE | SDL_DOUBLEBUF);

		Uint32 bgcolor = SDL_MapRGB(screen->format, 0xFF, 0xFF, 0xFF);
		SDL_FillRect(screen, 0, bgcolor);
		draw_grid(screen, game);

		int** last_field = (int**) emalloc(sizeof(int*)*game->height);
    	for (int i = 0; i < game->height; i++) {
        	last_field[i] = (int*) emalloc(game->width*sizeof(int));
        	memset(last_field[i], 0, game->width*sizeof(int));
    	}

		SDL_Event event;
		int gameRunning = true;
		int delay = REFRESH_RATE;

		while (gameRunning) {
			// Tastendruck-Events pollen
			if (SDL_PollEvent(&event)) {
				// Abbuch falls ESC gedrückt wird
				if (event.type == SDL_QUIT) {
					gameRunning = false;
					break;
				}
			}

			// Aktualisiere nur alle <REFRESH_RATE> ms
			if (delay == REFRESH_RATE) {
				print_game_field(GUI, screen, game, last_field);
				SDL_Flip(screen);
				delay = 0;
			} else {
				SDL_Delay(1);
				delay++;
			}
		}

		SDL_Quit();
		return EXIT_SUCCESS;
	} else {
		// Ausgabe in der Konsole, falls GUI in der config.h auf '0' gesetzt wurde
		while(true){
			print_game_field(GUI, NULL, game, NULL);
			usleep(REFRESH_RATE*1000);
		}
	}
}
