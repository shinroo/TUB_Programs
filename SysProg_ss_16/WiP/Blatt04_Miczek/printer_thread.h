#ifndef GAMEOFLIFE_PRINTER_THREAD_H
#define GAMEOFLIFE_PRINTER_THREAD_H

#include "game_field.h"

// Falls GUI in der config.h auf '1' gesetzt wurde, zeichnet diese Funktion bei
// Aufruf alle Änderungen des Spielfelds seit dem letzten Aufruf
void update_matrix(SDL_Surface* screen, game_field* field, int** last_field);

// Falls GUI in der config.h auf '1' gesetzt wurde, zeichnet diese Funktion bei
// Aufruf das Matrixgitter des Spielfelds neu/nach.
void draw_grid(SDL_Surface* screen, game_field* game);

// Gibt in regelmäßigen Abständen den Zustand des Spielfelds aus.
void* print_game(void* args);

#endif //GAMEOFLIFE_PRINTER_THREAD_H
