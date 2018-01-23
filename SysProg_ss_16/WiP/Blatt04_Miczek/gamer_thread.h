#ifndef GAMEOFLIFE_GAMER_THREAD_H
#define GAMEOFLIFE_GAMER_THREAD_H

#include "game_field.h"


typedef struct {
    int id;
    game_field *field;
} gamer_args;

// Diese Funktion läuft in einer Dauerschleife, während der jeder Thread die 
// Folgegenerationen seines Anteils am Spielfeld berechnet.
void* play_game(void* args);

#endif //GAMEOFLIFE_GAMER_THREAD_H
