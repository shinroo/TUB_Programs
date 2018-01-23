#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include "SDL.h"

#include "utils.h"
#include "game_field.h"
#include "gamer_thread.h"
#include "printer_thread.h"
#include "config.h"


int main() {
    if (NUM_GAMER_THREADS > GAMEFIELD_HEIGHT/2){
        perror("Too many threads!");
        exit(1);
    } else if (NUM_GAMER_THREADS < 1){
		perror("There must be at least one gamer thread");
        exit(1);
    }
    // Deklariere Threads
    pthread_t gamer[NUM_GAMER_THREADS], printer;
    // Erzeuge das Spielfeld
    game_field* field = make_game_field(GAMEFIELD_WIDTH, GAMEFIELD_HEIGHT, NUM_GAMER_THREADS);
    // Starte Gamer-Threads
    for (int i = 0; i < NUM_GAMER_THREADS; i++) {
        gamer_args *g_args = (gamer_args*) emalloc(sizeof(gamer_args));
        g_args->id = (unsigned int) i;
        g_args->field = field;
        if(0 != pthread_create(&gamer[i], NULL, play_game, (void*) g_args)) {
            perror("Error while creating the gamer threads");
            exit(1);
        }
    }
    // Starte Printer-Thread
    if (0 != pthread_create(&printer, NULL, print_game, (void*) field)) {
        perror("Error while creating the printer thread");
        exit(1);
    }
    // Warte bis sich die Gamer-Threads beenden
    for (int i = 0; i < NUM_GAMER_THREADS; i++) {
        pthread_join(gamer[i], NULL);
    }
    pthread_join(printer, NULL);
    return 0;
}
