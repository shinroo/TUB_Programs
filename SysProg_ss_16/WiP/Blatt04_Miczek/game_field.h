#ifndef GAMEOFLIFE_GAME_FIELD_H
#define GAMEOFLIFE_GAME_FIELD_H

#include <pthread.h>
#include <stdbool.h>
#include "SDL.h"


/**
 * field: Pointer auf das 2-dimensionale Spielfeld
 * field_filled: Zählvariable, um zu zählen, wie viele der Gamer-Threads bisher
 *   ihre berechnete Folgegeneration in das Spielfeld geschrieben haben
 * field_read: Zählvariable, um zu zählen, wie viele der Gamer-Threads bisher
 *   ihre Ränder aus der aktuellen Generation ausgelesen haben
 * printed: Der Printer-Thread kann hiermit angeben, das er die aktuelle
 *   Generation ausgeben hat
 * num_threads: Anzahl der Gamerthreads
 * height: Höhe des Spielfeldes
 * width: Breite des Spielfeldes
 */
typedef struct {
    int** field;
    int field_filled;
    int field_read;
    bool printed;
    int num_threads;
    int height;
    int width;
    pthread_mutex_t field_filled_m;
    pthread_mutex_t field_read_m;
    pthread_mutex_t printed_m;
} game_field;


// Initialisert das Spielfeld
game_field* make_game_field(int width,int height,int num_threads);


/**
 * Schreibt die berechnete Folgegeneration in das Spielfeld
 * field: Zeiger auf das Spielfeld
 * row_start: Erste Zeile des Spielfeldes, das der jeweilige Thread berechnet
 * num_rows: Anzahl der Zeilen, die der jeweilige Thread berechnet
 * src_mat: Zeiger auf die Matrix in der der jeweilige Threads die Folgegeneration
 *   zwischenspeichert
 **/
int write_to_field(game_field *field, int row_start, int num_rows, int **src_mat);


/**
* Gibt das aktuelle Spielfeld auf der Konsole oder mittels SDL in einem Fenster aus
* GUI: Entscheidet ob graphisch via SDL (GUI=1) oder ohne (GUI=0)
* screen: Zeiger auf das SDL-Fenster. NULL, falls GUI = 0.
* field: Zeiger auf das Spielfeld
* last_field: Zeiger auf SDL-Swap-Buffer. NULL, falls GUI = 0.
**/
void print_game_field(int GUI, SDL_Surface* screen, game_field *field, int** last_field);

/**
 * Der jeweilige Gamer-Thread liest in read_adjacent_lines seine Ränder aus dem Spielfeld
 * field: Pointer auf das Spielfeld
 * height_above: Offset der oben angrenzenden Zeile des Spielfeldes
 * height_below: Offset der unten angrenzenden Zeile des Spielfeldes
 * line_above: Buffer für die oben angrenzende Zeile
 * line_below: Buffer für die unten angrenzende Zeile
 */
void read_adjacent_lines(game_field *field, int height_above, int height_below, int **line_above, int **line_below);

#endif //GAMEOFLIFE_GAME_FIELD_H
