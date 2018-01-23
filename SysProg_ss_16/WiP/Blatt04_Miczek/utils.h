#ifndef UTILS_H_
#define UTILS_H_

#include <stdint.h>
#include <stdlib.h>

// Sicheres Resevieren von Speicher
void* emalloc(size_t size);

// Vertauscht zwei Matrix-Zeiger
void swap (int*** m1, int*** m2);

// Zeichnet in das übergebene Spielfeld eine sog. "Glidergun". Die Figur kann via
// COL_OFFSET und ROW_OFFSET in der config.h verschoben werden. 
// Siehe: https://www.youtube.com/watch?v=fyrtJn5eK5U
void set_glider_gun(int width, int height, int **field);

// Zeichnet in das übergebene Spielfeld einen sog. "Glider". Die Figur kann via
// COL_OFFSET und ROW_OFFSET in der config.h verschoben werden.
void set_glider(int width, int height, int **field);

// Berechnet für den Thread mit der übergebenen ID aus der Größe des Spielfeldes
// und der Anzahl der Gamer-Threads die Größe des eigenen Zuständigkeitsbereichs
int compute_my_part(int height,int id, int max_threads);

#endif /* UTILS_H_ */
