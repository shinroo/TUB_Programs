#include <string.h>
#include <stdio.h>
#include "game_field.h"
#include "utils.h"
#include "printer_thread.h"
#include <sys/time.h>


game_field* make_game_field(int width, int height, int num_threads) {

    // TODO: Erzeugen Sie ein Spielfeld mit der Breite width und der Höhe height
    game_field* gameField = (game_field*)malloc(sizeof(game_field));

    gameField->width = width;
    gameField->height = height;
    gameField->num_threads = num_threads;

    // TODO: Initialisieren Sie die erste Generation (Nutzen Sie hierfür die
	// Funktionen set_glider() und/oder set_glider_gun() aus utils.c)

  set_glider_gun(width, height, gameField->field);


    // TODO: Initialisieren Sie alle notwendigen Mutex- und Cond-Objekte sowie
	// weitere Struct-Komponenten (TIPP: Da set_glider/set_glider_gun so tun,
	// als ob die Threads das Spielfeld gefüllt hätten, intialisieren Sie hier
	// field_filled mit der Anzahl der verwendeten Gamer-Threads)

  gameField->field_filled = num_threads;
  gameField->field_read = num_threads;


}

int write_to_field(game_field *field, int row_start, int num_rows, int **src_mat) {

    // TODO: Stellen Sie sicher, dass jeder Gamer-Thread an dieser Stelle darauf
	// wartet, bis alle anderen Gamer-Threads ihre Ränder ausgelesen haben
  while (field->field_read != field->num_threads) {
  }


    // TODO: Stellen Sie danach sicher, dass jeder Gamer-Thread an dieser Stelle
	// solange wartet, bis die Ausgabe des Printer-Threads erfolgt


    for (int i = 0; i < num_rows; i++) {
      memcpy(field->field[row_start+i], src_mat[i], field->width*sizeof(int));
    }
    field->field_filled ++;
    while (field->field_filled != field->num_threads) {
    }

        // TODO: Zurücksetzen der Print-Bedingungen
    field->field_filled = 0;
    field->field_read = 0;
        // TODO: Warten Sie darauf, dass alle Gamer-Threads in das Spielfeld
		// geschrieben haben und starten Sie dann den Printer-Thread
    print_game(field);

	return 1;
}


void print_game_field(int GUI, SDL_Surface* screen, game_field *field, int** last_field){

  while (field->field_filled != field->num_threads) {
  }

    if (GUI == 1) {
        update_matrix(screen, field, last_field);
    } else {
        printf("----------------------------------------\n");
        for (int i = 0; i < field->height; i++) {
            for (int j = 0; j < field->width; j++) {
                int elem = field->field[i][j];
                if (elem == 1) {
                    printf("\x1B[31m%d ", elem);
                } else {
                    printf("\x1B[37m%d ", elem);
                }
            }
            printf("\n");
        }
        printf("----------------------------------------\n");
    }

    // TODO: Signalisieren Sie den anderen Threads, dass das Spielfeld ausgegeben wurde
}


void read_adjacent_lines(game_field *field, int height_above, int height_below, int **line_above, int **line_below) {
    memcpy(*line_above, field->field[height_above], field->width*sizeof(int));
    memcpy(*line_below, field->field[height_below], field->width*sizeof(int));

    // game_field->field_read and game_field->field_filled are made volatile int (atomic), so they don't need any mutex
      field->field_read ++;

    // Wait for all gamer threads to compute their part of the field
      while (field->field_read != field->num_threads) {
      }
}
