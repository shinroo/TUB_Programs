#include <string.h>
#include <stdio.h>
#include "game_field.h"
#include "utils.h"
#include "printer_thread.h"
#include <sys/time.h>


game_field* make_game_field(int width, int height, int num_threads) {

    // TODO: Erzeugen Sie ein Spielfeld mit der Breite width und der Höhe height
    game_field* gameField = (game_field*)malloc(sizeof(game_field));
    gameField->field = malloc(sizeof(int*)*height);
    for (int i = 0; i < height; i++) {
      gameField->field[i] = malloc(sizeof(int)*width);
      memset(gameField->field[i], 0, sizeof(int)*width);
    }

    gameField->width = width;
    gameField->height = height;
    gameField->num_threads = num_threads;
    // TODO: Initialisieren Sie die erste Generation (Nutzen Sie hierfür die
	// Funktionen set_glider() und/oder set_glider_gun() aus utils.c)

  set_glider(width, height, gameField->field);


    // TODO: Initialisieren Sie alle notwendigen Mutex- und Cond-Objekte sowie
	// weitere Struct-Komponenten (TIPP: Da set_glider/set_glider_gun so tun,
	// als ob die Threads das Spielfeld gefüllt hätten, intialisieren Sie hier
	// field_filled mit der Anzahl der verwendeten Gamer-Threads)
  pthread_mutex_lock(&gameField->field_filled_m);
  gameField->field_filled = gameField->num_threads;
  pthread_mutex_unlock(&gameField->field_filled_m);
  pthread_mutex_lock(&gameField->field_read_m);
  gameField->field_read = gameField->num_threads;
  pthread_mutex_unlock(&gameField->field_read_m);
  pthread_mutex_lock(&gameField->printed_m);
  gameField->printed = false;
  pthread_mutex_unlock(&gameField->printed_m);
  return gameField;

}

int write_to_field(game_field *field, int row_start, int num_rows, int **src_mat) {

    // TODO: Stellen Sie sicher, dass jeder Gamer-Thread an dieser Stelle darauf
	// wartet, bis alle anderen Gamer-Threads ihre Ränder ausgelesen haben
  //printf("gamer: waiting for file read %d\n", field->field_read);
  pthread_mutex_lock(&field->field_read_m);
  while (field->field_read > 0) {
    pthread_mutex_unlock(&field->field_read_m);
    pthread_mutex_lock(&field->field_read_m);
  }
  pthread_mutex_unlock(&field->field_read_m);
  //printf("gamer: file read %d\n", field->field_read);

    // TODO: Stellen Sie danach sicher, dass jeder Gamer-Thread an dieser Stelle
	// solange wartet, bis die Ausgabe des Printer-Threads erfolgt
  //printf("gamer: waiting for printed %d\n", field->printed);
  pthread_mutex_lock(&field->printed_m);
  while (field->printed != true) {
    pthread_mutex_unlock(&field->printed_m);
    pthread_mutex_lock(&field->printed_m);
  }
  pthread_mutex_unlock(&field->printed_m);
  //printf("gamer: printed %d\n", field->printed);
    for (int i = 0; i < num_rows; i++) {
      memcpy(field->field[row_start+i], src_mat[i], field->width*sizeof(int));
    }
    pthread_mutex_lock(&field->field_filled_m);
    field->field_filled --;
    pthread_mutex_unlock(&field->field_filled_m);

    //printf("gamer: waiting for filled %d\n", field->field_filled);
    pthread_mutex_lock(&field->field_filled_m);
    while (field->field_filled > 0) {
      pthread_mutex_unlock(&field->field_filled_m);
      pthread_mutex_lock(&field->field_filled_m);
    }
    pthread_mutex_unlock(&field->field_filled_m);
    //printf("gamer: filled %d\n", field->field_filled);
    // Zurücksetzen der Print-Bedingungen
    pthread_mutex_lock(&field->printed_m);
    field->printed = false;
    pthread_mutex_unlock(&field->printed_m);
    //  Warten Sie darauf, dass alle Gamer-Threads in das Spielfeld
		// geschrieben haben und starten Sie dann den Printer-Thread


	return 1;
}


void print_game_field(int GUI, SDL_Surface* screen, game_field *field, int** last_field){
  // wait until being woken up
  //printf("printer: waiting for printed %d\n", field->printed);
  pthread_mutex_lock(&field->printed_m);
  while (field->printed != false) {
    pthread_mutex_unlock(&field->printed_m);
    pthread_mutex_lock(&field->printed_m);
  }
  pthread_mutex_unlock(&field->printed_m);
  //printf("printer: %d\n", field->printed);

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

    // Signalisieren Sie den anderen Threads, dass das Spielfeld ausgegeben wurde


    pthread_mutex_lock(&field->field_filled_m);
    field->field_filled = field->num_threads;
    pthread_mutex_unlock(&field->field_filled_m);

    pthread_mutex_lock(&field->field_read_m);
    field->field_read = field->num_threads;
    pthread_mutex_unlock(&field->field_read_m);

    pthread_mutex_lock(&field->printed_m);
    field->printed = true;
    pthread_mutex_unlock(&field->printed_m);
    //printf("printer: Signalised.\n" );
}


void read_adjacent_lines(game_field *field, int height_above, int height_below, int **line_above, int **line_below) {
    memcpy(*line_above, field->field[height_above], field->width*sizeof(int));
    memcpy(*line_below, field->field[height_below], field->width*sizeof(int));
    pthread_mutex_lock(&field->field_read_m);
      field->field_read --;
    pthread_mutex_unlock(&field->field_read_m);
    //  printf("read adjacent lines: field->field_read %d \n", field->field_read);
    // Wait for all gamer threads to compute their part of the field
}
