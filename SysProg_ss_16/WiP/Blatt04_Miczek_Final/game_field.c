#include <string.h>
#include <stdio.h>
#include "game_field.h"
#include "utils.h"
#include "printer_thread.h"
#include <sys/time.h>


game_field* make_game_field(int width, int height, int num_threads) {

	// Creates gameField with width and height
	game_field* gameField = (game_field*)malloc(sizeof(game_field));
	gameField->field = malloc(sizeof(int*)*height);
	for (int i = 0; i < height; i++) {
		gameField->field[i] = malloc(sizeof(int)*width);
		memset(gameField->field[i], 0, sizeof(int)*width);
	}
	gameField->width = width;
	gameField->height = height;
	gameField->num_threads = num_threads;

	// Initializes the first generation
	set_glider_gun(width, height, gameField->field);


	// Initialize all needed mutex and conditional object, using
	// locks for safety.
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

	// Waits while all other Gamer-Threads have read their edges
	pthread_mutex_lock(&field->field_read_m);
	while (field->field_read > 0) {
		pthread_mutex_unlock(&field->field_read_m);
		pthread_mutex_lock(&field->field_read_m);
	}
	pthread_mutex_unlock(&field->field_read_m);

	// Wait for printer thread to finish work
	pthread_mutex_lock(&field->printed_m);
	while (field->printed != true) {
		pthread_mutex_unlock(&field->printed_m);
		pthread_mutex_lock(&field->printed_m);
	}
	pthread_mutex_unlock(&field->printed_m);
	
	// Fill field
	for (int i = 0; i < num_rows; i++) {
		memcpy(field->field[row_start+i], src_mat[i], field->width*sizeof(int));
	}
	pthread_mutex_lock(&field->field_filled_m);
	field->field_filled --;
	pthread_mutex_unlock(&field->field_filled_m);

	// Wait until field has been filled
	pthread_mutex_lock(&field->field_filled_m);
	while (field->field_filled > 0) {
		pthread_mutex_unlock(&field->field_filled_m);
		pthread_mutex_lock(&field->field_filled_m);
	}
	pthread_mutex_unlock(&field->field_filled_m);
	
	// Set print back to false
	pthread_mutex_lock(&field->printed_m);
	field->printed = false;
	pthread_mutex_unlock(&field->printed_m);

	return 1;
}


void print_game_field(int GUI, SDL_Surface* screen, game_field *field, int** last_field){

	// Wait until woken up
	pthread_mutex_lock(&field->printed_m);
	while (field->printed != false) {
		pthread_mutex_unlock(&field->printed_m);
		pthread_mutex_lock(&field->printed_m);
	}
	pthread_mutex_unlock(&field->printed_m);

	// Create graphical inteface if option has been set in config
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

	// Update field_filled and field_read counting variables
	pthread_mutex_lock(&field->field_filled_m);
	field->field_filled = field->num_threads;
	pthread_mutex_unlock(&field->field_filled_m);

	pthread_mutex_lock(&field->field_read_m);
	field->field_read = field->num_threads;
	pthread_mutex_unlock(&field->field_read_m);

	// Set printed to true
	pthread_mutex_lock(&field->printed_m);
	field->printed = true;
	pthread_mutex_unlock(&field->printed_m);
}


void read_adjacent_lines(game_field *field, int height_above, int height_below, int **line_above, int **line_below) {
	memcpy(*line_above, field->field[height_above], field->width*sizeof(int));
	memcpy(*line_below, field->field[height_below], field->width*sizeof(int));

	pthread_mutex_lock(&field->field_read_m);
	field->field_read --;
	pthread_mutex_unlock(&field->field_read_m);
}
