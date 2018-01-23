#include "utils.h"
#include <stdio.h>
#include "config.h"

void* emalloc(size_t size) {
	void *buf = malloc(size);
	if (buf == NULL) {
		perror("memory allocation failed");
		exit(1);
	}
	return buf;
}


void swap (int*** m1, int*** m2)
{
	int** temp;
	temp = *m1;
	*m1 = *m2;
	*m2 = temp;
}


void set_glider_gun(int width, int height, int **field) {
	int col_coords[] = {1, 0, 1, 0, 10, 10, 10, 11, 11, 12, 12, 13, 13, 14, 15, 15, 16, 16, 16, 17, 20, 20, 20, 21, 21, 21, 22, 22, 24, 24, 24, 24, 35, 35, 34, 34};
	int row_coords[] = {24, 24, 25, 25, 25, 24, 26, 23, 27, 22, 28, 22, 28, 25, 23, 27, 25, 26, 24, 25, 22, 23, 24, 22, 23, 24, 21, 25, 21, 25, 20, 26, 23, 22, 23, 22};
	int array_size_col = sizeof(col_coords) / sizeof(col_coords[0]);
	int array_size_row = sizeof(row_coords) / sizeof(row_coords[0]);
	if (array_size_col != array_size_row) {
		printf("Error in Startset (x_coords != y_coords)");
		exit(1);
	}
	for (int i = 0; i < array_size_row; i++) {
		int row = row_coords[i] + ROW_OFFSET;
		int col = col_coords[i] + COL_OFFSET;
		if(row < 0 ||  row >= height || col < 0 || col >= width) {
			printf("Error in Startset at coordinat %d! Gamefield doesn't fit\n", i);
			exit(1);
		}
	}
	for (int i = 0; i < array_size_row; i++) {
		field[row_coords[i]][col_coords[i]] = 1;
	}
}


void set_glider(int width, int height, int **field) {
	int col_coords[] = {3, 5, 4, 5, 4};
	int row_coords[] = {4, 4, 5, 5, 6};
	int array_size_col = sizeof(col_coords) / sizeof(col_coords[0]);
	int array_size_row = sizeof(row_coords) / sizeof(row_coords[0]);
	if (array_size_col != array_size_row) {
		printf("Error in Startset (x_coords != y_coords)");
		fflush(stdout);
	}

	for (int i = 0; i < array_size_row; i++) {
		int row = row_coords[i] + ROW_OFFSET;
		int col = col_coords[i] + COL_OFFSET;
		if (row < 0 ||  row >= height || col < 0 || col >= width) {
			printf("Error in Startset at coordinat %d! Gamefield doesn't fit, row = %d, col = %d\n", i, row, col);
		}
	}
	for (int i = 0; i < array_size_row; i++) {
		field[row_coords[i]][col_coords[i]] = 1;
	}
}


int compute_my_part(int height,int id, int max_threads) {
	int part_size = height/max_threads;
	int start = id * part_size;
	int end;
	if (id != max_threads - 1) {
		end = start + part_size-1;
	} else {
		end = height-1;
	}
	return end+1 - start;
}
