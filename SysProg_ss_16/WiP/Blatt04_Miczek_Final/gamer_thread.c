#include <stdio.h>
#include <stdbool.h>
#include "gamer_thread.h"
#include "utils.h"


// Prüft jede Zelle auf die 1. Regel des Game of Life
int ruleOne(int col, int row, int height, int **field_part, int* line_above, int*line_below, int o_width) {
    int count = 0;
    int result = 0;
    int help = (col == 0) ? o_width-1 : col-1;

    // Wir sind mitten im Thread-Bereich des Spielfelds
    if (row < height-1 && row > 0) {
        if (field_part[row][(col+1)%(o_width)] == 1) count++;
        if (field_part[row][help] == 1) count++;
        if (field_part[row+1][help] == 1) count++;
        if (field_part[row+1][col] == 1) count++;
        if (field_part[row+1][(col+1)%(o_width)] == 1) count++;
        if (field_part[row-1][help] == 1) count++;
        if (field_part[row-1][col] == 1) count++;
        if (field_part[row-1][(col+1)%(o_width)] == 1) count++;
        if (count == 3) {
			result = 1;
		} else result = 0;
    }else {
        // Wir sind am oberen Rand des Thread-Bereichs des Spielfelds
        if (row == 0) {
            if (field_part[row][(col+1)%(o_width)] == 1) count++;
            if (field_part[row][help] == 1) count++;
            if (field_part[row+1][help] == 1) count++;
            if (field_part[row+1][col] == 1) count++;
            if (field_part[row+1][(col+1)%(o_width)] == 1) count++;
            if (line_above[help] == 1) count++;
            if (line_above[col] == 1) count++;
            if (line_above[(col + 1) % (o_width)] == 1) count++;
            if (count == 3) {
                result = 1;
            } else result = 0;
        // Wir sind am unteren Rand des Thread-Bereichs des Spielfelds
        } else if (row == height-1) {
            if (field_part[row][(col+1)%(o_width)] == 1) count++;
            if (field_part[row][help] == 1) count++;
            if (line_below[help] == 1) count++;
            if (line_below[col] == 1) count++;
            if (line_below[(col + 1) % (o_width)] == 1) count++;
            if (field_part[row-1][help] == 1) count++;
            if (field_part[row-1][col] == 1) count++;
            if (field_part[row-1][(col+1)%(o_width)] == 1) count++;
            if (count == 3) {
				result = 1;
			} else result = 0;
        }
    }

    return result;
}


// Prüft jede Zelle auf die 2., 3. und 4. Regel des Game of Life
int ruleTwoThreeFour(int col, int row, int height, int **field_part, int*line_above, int*line_below, int o_width) {
    int count = 0;
    int result = 0;
    int help = (col == 0) ? o_width-1 : col-1;

    // Wir sind mitten im Thread-Bereich des Spielfelds
    if (row < height-1 && row > 0) {
        if (field_part[row][(col+1)%(o_width)] == 1) count++;
        if (field_part[row][help] == 1) count++;
        if (field_part[row+1][help] == 1) count++;
        if (field_part[row+1][col] == 1) count++;
        if (field_part[row+1][(col+1)%(o_width)] == 1) count++;
        if (field_part[row-1][help] == 1) count++;
        if (field_part[row-1][col] == 1) count++;
        if (field_part[row-1][(col+1)%(o_width)] == 1) count++;
        if ((count == 2) || (count == 3)) {
            result = 1;
        } else {
            result = 0;
        }
    } else {
        // Wir sind am oberen Rand des Thread-Bereichs des Spielfelds
        if (row == 0) {
            if (field_part[row][(col+1)%(o_width)] == 1) count++;
            if (field_part[row][help] == 1) count++;
            if (field_part[row+1][help] == 1) count++;
            if (field_part[row+1][col] == 1) count++;
            if (field_part[row+1][(col+1)%(o_width)] == 1) count++;
            if (line_above[help] == 1) count++;
            if (line_above[col] == 1) count++;
            if (line_above[(col + 1) % (o_width)] == 1) count++;
            if ((count == 2) || (count == 3)) {
                result = 1;
            } else result = 0;
        // Wir sind am unteren Rand des Thread-Bereichs des Spielfelds
        } else if (row == height-1) {
            if (field_part[row][(col+1)%(o_width)] == 1) count++;
            if (field_part[row][help] == 1) count++;
            if (line_below[help] == 1) count++;
            if (line_below[col] == 1) count++;
            if (line_below[(col + 1) % (o_width)] == 1) count++;
            if (field_part[row-1][help] == 1) count++;
            if (field_part[row-1][col] == 1) count++;
            if (field_part[row-1][(col+1)%(o_width)] == 1) count++;
            if ((count == 2) || (count == 3)) {
                result = 1;
            } else result = 0;
        }
    }

    return result;
}


void* play_game(void *args) {
    gamer_args* arg_struct = (gamer_args*) args;
  //  printf("6\n");
    int o_width = arg_struct->field->width;
  //  printf("width\n");
    int o_height = arg_struct->field->height;
//    printf("heigth\n");
    int width = o_width;
    // Berechnet die Größe des Teils vom Spielfeld
    int height = compute_my_part(o_height,arg_struct->id,arg_struct->field->num_threads);
    // Wo startet der Teil des jeweiligen Threads
    int my_start_offset = arg_struct->id * (o_height/arg_struct->field->num_threads);

    // Erzeuge zwei Matrizen, um die alte Generation zu speichern und die neue zu berechnen
    int **old_field = (int**) emalloc(height*sizeof(int *));
    int **new_field = (int**) emalloc(height*sizeof(int *));
    for (int i = 0; i < height; i++) {
        old_field[i] = (int*) emalloc(width*sizeof(int));
        new_field[i] = (int*) emalloc(width*sizeof(int));
    }
    // Kopiere das Startspielfeld, um neue Generation zu berechnen
    for (int i = 0; i < height; i++) {
        memcpy(old_field[i], arg_struct->field->field[my_start_offset+i], width*sizeof(int));
    }

    // Speicher für die Ränder des Spielfeldteils
    int* line_above = (int*) emalloc(sizeof(int)*width);
    int* line_below = (int*) emalloc(sizeof(int)*width);
    int line_above_off, line_below_off;
    while (true) {
        // Bestimme die Position der Ränder und lese sie ein
        if (arg_struct->field->num_threads > 1) {
            if (arg_struct->id == 0) {
                line_above_off = o_height - 1;
                line_below_off = height;
            } else if (arg_struct->id == arg_struct->field->num_threads - 1) {
                line_above_off = my_start_offset - 1;
                line_below_off = 0;
            } else {
                line_above_off = my_start_offset - 1;
                line_below_off = my_start_offset + height;
            }

            read_adjacent_lines(arg_struct->field, line_above_off, line_below_off, &line_above, &line_below);
        } else {
            line_above_off = o_height-1;
            line_below_off = 0;
            read_adjacent_lines(arg_struct->field, line_above_off, line_below_off, &line_above, &line_below);
        }

		// Berechne aus aktueller Generation un den Rändern die neue Generation
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                if (old_field[i][j] == 0) {
                    new_field[i][j] = ruleOne(j, i, height, old_field, line_above, line_below, o_width);
                } else {
                    new_field[i][j] = ruleTwoThreeFour(j, i, height, old_field, line_above, line_below, o_width);
                }
            }
        }
        // Vertausche die Matrix-Zeiger
        swap(&old_field, &new_field);
        // Schreibe eigenen Teil der neuen Generation in das Spielfeld
        write_to_field(arg_struct->field, my_start_offset, height, old_field);
    }
}
