#include <stdio.h>
#include <stdlib.h>
#include "input_blatt01.h"

int MAX_LAENGE = 1000;
int MAX_VALUE = 100;

void count_sort_calculate_counts(int input_array[], int len, int count_array[]) {
    // Hier Funktion implementieren
    for (int j = 0; j < len; j++){
		count_array[input_array[j]] = count_array[input_array[j]] + 1; 
	}
}

void count_sort_write_output_array(int output_array[], int len, int count_array[]) {
    // Hier Funktion implementieren
    int k = 0;	

    for (int j = 0; j < MAX_VALUE; j++){
		for (int i = 0; i < count_array[j]; i++){
			output_array[k] = j;
			k++;		
		}		
	} 
}



int main(int argc, char *argv[]) {

    if (argc < 2){
        printf("Aufruf: %s <Dateiname>\n", argv[0]);
        printf("Beispiel: %s zahlen.txt\n", argv[0]);
        exit(1);
    }

    char *filename = argv[1];
    
    int input_array[MAX_LAENGE];   
    int len = read_array_from_file(input_array, MAX_LAENGE, filename);

    printf("Unsortiertes Array:");
    print_array(input_array, len);
    
    // HIER alle nötigen Deklarationen und Funktionsaufrufe für Count Sort einfügem

    int count_array[MAX_LAENGE];

    int output_array[MAX_LAENGE]; 

    count_sort_calculate_counts(input_array, len, count_array);
    count_sort_write_output_array(output_array, len, count_array);

    printf("Sortiertes Array:");
    print_array(output_array, len);

    return 0;
}
