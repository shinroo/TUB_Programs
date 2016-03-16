#include <stdio.h>
#include <stdlib.h>
#include "input_blatt02.h"

int MAX_VALUE = 5000000;

void count_sort_calculate_counts(int input_array[], int len, int count_array[], int* befehle) {
	(*befehle)++; // Zuweisung (i)
	for (int j = 0; j < len; j++){
		(*befehle)++; // Vergleich
		count_array[input_array[j]] = count_array[input_array[j]] + 1; 
		(*befehle)++; // Zuweisung (count_array)
		(*befehle)++; // Vergrößerung des Wertes (j)
	}

}

void count_sort_write_output_array(int output_array[], int len, int count_array[], int* befehle) {
	int k = 0;	
	(*befehle)++; // Zuweisung (k)
	(*befehle)++; // Zuweisung (j)
    	for (int j = 0; j < MAX_VALUE; j++){
		(*befehle)++; // Vergleich
		(*befehle)++; // Zuweisung (i)
		for (int i = 0; i < count_array[j]; i++){
			(*befehle)++; // Vergleich
			output_array[k] = j;
			(*befehle)++; // Zuweisung (output_array)
			k++;		
			(*befehle)++; // Vergrößerung des Wertes (k)
			(*befehle)++; // Vergrößerung des Wertes (i)
		}		
		(*befehle)++; // Vergrößerung des Wertes (j)
	} 
}

void count_sort(int array[], int len, int* befehle) {
	//erzeuge count array
	int* count_array = (int*) malloc (MAX_VALUE * sizeof(int));

	//init count array
	(*befehle)++; // Zuweisung (i)	
	for(int i = 0; i < MAX_VALUE; i++){
		(*befehle)++; // Vergleich
		count_array[i] = 0;
		(*befehle)++; // Zuweisung (count_array)
		(*befehle)++; // Vergrößerung des Wertes (i)	
	}

	//rufe count_sort Funktionen auf
	count_sort_calculate_counts(array,len,count_array,befehle);
	(*befehle)++; // Funktionsaufruf
	count_sort_write_output_array(array,len,count_array,befehle);
	(*befehle)++; // Funktionsaufruf
	
	//gib den Speicherplatz wieder frei
	free(count_array);


}


void insertion_sort(int array[], int len, int* befehle) {
	int i,j,key;

	(*befehle)++; // Zuweisung (j)	
	for (j = 1; j<len;j++){
		(*befehle)++; // Vergleich
		key = array[j];
		(*befehle)++; // Zuweisung (key)
		i = j-1;
		(*befehle)++; // Zuweisung (i)
		while ((i > -1) && (array[i] > key)) {
			(*befehle)++; // Vergleich
			array[i+1] = array[i];
			(*befehle)++; // Zuweisung (array)
			i--;
			(*befehle)++; //Verniederung des Wertes (i)
		}	
	
		array[i+1] = key;
		(*befehle)++; // Zuweisung (array)
		(*befehle)++; // Vergrößerung des Wertes (j)
	}

}

int WERTE[] = {10000,20000,30000,40000,50000};
int LEN_WERTE = 5;
int LEN_ALGORITHMEN = 2;

int main(int argc, char *argv[]) {

    long befehle_array[LEN_ALGORITHMEN][LEN_WERTE];
    double laufzeit_array[LEN_ALGORITHMEN][LEN_WERTE];

    for(int j = 0; j < LEN_WERTE; ++j)
    {
        int n = WERTE[j];
        
        //lege Arrays der Länge n an
        int* array_countsort = malloc(sizeof(int) * n);
        int* array_insertionsort = malloc(sizeof(int) * n);
        
        //fülle array_countsort mit Zufallswerten ..
        fill_array_randomly(array_countsort, n, MAX_VALUE);
        //.. und kopiere die erzeugten Werte in das Array array_insertionsort
        copy_array_elements(array_insertionsort, array_countsort, n);
        
        //teste ob beide Arrays auch wirklich die gleichen Werte enthalten
        if(!check_equality_of_arrays(array_countsort, array_insertionsort, n))
        {
            printf("Die Eingaben für beide Algorithmen müssen für die Vergleichbarkeit gleich sein!\n");
            return -1;
        }
        
        for(int i = 0; i < LEN_ALGORITHMEN; ++i)
        {
            int anzahl_befehle = 0;
            
            start_timer();
        
            //Aufruf der entsprechenden Sortieralgorithmen
            if(i==0)
            {
                    count_sort(array_countsort, n, &anzahl_befehle);
            }
            else if(i==1)
            {
                    insertion_sort(array_insertionsort, n, &anzahl_befehle);
            }
            
            //speichere die Laufzeit sowie die Anzahl benötigter Befehle
            laufzeit_array[i][j] = end_timer();
            befehle_array[i][j] = anzahl_befehle;
        }
        
        //teste ob die Ausgabe beider Algorithmen gleich ist
        if(!check_equality_of_arrays(array_countsort, array_insertionsort, n))
        {
            printf("Die Arrays sind nicht gleich. Eines muss (falsch) sortiert worden sein!\n");
            return -1;
        }
        
        //gib den Speicherplatz wieder frei
        free(array_countsort);
        free(array_insertionsort);
    }
    
    //Ausgabe der Anzahl ausgeführter Befehle sowie der gemessenen Laufzeiten (in Millisekunden)
    printf("Parameter MAX_VALUE hat den Wert %d\n", MAX_VALUE);
    printf("\t %32s           %32s \n", "Countsort","Insertionsort");
    printf("%8s \t %16s %16s \t %16s %16s \n", "n","Befehle", "Laufzeit","Befehle.","Laufzeit");
    
    for(int j = 0; j < LEN_WERTE; ++j)
    {
        printf("%8d \t ",WERTE[j]);
        for(int i = 0; i < LEN_ALGORITHMEN; ++i)
        {
            printf("%16ld %16.4f \t ",  befehle_array[i][j], laufzeit_array[i][j]);
        }            
        printf("\n");
    }
    
    return 0;
}
