#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include "input_blatt06.h"
/*
Diese Funktion implementiert den iterativen Mergesort Algorithmus auf einem Array.
Sie soll analog zum Pseudocode in Listing 4 implementiert werden.

    array:  Pointer auf den Beginn des Arrays
    first:  Index des ersten Elements
    length: Anzahl der Elemente des Arrays
*/

void merge(int* array, int start, int middle, int end)
{
// HIER Funktion merge() implementieren
  //declare temporary array
  int* b = (int*) calloc (end-start+1,sizeof(int));

  //declare temporary variables
  int k = start;
  int m = middle + 1;
  int i = 0;

  //copy values into temp array
  while ((k <= middle) && (m <= end)){
    if (array[k] <= array[m]){
      b[i] = array[k];
      k++;
    }
    else{
      b[i] = array[m];
      m++;
    }
    i++;
  }

  //copy remaining items from the left
  while (k <= middle){
    b[i] = array[k];
    k++;
    i++;
  }

  //copy remaining items from the right
  while (m <= end){
    b[i] = array[m];
    m++;
    i++;
  }

  int j = 0;

  //copy values back to array
  while (j < i){
    array[start+j] = b[j];
    j++;
  }

  free(b);
}

void merge_sort(int* array, int first, int last)
{
 // HIER Funktion merge_sort() implementieren
  int step = 1;
  int left,middle,right;
  while (step <= last) {

    left = 0;
    right = 0;
    middle = 0;

    while (left <= last-step) {
        middle = left + step -1;

        //min(middle,last)
        if (middle > last){
            middle = last;
        }

        right = left + 2 * step - 1;

        //min(right,last)
        if (right > last){
            right = last;
        }

        merge(array,left,middle,right);

        left = left + 2*step;
    }
    step = step * 2;
  }
}

/*
Hauptprogramm.

Liest Integerwerte aus einer Datei und gibt
diese sortiert im selben Format über die Standardausgabe wieder aus.

Aufruf: ./introprog_blatt06_aufgabe01 <maximale anzahl>  <dateipfad>
*/
int main (int argc, char *argv[])
{
    if (argc!=3){
        printf ("usage: %s <maximale anzahl>  <dateipfad>\n", argv[0]);
        exit(2);
    }

    char *filename = argv[2];

    // Hier array initialisieren
    int* array = (int*) calloc (atoi(argv[1]),sizeof(int));

    int len = read_array_from_file(array, atoi(argv[1]), filename);

    printf("Eingabe:\n");
    print_array(array, len);

    // HIER Aufruf von "merge_sort()"
    merge_sort(array,0,len-1);

    printf("Sortiert:\n");
    print_array(array, len);

    return 0;
}
