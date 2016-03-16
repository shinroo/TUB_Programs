#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include "input_blatt06.h"
/*
Diese Funktion implementiert den iterativen Mergesort Algorithmus auf einem Array.
Sie soll analog zum Pseudocode in Listing 4 implementiert werden.

    array:  Pointer auf den Beginn des Arrays
    first:  Index des ersten Elements
    len  :  Index des letzten Elements
*/

void merge(int* array, int start, int middle, int end)
{
// HIER Funktion merge() implementieren
}

void merge_sort(int* array, int first, int last)
{
 // HIER Funktion merge_sort() implementieren
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
    
    int len = read_array_from_file(array, atoi(argv[1]), filename);

    printf("Eingabe:\n");
    print_array(array, len);

    // HIER Aufruf von "merge_sort()"

    printf("Sortiert:\n");
    print_array(array, len);

    return 0;
}
