#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "input3.h"

int main(int argc, char **argv) {
    // lege Speicher an, um die Pointer zu den einzelnen Zeilen zu speichern
    char* string_array[5];

    // reserviere den Speicher für den Inhalt der Zeilen statisch
    // dies sollte in Aufgabe 2 dynamisch mittels malloc geschehen
    char erste_zeile[50];
    char zweite_zeile[50];
    char dritte_zeile[50];
	
    // beschreibe den Speicher mit entsprechenden Informationen
    sprintf(erste_zeile, "Erste Stadt");
    sprintf(zweite_zeile, "Zweite Stadt");
    sprintf(dritte_zeile, "Dritte Stadt");

    // lege die Pointer im Array ab
    string_array[0] = erste_zeile;
    string_array[1] = zweite_zeile;
    string_array[2] = dritte_zeile;

    // schreibe die Datei mit den entsprechenden 3 Zeilen
    write_file(string_array, 3);
}
