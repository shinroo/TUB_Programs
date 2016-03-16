#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <string.h>

//initialisiert den Zufallszahlengenerator mit einem fixen Wert
void initialize_random_generator(){
    //Veranstaltungsnummer von Einführung in die Programmierung
    srand(0432205);
}

//befüllt die ersten len Elemente des arrays array mit Werten im Bereich
//0 bis max_value (ausschließlich)
void fill_array_randomly(int array[], int len, int max_value)
{
    for(int i=0; i < len; ++i)
    {
        array[i] = rand() % max_value;
    }
}


// Gibt das Array nach dem gewünschten Format aus.
void print_array(int array[], int len) {
    for (int i = 0; i < len ; i++) {
        printf(" %d", array[i]);
    }
    printf("\n");
}

//überprüft -- mittels memcmp -- ob die zwei übergebenen Arrays
//in den ersten len Einträgen übereinstimmen
int check_equality_of_arrays(int* array_1, int* array_2, int len)
{
    return(memcmp(array_1, array_2, len* sizeof(int)) == 0);
}

//kopiert die ersten len Einträge -- mittels memcpy -- des Arrays source
//in das Array destination
void copy_array_elements(int* destination, int* source, int len)
{
    memcpy(destination, source, len*sizeof(int));
}

//globale Variable, welche zum Speichern des Startzeitpunktes dient
long _start_timer = -1;

//speichert die aktuelle Uhrzeit 
void start_timer() {
   _start_timer  = clock();
}

int get_value_one()
{
    int sum = 0;
    for(int i = 0; i < 500001; ++i)
    {
        sum = (sum+1)%2;
    }
    return sum;
}

//gibt die Zeit in Sekunden zurück, die seit dem Aufruf der Funktion
//start_timer vergangen sind
double end_timer() {
    if(_start_timer == -1)
    {
        printf("Es muss zuerst start_timer() aufgerufen werden!\n");
    }
    double elapsed_time = (double) (clock() - _start_timer) / ((CLOCKS_PER_SEC) / 1000);
    _start_timer = -1;
   return elapsed_time;
}

