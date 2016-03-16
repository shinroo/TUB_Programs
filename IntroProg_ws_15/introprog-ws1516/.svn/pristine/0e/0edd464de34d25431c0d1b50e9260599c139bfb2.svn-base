#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "input_blatt03.h"

/* Bewirkt, dass statt 'struct _element' auch 'element' verwendet werden kann. */
typedef struct _element element; 
/* Bewirkt, dass statt 'struct _list' auch 'list' verwendet werden kann. */
typedef struct _list list;

struct _list {      /* Separater Wurzelknoten */
    element *first; /* Anfang/Kopf der Liste */
    int count;      /* Anzahl der Elemente */
};

/* HIER struct element implementieren. */

struct _element {
	char title[255];
	char author[255];
	int year;
	long long int isbn;
	element* next;
};


/* Fuege ein Element in die Liste ein, sodass die Liste aufsteigend nach ISBN
 * sortiert ist.
 * Dafür muss das erste Element ermittelt werden, dass in der
 * bisher sortierten Liste eine ISBN besitzt die größer ist als die des neuen
 * Elements. Wenn die Liste leer ist soll das Element direkt an den Anfang
 * platziert werden.
 *
 * first      - Erstes Element (bzw. Anfang) der Liste
 * new_elem   - Neues Element das in die Liste eingefuegt werden soll.
 *
 * Gib einen Pointer auf den neuen oder alten Anfang der Liste zurueck.
 */
element* insert_sorted(element *first, element *new_elem) {
    /* HIER implementieren. */

	element* reference;
	
	//check if new_elem should replace first as the root of the list
	if ((first == NULL) || (first->isbn >= new_elem->isbn))  {
		//insert new_elem
		new_elem->next = first;
		return new_elem; //if this code is executed it means that new_elem will become the new root of the list		
	}
	//if first is still the root of the list
	else {
		reference = first;

		//find the list element, after which new_elem should be added
		while ((reference->next != NULL) && (reference->next->isbn < new_elem->isbn)){
		
			reference = reference->next;
	
		} 		
		
		//insert new_elem
		new_elem->next = reference->next;
		reference->next = new_elem;
		return first; //if this code is executed it means that first is still the root of the list
	}
}

/* Kreiere ein neues Element mit dynamischem Speicher.
 *
 * title     - Der Titel des Buches
 * author    - Autor des Buches
 * year      - Erscheinungsjahr des Buches
 * isbn      - ISBN des Buches
 *
 * Gib einen Pointer auf das neue Element zurueck.
 */
element *construct_element(char *title, char* author, int year, long long int isbn) {
    /* HIER implementieren. */
	
	element* newelem = (element*) malloc(sizeof(element));
	
	strncpy(newelem->title,title,MAX_STR);
	strncpy(newelem->author,author,MAX_STR);
	newelem->year = year;
	newelem->isbn = isbn;

	return newelem;

}

/* Gib den der Liste und all ihrer Elemente zugewiesenen Speicher frei. */
void free_list(list *alist) {
    /* HIER implementieren. */

	element* current = alist->first;
	int count = alist->count;
	element* next;

	while (count > 0) {
	
		next = current->next;
		free(current);
		current = next;
		count--;
	
	}
	
	free(alist);	

}

/* Lese die Datei ein und fuege neue Elemente in die Liste ein 
 * _Soll nicht angepasst werden_
 * */
void read_list(char* filename, list *alist) {
    element* new_elem;

    char title[MAX_STR];
    char author[MAX_STR];
    int year;
    long long int isbn;
    while(read_line(filename, title, author, &year, &isbn) == 0) {
        new_elem = construct_element(title, author, year, isbn);
        alist->first = insert_sorted(alist->first, new_elem);
        alist->count++;
    }
}

/* Erstelle die Liste:
 *  - Weise ihr dynamischen Speicher zu 
 *  - Initialisiere die enthaltenen Variablen
 * _Soll nicht angepasst werden_
 */
list* construct_list() {
    list *alist = malloc(sizeof(list));
    alist->first = NULL;
    alist->count = 0;
    return alist;
}

/* Gib die Liste aus:
 * _Soll nicht angepasst werden_
 */
void print_list(list *alist) {
    printf("Meine Bibliothek\n================\n\n");
    int counter = 1;
    element *elem = alist->first;
    while (elem != NULL) {
        printf("Buch %d\n", counter);
        printf("\tTitel: %s\n", elem->title);
        printf("\tAutor: %s\n", elem->author);
        printf("\tJahr:  %d\n", elem->year);
        printf("\tISBN:  %lld\n", elem->isbn);
        elem = elem->next;
        counter++;
    }
}

/* Main Funktion
 * _Soll nicht angepasst werden_
 */
int main() {
    list *alist = construct_list();
    read_list("buecherliste.txt", alist);
    print_list(alist);
    free_list(alist);
    return 0;
}
