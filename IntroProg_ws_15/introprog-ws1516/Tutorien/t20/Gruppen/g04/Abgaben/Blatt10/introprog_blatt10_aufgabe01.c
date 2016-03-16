#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include "blatt10.h"

/*

 Die benoetigten structs findet ihr in blatt10.h

*/

void init_list(list* mylist)
{
// HIER Liste initialisieren
  mylist->first = NULL;
  mylist->last = NULL;
}


// Diese Funktion fügt Listenelemente am Anfang der Liste an
void insert_front(list_element* le, list* mylist)
{
    // HIER Code einfügen
    //check if list is empty
    if (mylist->last == NULL){
      le->next = NULL;
      mylist->first = le;
      mylist->last = le;
    }
    else {
      le->next = mylist->first;
      mylist->first = le;
    }
}

// Speicher für Listenelemente wieder freigeben
void free_list(list* mylist)
{
    // HIER Code einfügen
    list_element* current = mylist->first;
    list_element* temp;
    while (current != NULL) {
      temp = current;
      current = current->next;
      free(temp);
    }
}


// Namen, Zahlen Paare in Liste einlesen
void read_data(char* filename, list* mylist)
{
		// HIER Code einfügen:
        // * Speicher allozieren
        // * Daten in list_element einlesen
        // * insert_front benutzen um list_element in Liste einzufügen

    //open file
    FILE *fp = NULL;
    fp = fopen(filename,"r");
    if (fp == NULL){
      perror(filename);
      exit(1);
    }

    int MAX_LEN_LINE = 1000;

    char line[MAX_LEN_LINE];
    char name[100];
    int anzahl;

    //read and process every line of the file
    while(fgets(line,MAX_LEN_LINE,fp)){
      if (sscanf(line, " %s %d ",name,&anzahl) == 2){
        list_element *new = (list_element*) malloc (sizeof(list_element));
        //since we don't have access to strncpy, use for loop to copy string
        for (int i=0; i < 100; i++){
          new->data.name[i] = name[i];
        }
        new->data.anzahl = anzahl;
        insert_front(new,mylist);
      }
      else {
        fprintf(stderr, "Incorrect password format\n");
      }
    }

    fclose(fp);
}

list_element* partition( list* input, list* left, list* right )
{
    // HIER Code einfügen:
    // parition() Funktion implementieren
    list_element* pivot = input->first;
    list_element* current = input->first->next;
    list_element* temp;

    //split list into sublists, left and right
    while (current != NULL){
      temp = current;
      if (current->data.anzahl >= pivot->data.anzahl) {
        current = current->next;
        insert_front(temp, right);
      }
      else {
        current = current->next;
        insert_front(temp,left);
      }
    }

    return pivot;
}

// Hauptfunktion des quicksort Algorithmus
void qsort_list(list* mylist)
{
    // HIER Code einfügen

    if (mylist->first != mylist->last){

      list left;
      list right;

      init_list(&left);
      init_list(&right);

      list_element* pivot = partition(mylist,&left,&right);
      qsort_list(&left);
      qsort_list(&right);

      if (left.first == NULL){
        left.first = pivot;
        left.last = pivot;
      }
      else {
        left.last->next = pivot;
      }

      mylist->first = left.first;
      pivot->next = right.first;

      if(right.first == NULL){
        mylist->last = pivot;
      }
      else{
        mylist->last = right.last;
      }
    }
}

// Liste ausgeben
void print_list(list* mylist)
{
    // HIER Code einfügen:
    // * Laufe über die list_element in mylist und gebe sie aus.
    list_element* current;

    current = mylist->first;

    while(current != NULL){
        printf("%s %d\n",current->data.name,current->data.anzahl);
        current = current->next;
    }

}
