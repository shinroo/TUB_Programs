#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
typedef struct passwort passwort;

struct passwort{
    char name[100];
    int anzahl;
};

typedef struct list_element list_element;

struct list_element {
    passwort data;
    list_element* next;
};

typedef struct list list;

struct list {
    list_element* first;
    list_element* last;
};

void init_list( list* mylist );
void insert_front( list_element* le, list* mylist );
void free_list( list* mylist );
void read_data( char* filename, list* mylist );
list_element* partition( list* input, list* left, list* right );
void qsort_list( list* mylist );
void print_list( list* mylist );
