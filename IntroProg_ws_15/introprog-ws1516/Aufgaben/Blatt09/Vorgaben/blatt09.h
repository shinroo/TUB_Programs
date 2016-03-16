#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <ctype.h>

#define MAX_HEAP_SIZE 400
#define MAX_LINE_SIZE 100

typedef struct heap heap;

/* Dieses struct definiert einen Heap

   Der Heap kann maximal MAX_HEAP_SIZE Elemente enthalten
*/
struct heap {
    int array[MAX_HEAP_SIZE];
    int size;
};

void heapify(heap* h, int i);
int heap_extract_max(heap* h);
int heap_insert(heap* h, int key);
int read_user_input();
