#include "blatt09.h"

/* Stellt die Heap Bedingung wieder her, wenn der linke und rechte Unterbaum die Heap
 * Bedingung schon erfüllen.
 */

/*

heapify was impelemented using the following algorithm:

Heapify(A,i)
1. l = left(i)
2. r = right(i)
3. if l <= heap-size[A] and A[l] > A[i]
    then largest = l
4.  else largest = i
5. if r <= heap-size[A] and A[r] > A[largest]
    then largest = r
6. if largest != i
    then A[i] = A[largest]
7. Heapify(A,largest)

*/

void heapify(heap* h, int i) {

  int left = (i+1)*2-1;
  int right = (i+1)*2;
  int largest;
  int temp;

  //find the largest
  if ((left <= h->size) && (h->array[left] > h->array[i])){
    largest = left;
  }
  else {
    largest = i;
  }
  if ((right < h->size) && (h->array[right] > h->array[largest])){
    largest = right;
  }

  //if the largest isn't the i'th element, adjust heap according to heap property
  if (largest != i){
    temp = h->array[i];
    h->array[i] = h->array[largest];
    h->array[largest] = temp;
    heapify(h,largest);
  }

}

/* Holt einen Wert aus dem Heap
 *
 * Wenn der Heap nicht leer ist, wird die größte Zahl zurückgegeben.
 * Wenn der Heap leer ist, wird -1 zurückgegeben.
 */
int heap_extract_max(heap* h) {

  //check for empty heap
  if (h->size < 1){
    return(-1);
  }

  //get max
  int max = h->array[0];

  //reduce heap size by 1 and adjust heap
  h->array[0] = h->array[h->size -1];
  h->size = h->size -1;

  //heapify adjusted heap
  heapify(h,0);

  return max;

}

/* Fügt einen Wert in den Heap ein
 *
 * Wenn der Heap bereits voll ist, wird -1 zurückgegeben,
 */
int heap_insert(heap* h, int key) {

  //check for full heap
  if(h->size >= MAX_HEAP_SIZE){
    return(-1);
  }

  h->size = h->size +1;

  int i = h->size-1;

  //adjust heap
  while ((i>0) && (h->array[(i+1)/2-1] < key)){
    h->array[i] = h->array[(i+1)/2-1];
    i = (i+1)/2-1;
  }

  //add new element to heap
  h->array[i] = key;

  return(0);
}

/* Lese die Eingabe von STDIN. Lese jeweils nur eine Zeile ein.
 * Gebe folgende Werte zurück
 *
 * Eine positive Zahl, wenn eine solche eingegeben wurde (z.b. "10").
 * -1 Bei der Eingabe von 'n'
 * -2 Bei der Eingabe von 'q'
 * -3 Wenn die Eingabe sich nicht eindeutig zuordnen ließ.
 *
 */
int read_user_input() {

  char* buffer = (char*) malloc (MAX_LINE_SIZE);
  char* extra = (char*) malloc (MAX_LINE_SIZE);

  //read line
  gets(buffer);

  int number;
  int check;

  //check if string is empty
  if (buffer == NULL){
    free(buffer);
    free(extra);
    return(-3);
  }

  //check if string is a number
  if (sscanf(buffer,"%d %s ",&check, extra) == 1){
      number = atoi(buffer);

      if (number > 0){
        free(buffer);
        free(extra);
        return(number);
      }
      else{
        free(buffer);
        free(extra);
        return(-3);
      }
  }
  //if string is not a number, check for recognized characters 'n' or 'q'
  else{
    if (strncmp(buffer,"n",strlen(buffer)) == 0) {
      free(buffer);
      free(extra);
      return(-1);
    }
    else if (strncmp(buffer,"q",strlen(buffer)) == 0) {
      free(buffer);
      free(extra);
      return(-2);
    }
    else{
      free(buffer);
      free(extra);
      return(-3);
    }
  }



}
