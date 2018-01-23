#include "SRTN.h"
#include "task.h"
#include <stdlib.h>
#include <stdio.h>
#include "Queue.h"


//Hilfsfunktion, Nutzung optional, aber empfohlen
/*
int length_comparator(const void *a, const void *b)
{
	return ((def_task*) a)->length - ((def_task*) b)->length;
}
*/

static Queue *q;
static int srtn_isRunning;

int init_SRTN()
{
	if (!srtn_isRunning) {
    q = (Queue*)malloc(sizeof(Queue));
    q->size = 0;
    q->head = NULL;
    q->tail = NULL;
		srtn_isRunning = 1;
    return 0;
  }
  if (q == NULL)
	  return 0;
  else
	  return 1;
}

void free_SRTN()
{
	if (q->head == NULL) {
    free(q);
    return;
  }
  while(q->head) {
    Element *el = queue_poll(q);
		printf("freeing %d\n",el->task.id);
    free(el);
  }
  free(q);
}

void arrive_SRTN(int id, int length)
{
	def_task task;
	task.id = id;
	task.length = length;
	if (queue_size(q) == 0)
		running_task = task.id;
	queue_offer(q,task);

	//TODO: sort
	def_task tasks[q->size];
	Element* temp;
	temp = q->head;
	int counter = 0;
	//copy tasks to array
	while(temp != NULL){
		tasks[counter] = temp->task;
		temp = temp->next;
		counter++;
	}
	def_task t_temp;
	//bubble sort array of tasks
	for (int c = 0 ; c < ( q->size - 1 ); c++)
  {
    for (int d = 0 ; d < q->size - c - 1; d++)
    {
      if (tasks[d].length > tasks[d+1].length) /* For decreasing order use < */
      {
        t_temp = tasks[d];
        tasks[d] = tasks[d+1];
        tasks[d+1] = t_temp;
      }
    }
  }
	temp = q->head;
	counter = 0;
	//copy tasks back
	while(temp != NULL){
		temp->task = tasks[counter];
		temp = temp->next;
		counter++;
	}
	running_task = q->head->task.id;
}

void finish_SRTN(int id)
{
	if (queue_peek(q) == NULL || q->head->task.id != id) {
		printf("Queue either empty, or the process with the given id is not the current head of the Queue!\n");
		return;
	}
	Element *el = queue_poll(q);
	free(el);
	if (queue_size(q)){
		switch_task(q->head->task.id);
	}else{
		switch_task(IDLE);
		free_SRTN();
	}
}
