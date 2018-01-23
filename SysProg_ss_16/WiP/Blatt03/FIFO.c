#include "FIFO.h"
#include "task.h"
#include <stdlib.h>
#include <stdio.h>
#include "Queue.h"

static Queue *q;
static int fifo_isRunning;
/*
 * Two modes of this function: run before the start (so initialisation needed - fifo_isRunning == 0)
 * Or normal mode - fifo_isRunning == 1. If the queue is not empty, return 1. If it's empty, return 0.
 */
int init_FIFO()
{
	if (!fifo_isRunning) {
    q = (Queue*)malloc(sizeof(Queue));
    q->size = 0;
    q->head = NULL;
    q->tail = NULL;
	fifo_isRunning = 1;
    return 0;
  }
  if (q == NULL)
	  return 0;
  else
	  return 1;

}
/*
 * Check, if the queue is really empty. If it is, frees the queue. If not, frees all q elements first and then q.
 */
void free_FIFO()
{
  if (q->head == NULL) {
    free(q);
    return;
  }
  while(q->head) {
    Element *el = queue_poll(q);
    free(el);
  }
  free(q);
}

void arrive_FIFO(int id, int length)
{
	def_task task;
	task.id = id;
	task.length = length;
	if (queue_size(q) == 0)
		running_task = task.id;
	queue_offer(q, task);
}

void tick_FIFO()
{

}

void finish_FIFO(int id)
{
  if (queue_peek(q) == NULL || q->head->task.id != id) {
    printf("Queue either empty, or the process with the given id is not the current head of the Queue!\n");
    return;
  }
  Element *el = queue_poll(q);
  free(el);
  if (queue_size(q))
	  switch_task(q->head->task.id);
  else {
	  switch_task(IDLE);
	  free_FIFO();
  }
}
