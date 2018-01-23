#include "task.h"

#ifndef QUEUE_H_
#define QUEUE_H_

typedef struct _Element
{
	def_task task;
	struct _Element *next;
} Element;

typedef struct _Queue
{
	int size;
	Element *head;
	Element *tail;
} Queue;


static int no_comparator(const void *a, const void *b)
{
	return -1;
}

Queue* queue_new(int (*comparator)(const void *a, const void *b));

void queue_free(Queue *queue);

void queue_offer(Queue *queue, def_task task);

void* queue_peek(Queue *queue);

void* queue_poll(Queue *queue);

int queue_size(Queue *queue);

#endif /* QUEUE_H_ */
