#ifndef QUEUE_H_
#define QUEUE_H_

typedef struct Queue Queue;

static int no_comparator(const void *a, const void *b)
{
	return -1;
}

Queue* queue_new(int (*comparator)(const void *a, const void *b));

void queue_free(Queue *queue);

void* queue_offer(Queue *queue, void* value);

void* queue_peek(Queue *queue);

void* queue_poll(Queue *queue);

int queue_size(Queue *queue);

#endif /* QUEUE_H_ */
