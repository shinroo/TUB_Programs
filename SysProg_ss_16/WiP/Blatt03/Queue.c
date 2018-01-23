#include "Queue.h"
#include  <stdio.h>
#include <stdlib.h>

void queue_free(Queue *queue)
{
  if (queue_size(queue) == 0) {
    free(queue);
  }
};

void queue_offer(Queue *queue, def_task task)
{
  Element *new_el = (Element*)malloc(sizeof(Element));
  new_el->task = task;
  new_el->next = NULL;
  queue->size++;
  if (queue->head == NULL) {
    queue->head = new_el;
    queue->tail = new_el;
    return;
  }
  queue->tail->next = new_el;
  queue->tail = new_el;
};

void* queue_peek(Queue *queue)
{
  if (queue->size == 0 || queue->head == NULL)
    return NULL;
  return queue->head;
};

void* queue_poll(Queue *queue)
{
  if (queue->size == 0 || queue->head == NULL)
    return NULL;

  // if we are going to remove the last element, set the pointer last to NULL
  if (queue->size == 1)
    queue->tail = NULL;

  Element *el = queue->head;
  queue->head = el->next;
  queue->size--;
  return el;
};

int queue_size(Queue *queue)
{
  return queue->size;
};
