#include "MLF.h"
#include "task.h"
#include <stdlib.h>
#include <stdio.h>
#include "Queue.h"

// Vorgabe nicht lauffähig, eine Queue muss selbst geschrieben und implementiert werden.
Queue **q;

//Hilfsvarablen, Nutzung optional aber empfohlen.
static int step;
static int num;

static int used;

static def_task *task;
static Queue *task_queue;

static int task_queue_idx;

// Hilfsfunktionen, an denen sich orientiert werden kann. Sie können auch genutzt werden.

int smallestReadyQueue()
{
	int i;
	for (i = 0; i < num; i++)
	{
		if (queue_size(q[i]) > 0)
		{
			return i;	
		} 
	}
	return -1;
}

void schedule_task()
{
	task_queue_idx = smallestReadyQueue();

	if (task_queue_idx != -1)
	{
		task_queue = q[task_queue_idx];
		task = (def_task*) queue_poll(task_queue);
		switch_task(task->id);
	}
	else
	{
		switch_task(IDLE);
		task->id = -1;
		task_queue = NULL;
	}
}
// Ende Hilfsfunktionen

int init_Queue(){

	q = (Queue**) malloc (sizeof(Queue*)*num);

	for(int i = 0; i < num; ++i){
		Queue *newQueue = (Queue*) malloc (sizeof(Queue));
		newQueue->size = 0;
		newQueue->head = NULL;
		newQueue->tail = NULL;
		q[i] =  newQueue;
	}
	if(q == NULL)
		return 0;
	else
		return 1;
}

int init_MLF(int time_step, int num_queues)
{
	step = time_step;
	num = num_queues;
	int retVal = init_Queue();
	task = (def_task*) malloc (sizeof(def_task));
	task->id = -1;
	return retVal;
}

void free_Queue(Queue* queue){
	while(queue->head){
		Element *el = queue_poll(queue);
		free(el);
	}
}

void free_MLF()
{
	for( int i = 0; i < num; ++i){
		free_Queue(q[i]);	
		free(q[i]);
	}
	free(q);
}

void arrive_MLF(int id, int length)
{
	def_task newTask;
	newTask.id = id;
	newTask.length = length;
	if(task->id == -1){
		task->length = length;
		task->id = id;
		switch_task(task->id);
		used = step;
	}else{
		queue_offer(q[0], newTask);
	}
}
void tick_MLF()
{
	printf("---------TICK TOCK-------\n");
	printf("Task id(%d), length(%d), used(%d)\n", task->id, task->length, used);

	// BEGIN
	if(task->id == -1){
		schedule_task();
		if(task_queue_idx < num-1){
			used = (task_queue_idx+1) * step;
		}else{
			used = task->length;
		}
	printf("New task is: id(%d), length(%d), used(%d)\n", task->id, task->length, used);
	}
	// END
	used--;
	if(task->id != -1){
		task->length--;
		if(task->length == 0){
			task->id = -1;
		}
	}
	if(task->id != -1 && used==0){
		queue_offer(q[task_queue_idx+1], *task);
		task->id = -1;
	}
}
