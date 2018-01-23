#include "SRTN.h"
#include "task.h"
#include <stdlib.h>
#include <stdio.h>


//Hilfsfunktion, Nutzung optional, aber empfohlen
int length_comparator(const void *a, const void *b)
{
	return ((def_task*) a)->length - ((def_task*) b)->length;
}

static int srtn_isRunning;

int init_SRTN()
{

}

void free_SRTN()
{

}

void arrive_SRTN(int id, int length)
{

}

void tick_SRTN()
{

}

void finish_SRTN(int id)
{

}
