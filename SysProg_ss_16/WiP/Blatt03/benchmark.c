
#include "benchmark.h"
#include <stdlib.h>

void createTasks()
{
	srand(1234);
	int i;
	for (i = 0; i < NUM_TASKS; i++)
	{
		def_task t;
		t.id = i; t.length = rand() % 100; //t.inserted = i;
		tasks[i] = t;
	}
}