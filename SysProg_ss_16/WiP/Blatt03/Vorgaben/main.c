#include <stdio.h>
#include <stdlib.h>

#include "FIFO.h"
#include "task.h"

int main(int argc, char** argv)
{
	// just a very basic example to test the FIFO implementation
	if (init_FIFO())
	{
		arrive_FIFO(0, 1);
		arrive_FIFO(1, 5);

		printf("%i\n", running_task);
		finish_FIFO(0);
		printf("%i\n", running_task);
		finish_FIFO(1);
		printf("%i\n", running_task);
	}

	return 0;
}
