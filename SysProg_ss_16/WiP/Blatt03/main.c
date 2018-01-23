#include <stdio.h>
#include <stdlib.h>
#include "FIFO.h"
#include "MLF.h"
#include "SRTN.h"
#include "task.h"

int main(int argc, char** argv)
{
	// just a very basic example to test the FIFO implementation

	printf("----FIFO----\n");
	init_FIFO();
	if (init_FIFO())
	{
		arrive_FIFO(4, 1);
		arrive_FIFO(0, 1);
		arrive_FIFO(3, 1);
		arrive_FIFO(1, 5);

		printf("%i\n", running_task);
		finish_FIFO(4);
		printf("%i\n", running_task);
		finish_FIFO(0);
		printf("%i\n", running_task);
		finish_FIFO(3);
		printf("%i\n", running_task);
		finish_FIFO(1);
		printf("%i\n", running_task);

	}

	printf("----SRTN----\n");
	if (!(init_SRTN()))
	{
		arrive_SRTN(1, 1);
		arrive_SRTN(2, 2);
		arrive_SRTN(3, 1);
		arrive_SRTN(4, 2);
		arrive_SRTN(5, 1);

		printf("%i\n", running_task);
		finish_SRTN(running_task);
		printf("%i\n", running_task);
		finish_SRTN(running_task);
		printf("%i\n", running_task);
		finish_SRTN(running_task);
		printf("%i\n", running_task);
		finish_SRTN(running_task);
		printf("%i\n", running_task);
		finish_SRTN(running_task);
	}

	printf("----MLF----\n");
	if(init_MLF(2, 4)){

		int size = 10;
		int taskList[size];
		for(int i = 0; i < size; ++i){
			taskList[i] = 0;
		}

		taskList[0] = 30;
		taskList[2] = 1;
		taskList[5] = 3;
		taskList[7] = 2;
		taskList[8] = 2;
		taskList[9] = 10;


		int numTicks = 0;
		for(int i = 0; i < size; ++i){
			numTicks += taskList[i];
		}

		for(int i = 0; i < numTicks; ++i){
			if(i < size && taskList[i] != 0){
				printf("Element arriving in task list!\n");
				arrive_MLF(i, taskList[i]);
			}
			tick_MLF();
		}
		free_MLF();
	}else{
		printf("Couldn't start MLF scheduler.\n");
	}

	return 0;
}
