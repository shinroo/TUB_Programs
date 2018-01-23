/*
 * FIFO scheduler.
 *
 * Schedules task in the order of their arrival.
 * 
 * This scheduler is non preemptive, meaning the tick function
 * has no effect.
 *
 * If the arrive of new tasks happens to be at the same time
 * a running task finishes, it is guaranteed that the arrive
 * function is called first.
 */

#ifndef FIFO_H_
#define FIFO_H_

/*
 * Initialize data structures if needed.
 *
 * returns: 1 if this scheduler should be considered in the tests, 0 otherwise.
 */
int init_FIFO();

/*
 * Frees all allocated memory used.
 */
void free_FIFO();

/*
 * Is called when a new task arrives that has to be scheduled.
 *
 * id    : the id of the task
 * length: the time the task runs
 */
void arrive_FIFO(int id, int length);

/*
 * Is called when the next tick is reached.
 *
 * This function has no effect in this non preemptive FIFO scheduling.
 */
void tick_FIFO();

/*
 * Is called when the task with the specified id is finished.
 *
 * id: the id of the finished task
 */
void finish_FIFO(int id);

#endif /* FIFO_H_ */
