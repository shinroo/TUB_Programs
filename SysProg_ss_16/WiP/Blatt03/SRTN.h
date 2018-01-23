/*
 * Shortest Remaining Time Next scheduler.
 *
 * In this scheduler, the task with the shortest remaining time (length)
 * currently available is scheduled when another task finishes.
 *
 * Tasks with the same execution time are ordered according to their arrival.
 *
 * This scheduler is non preemptive, meaning the tick function
 * has no effect.
 *
 * If the arrive of new tasks happens to be at the same time
 * a running task finishes, it is guaranteed that the arrive
 * function is called first.
 */

#ifndef SRTN_H_
#define SRTN_H_

/*
 * Initialize data structures if needed.
 *
 * returns: 1 if this scheduler should be considered in the tests, 0 otherwise.
 */
int init_SRTN();

/*
 * Frees all allocated memory used.
 */
void free_SRTN();

/*
 * Is called when a new task arrives that has to be scheduled.
 *
 * id    : the id of the task
 * length: the time the task runs
 */
void arrive_SRTN(int id, int length);

/*
 * Is called when the next tick is reached.
 *
 * This function has no effect in this non preemptive SRTN scheduling.
 */
void tick_SRTN();

/*
 * Is called when the task with the specified id is finished.
 *
 * id: the id of the finished task
 */
void finish_SRTN(int id);

#endif /* SRTN_H_ */
