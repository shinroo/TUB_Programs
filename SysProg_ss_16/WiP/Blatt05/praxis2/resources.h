#ifndef INC_RESOURCES_H
#define INC_RESOURCES_H

#include <pthread.h>

/* set of resources */
enum {FIRST_RESOURCE, BAGGER=FIRST_RESOURCE, KRAN, BETONMISCHER, LASTWAGEN, NUM_RESOURCES}; 
/* set of threads */
enum {FIRST_THREAD, TERMINAL=FIRST_THREAD, LANDEBAHN, HANGAR, VORFELD, NUM_THREADS};
/* states for deadlock algorithms */
enum {UNSAFE, SAFE, UNDEFINED};

/* row vector */
typedef struct {
  unsigned resource[NUM_RESOURCES];
} Vector;

typedef struct {
  Vector thread[NUM_THREADS];
} Matrix;

typedef struct {
  pthread_mutex_t mutex;
  pthread_cond_t resource_released[NUM_RESOURCES];
  Matrix G; /* Gesamtanforderung */
  Matrix B; /* Belegt - Allocation */
  Matrix R; /* Restanforderung - Need */
  Vector f; /* frei - Available */
  Vector v; /* insgesamt vorhanden */
} State;

/* globally accessible variables, mutexes and signals */
extern const char LABEL[]; /* used to label resources */
State Fahrzeugbelegung; /* used for keeping track of resource state */

#endif /* INC_RESOURCES_H */
