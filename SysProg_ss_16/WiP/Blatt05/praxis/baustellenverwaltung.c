#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <stdbool.h>
#include <pthread.h>
#include <sys/syscall.h>

#include "resources.h"
#include "baustellenverwaltung.h"
#include "print.h"

#define DETECTION 1
#define AVOIDANCE 1
const char LABEL[] = "ABCD";

void init_globals(){

  /* initialize resource state */
  Matrix tmp;
  tmp.thread[TERMINAL] = (Vector) {{1,1,1,2}};
  tmp.thread[LANDEBAHN] = (Vector) {{3,3,0,0}};
  tmp.thread[HANGAR] = (Vector) {{3,0,0,0}};
  tmp.thread[VORFELD] = (Vector) {{0,3,3,0}};
  Fahrzeugbelegung.G = tmp;
  Fahrzeugbelegung.v = (Vector) {{3,3,3,3}};
  Fahrzeugbelegung.f = Fahrzeugbelegung.v;
  Fahrzeugbelegung.R = Fahrzeugbelegung.G;

  /* initialize mutexes/signals */
  for(unsigned r=FIRST_RESOURCE; r<NUM_RESOURCES; r++){
    if( pthread_cond_init (&(Fahrzeugbelegung.resource_released[r]), NULL) ){
      handle_error("cond_init");
    }
  }
  if( pthread_mutex_init(&(Fahrzeugbelegung.mutex), NULL) ){
    handle_error("mutex_init");
  }
  if( pthread_mutex_init(&g_cfd_mutex, NULL) ){
    handle_error("mutex_init");
  }   

  g_checkForDeadlocks = false;
  # if DETECTION
  g_checkForDeadlocks = true;
  # endif
}

/**
 * prueft, ob die Reservierung einer Ressource moeglich ist
 * t: der Prozess, der die Ressource reservieren will
 * r: die Ressource (Bagger, Kran, Lastwagen oder Betonmischer)
 * a: Anzahl der Ressourcen
 */
bool isSafe(unsigned t, unsigned r, unsigned a){

  bool answer = UNSAFE;

  char out[57];
  sprintf(out, "[%d] T%u: is \"allocate(%c, %u)\" safe?",
      gettid(), t+1, LABEL[r], a);

  if(Fahrzeugbelegung.f.resource[r] >= a){
	  answer = SAFE;
  }

  #if AVOIDANCE
  answer = avoid_deadlock(t, r, a, out);
  #endif

  char tmp[60];
  sprintf(tmp, "%s : %s\n", out, (answer == SAFE) ? "yes" : "no" );
  printc(tmp, t);
  
  #ifdef DEBUG
  print_State();
  #endif

  return answer;
}  

/**
 * prueft mittels Bankieralgorithmus, ob nach der Reservierung der Ressource in einen sicheren Zustand vorliegt
 * t: der Prozess der die Ressource reserviert
 * r: die zu reservierende Ressource
 * a: die Anzahl der Ressourcen
 * out: String-Variable, die zur Ausgabe zus\"atzlicher Informationen genutzt werden kann (z.B. moegliche Ausfuehrungsreihenfolge)
 */

bool avoid_deadlock(unsigned t, unsigned r, unsigned a, char *out){
	int answer = UNDEFINED;
	unsigned leftR;

	memset(out, 0, 4);

	if(a <= Fahrzeugbelegung.f.resource[r]){
		leftR = Fahrzeugbelegung.f.resource[r] - a;
	}else{
		return UNSAFE;
	}

	while(answer == UNDEFINED){
		for(unsigned i = 0; i < NUM_THREADS; ++i){
			if(i == t){
				if(leftR <  Fahrzeugbelegung.R.thread[t].resource[r]-a){
					printf("------WE ENTERED THE DEADLOCK IF--------\n");
					return UNSAFE;
				}
			}else if(leftR < Fahrzeugbelegung.R.thread[i].resource[r]){
				printf("------WE ENTERED THE DEADLOCK IF--------\n");
				return UNSAFE;
			}
		}
		answer = SAFE;
	}

	return answer;
}

bool isDeadlocked(unsigned t){

  int answer = UNDEFINED;

  char out[57];
  sprintf(out, "[%d] T%u: Deadlock detected?", gettid(), t+1);
  char tmp[60];

# if DETECTION
  while(answer == UNDEFINED){
	  for(int i = 0; i < NUM_THREADS; ++i){
		  for(int j = 0; j < NUM_RESOURCES; ++j){
			    printf("We have: %d and we need: %d. This is thread %d with r %d.\n", Fahrzeugbelegung.f.resource[i],
											Fahrzeugbelegung.R.thread[t].resource[i], i, j);
			  if(Fahrzeugbelegung.f.resource[j] < Fahrzeugbelegung.R.thread[i].resource[j]){
				  printf("------WE ENTERED THE DEADLOCK IF--------\n");
				  answer = UNSAFE;
				  goto i_hate_you;
			  }
		  }
	  }
	  answer = SAFE;
  }
# endif
  
i_hate_you:
  
  sprintf(tmp, "%s : %s\n", out, (answer == SAFE) ? "no" : "yes" );
  printc(tmp, t);
  
  #ifdef DEBUG
  print_State();
  #endif

  return answer? false : true;
}


void lock_state(unsigned t){
  printd("about to lock state");
  pthread_mutex_lock(&(Fahrzeugbelegung.mutex));
  printd("state locked");
}

void unlock_state(unsigned t){
  printd("state unlocked");
  pthread_mutex_unlock(&(Fahrzeugbelegung.mutex));
}

void allocate_Fahrzeug(unsigned t, unsigned r, unsigned a){

  char tmp[50]; 
  struct timespec ts;

  lock_state(t);

  clock_gettime(CLOCK_REALTIME, &ts);
  ts.tv_sec += 2;
  int alreadyWaited = 0;

  /* wait if request wasn't granted */
  while( (isSafe(t, r, a) != SAFE) ){
	  printf("---------AYYYYYYY MACARENA----------\n");
    sprintf(tmp, "[%d] T%u: waiting to allocate(%c, %u)\n",
        gettid(), t+1, LABEL[r], a);
    printc(tmp, t);
    
    /* first wait is a timed wait */
    if( !alreadyWaited )
      alreadyWaited = pthread_cond_timedwait(&(Fahrzeugbelegung.resource_released[r]),
        &(Fahrzeugbelegung.mutex), &ts);
    else
      pthread_cond_wait(&(Fahrzeugbelegung.resource_released[r]),
        &(Fahrzeugbelegung.mutex));
  }
  
  // t - thread
  // r - resource
  // a - anzahl
  Fahrzeugbelegung.f.resource[r] -= a;
  Fahrzeugbelegung.R.thread[t].resource[r] -= a;
  Fahrzeugbelegung.B.thread[t].resource[r] += a;

  printd("%u unit(s) of resource %c allocated", a, LABEL[r]);
  
  sprintf(tmp, "[%d] T%u: allocate(%c, %u)\n", gettid(), t+1,
      LABEL[r], a);
  printc(tmp, t);
  
  #ifdef DEBUG
  print_State();
  #endif

  unlock_state(t);
}

void release_Fahrzeug(unsigned t, unsigned r, unsigned a){

  char tmp[30];
  printd("[%d] T%u: about to release(%c, %u)\n",
      gettid(), t+1, LABEL[r], a);

  lock_state(t);

  /* TODO: update resource state */
  Fahrzeugbelegung.f.resource[r] += a;
  Fahrzeugbelegung.B.thread[t].resource[r] -= a;

  printd("%u unit(s) of resource %c released", a, LABEL[r]);

  pthread_cond_signal(&(Fahrzeugbelegung.resource_released[r]));

  sprintf(tmp, "[%d] T%u: release(%c, %u)\n", getpid(), t+1,
      LABEL[r], a);
  printc(tmp, t);
  
  #ifdef DEBUG
  print_State();
  #endif

  unlock_state(t);
}

void *thread_work(void *thread_number){

  long t = (long)thread_number;
  char tmp[20];

  sprintf(tmp, "[%u] T%ld: started\n", gettid(), t+1);
  printc(tmp, t);
  switch(t) {
    case TERMINAL:
      usleep(10000);
      allocate_Fahrzeug(t, BAGGER, 1);
      allocate_Fahrzeug(t, LASTWAGEN, 2);
      usleep(10000);
      release_Fahrzeug(t, BAGGER, 1);
      allocate_Fahrzeug(t, KRAN, 1);
      usleep(10000);
      allocate_Fahrzeug(t, BETONMISCHER, 1);
      usleep(50000);
      release_Fahrzeug(t, KRAN, 1);
      usleep(10000);
      release_Fahrzeug(t, BETONMISCHER, 1);
      release_Fahrzeug(t, LASTWAGEN, 2);
      break;
    case LANDEBAHN:
      usleep(10000);
      allocate_Fahrzeug(t, BAGGER, 2);
      allocate_Fahrzeug(t, KRAN, 3);
      usleep(10000);
      allocate_Fahrzeug(t, BAGGER, 1);
      usleep(50000);
      release_Fahrzeug(t, BAGGER, 3);
      usleep(10000);
      release_Fahrzeug(t, KRAN, 3);          
      break;
    case HANGAR:
      usleep(10000);
      allocate_Fahrzeug(t, BAGGER, 3);
      usleep(50000);
      release_Fahrzeug(t, BAGGER, 3);
      break;
    case VORFELD:
      usleep(10000);
      allocate_Fahrzeug(t, KRAN, 2);
      allocate_Fahrzeug(t, BETONMISCHER, 3);
      usleep(10000);
      allocate_Fahrzeug(t, KRAN, 1);
      usleep(50000);
      release_Fahrzeug(t, KRAN, 3);
      usleep(10000);
      release_Fahrzeug(t, BETONMISCHER, 3);
      break;
    case NUM_THREADS:
      /* DL-WatchDog */
      /* poll resource state to check for deadlocks */
      pthread_mutex_lock(&g_cfd_mutex);
      while( g_checkForDeadlocks ){
        pthread_mutex_unlock(&g_cfd_mutex);
        usleep(1000000);
        if( isDeadlocked(t) ){
          char tmp[35];
          sprintf(tmp, "[%d] T%ld: Deadlock detected!\n",
              gettid(), t+1);
          printc(tmp, t);
          pthread_exit((void*)EXIT_FAILURE);
        }
        pthread_mutex_lock(&g_cfd_mutex);
      }
      pthread_mutex_unlock(&g_cfd_mutex);
      break;
    default:
      printf("unexpected!");
      exit(EXIT_FAILURE);
      break;
  }

  pthread_exit(EXIT_SUCCESS);
}

int main(){

  init_globals();

  printf("Total resources are v:\n");
  print_Vector(&(Fahrzeugbelegung.v), true);
  printf("\n");
  printf("Currently available resources are f:\n");
  print_Vector(&(Fahrzeugbelegung.f), true);
  printf("\n");
  printf("Maximum needed resources are G:\n");
  print_Matrix(&(Fahrzeugbelegung.G), true);
  printf("\n");
  printf("Currently allocated resources are B:\n");
  print_Matrix(&(Fahrzeugbelegung.B), true);
  printf("\n");
  printf("Still needed resources are R:\n");
  print_Matrix(&(Fahrzeugbelegung.R), true);
  printf("\n");

  fflush(stdout);
  setbuf(stdout, NULL);

  /* spawn threads */
  pthread_attr_t attr;
  pthread_attr_init(&attr);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
  pthread_t thread[NUM_THREADS+1] = { 0 };
  for(long t=FIRST_THREAD; t<=NUM_THREADS; t++){
    if( pthread_create(&thread[t], &attr, thread_work, (void *)t) ){
      handle_error("create");
    }
  }
  pthread_attr_destroy(&attr);

  /* wait for threads */ 
  void *status;
  unsigned finished = 0;
  for(unsigned t=FIRST_THREAD; t<=NUM_THREADS; t++){
    if( pthread_join(thread[t], &status) ){
      handle_error("join");
    }
    if( status ){
      printf("\n[%d] T%u exited abnormally. Status: %ld\n",
          gettid(), t+1, (long)status);
    } else {
      printf("\n[%d] T%u exited normally.\n", gettid(), t+1);
    }    
    if( t<NUM_THREADS ) finished++;
    /* tell DL-WatchDog to quit*/
    if(finished==NUM_THREADS){
      pthread_mutex_lock(&g_cfd_mutex);
      g_checkForDeadlocks = false;
      pthread_mutex_unlock(&g_cfd_mutex);
    }  
  }

  /* Clean-up */
  if( pthread_mutex_destroy(&g_cfd_mutex) ){
    handle_error("mutex_destroy");
  } 
  if( pthread_mutex_destroy(&(Fahrzeugbelegung.mutex)) ){
      handle_error("mutex_destroy");
  }
  for(unsigned r=FIRST_RESOURCE; r<NUM_RESOURCES; r++){
    if( pthread_cond_destroy(&(Fahrzeugbelegung.resource_released[r])) ){
        handle_error("cond_destroy");
    }
  }

  printf("Main thread exited normally.\nFinal resource state:\n");

  print_State();

  exit(EXIT_SUCCESS);
}
