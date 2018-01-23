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

	/* TODO: do we have enough resources to grant request? */
	if(Fahrzeugbelegung.f.resource[r] >= a){
		answer = SAFE;
	}else{
		answer = UNSAFE;
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
	if(leadsToUnsafeState(t, r, a)){
		return UNSAFE;
	}else{
		computePossibleOrder(out,a,t,r);
		return SAFE;
	}
}

// Check if assignment will lead to unsafe state
bool leadsToUnsafeState(unsigned t, unsigned r, unsigned a){
	if(Fahrzeugbelegung.f.resource[r] < a){
		return true;
	}

	// left resources and requested resources
	int leftRes = Fahrzeugbelegung.f.resource[r] - a;
	int reqRes;

	for(unsigned i = 0; i < NUM_THREADS; ++i){
		reqRes = Fahrzeugbelegung.R.thread[i].resource[r];
		if(i == t){
			/*
			 * If testing the argument thread,
			 * test it accounting for the newly subtracted value.
			 * This probably makes sense.
			 */
			reqRes -= a;
		}
		if(leftRes < reqRes){
			return true;
		}
	}

	return false;
}

// Will eventually compute possible execution order
void computePossibleOrder(char* out, unsigned a, unsigned t, unsigned r){
	// Bruteforce time
	for(unsigned i1 = 0; i1 < NUM_THREADS; ++i1){
		for(unsigned i2 = 0; i2 < NUM_THREADS; ++i2){
			if(i1==i2){continue;}
			for(unsigned i3 = 0; i3 < NUM_THREADS; ++i3){
				if(i1==i3 || i2==i3){continue;}
				for(unsigned i4 = 0; i4 < NUM_THREADS; ++i4){
					if(i1==i4 || i2==i4 || i3 == i4){continue;}
					if(isPossibleOrder(i1,i2,i3,i4,a,t,r)){
						sprintf(out, "Possible order: %d, %d, %d, %d.", i1+1,i2+1,i3+1,i4+1);
						return;
					}
				}
			}
		}
	}
}

bool isPossibleOrder(unsigned t1, unsigned t2, unsigned t3, unsigned t4, unsigned a, unsigned t, unsigned r){
	bool iPO = true;

	// temp matrix
	Matrix checkMatrix;
	checkMatrix.thread[0] = Fahrzeugbelegung.R.thread[t1];
	checkMatrix.thread[1] = Fahrzeugbelegung.R.thread[t2];
	checkMatrix.thread[2] = Fahrzeugbelegung.R.thread[t3];
	checkMatrix.thread[3] = Fahrzeugbelegung.R.thread[t4];

	// temp vectors
	Vector checkFree = Fahrzeugbelegung.f;
	Matrix checkBel = Fahrzeugbelegung.B; 
	checkBel.thread[t].resource[r] += a;
	checkMatrix.thread[t].resource[r] -= a;

	//check if there are enough resources
	for(unsigned thread = 0; thread < 4; ++thread){
		for(unsigned resource = 0; resource < NUM_RESOURCES; ++resource){
			if(checkFree.resource[resource] < checkMatrix.thread[thread].resource[resource]){
				return false;
			}else{
				checkFree.resource[resource] += checkBel.thread[thread].resource[resource];
			}
		}
	}
	return iPO;
}

bool isDeadlocked(unsigned t){

	bool answer = UNDEFINED;

	char out[57];
	sprintf(out, "[%d] T%u: Deadlock detected?", gettid(), t+1);

# if DETECTION
	if(findDeadlock()){
		answer = UNSAFE;
	}else{
		answer = SAFE;
	}
# endif

	char tmp[60];
	sprintf(tmp, "%s : %s\n", out, (answer == SAFE) ? "no" : "yes" );
	printc(tmp, t);

#ifdef DEBUG
	print_State();
#endif

	return answer? false : true;
}

/*
 Runs through all threads and resources
 to check if there's enough resources in f
 to complete all threads
*/
bool findDeadlock(){
	for(int i = 0; i < NUM_THREADS; ++i){
		for(int j = 0; j < NUM_RESOURCES; ++j){
			if(Fahrzeugbelegung.f.resource[j] < Fahrzeugbelegung.R.thread[i].resource[j]){
				return true;
			}
		}
	}
	return false;
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

	Fahrzeugbelegung.B.thread[t].resource[r] += a;
	Fahrzeugbelegung.R.thread[t].resource[r] -= a;
	Fahrzeugbelegung.f.resource[r] -= a;

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

	Fahrzeugbelegung.B.thread[t].resource[r] -= a;
	Fahrzeugbelegung.f.resource[r] += a;

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
