#ifndef INC_DEADLOCK_H
#define INC_DEADLOCK_H

#ifdef SYS_gettid
#define gettid() (pid_t) syscall (SYS_gettid)
#else
#define gettid() getpid()
#endif

#define DETECTION 1 // Deadlock Detection enable/disable
//#define AVOIDANCE 1 // Deadlock Avoidance enable/disable

#define handle_error(msg) \
    do { perror(msg); exit(EXIT_FAILURE); } while (0)

/* t: thread_number, r: resource_number, a: amount */
void allocate_Fahrzeug(unsigned t, unsigned r, unsigned a);
void release_Fahrzeug(unsigned t, unsigned r, unsigned a);
bool isSafe(unsigned t, unsigned r, unsigned a);
bool isDeadlocked(unsigned t);
bool avoid_deadlock(unsigned t, unsigned r, unsigned a, char *out);
void *thread_work(void *thread_number);
void lock_state(unsigned t);
void unlock_state(unsigned t);
void init_globals();
bool findDeadlock();
bool leadsToUnsafeState(unsigned t, unsigned r, unsigned a);
void computePossibleOrder(char* out, unsigned a, unsigned t, unsigned r);
bool isPossibleOrder(unsigned t1, unsigned t2, unsigned t3, unsigned t4, unsigned a, unsigned t, unsigned r);

/* globally accessible variables, mutexes and signals */
bool g_checkForDeadlocks;
pthread_mutex_t g_cfd_mutex;

#endif  /* INC_DEADLOCK_H */
