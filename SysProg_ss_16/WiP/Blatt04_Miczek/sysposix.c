#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <pthread.h>

enum {PING, PONG} nextAction;

pthread_mutex_t mutex_nextAction;
pthread_cond_t cond_pingDone, cond_pongDone;

void *pingThread(){

	while(1){
		printf("PING");
		pthread_mutex_lock(&mutex_nextAction);
		nextAction = PONG;
		pthread_cond_signal(&cond_pongDone);
		pthread_mutex_unlock(&mutex_nextAction);
		pthread_mutex_lock(&mutex_nextAction);
		while (nextAction != PING){
			
			pthread_cond_wait(&cond_pingDone, &mutex_nextAction);
			
		}
		pthread_mutex_unlock(&mutex_nextAction);
	}
	pthread_exit(NULL);

}

void *pongThread(){

	while(1){
		pthread_mutex_lock(&mutex_nextAction);
                while (nextAction != PONG){
                        
                        pthread_cond_wait(&cond_pongDone, &mutex_nextAction);
                        
                }
               	pthread_mutex_unlock(&mutex_nextAction);
		printf("PONG");
                pthread_mutex_lock(&mutex_nextAction);
                nextAction = PING;
                pthread_cond_signal(&cond_pingDone);
                pthread_mutex_unlock(&mutex_nextAction);
        }
        pthread_exit(NULL);

}

int main(){
	
	pthread_t t1,t2;
	pthread_mutex_init(&mutex_nextAction,NULL);
	pthread_cond_init(&cond_pingDone,NULL);
	pthread_cond_init(&cond_pongDone,NULL);

	int rc = pthread_create(&t1,NULL,pingThread,NULL);
	if (rc != 0){
		printf("lol fag");
		exit(EXIT_FAILURE);
	}

	rc = pthread_create(&t2,NULL,pongThread,NULL);
	if (rc != 0){
		printf("lol fag2");
		exit(EXIT_FAILURE);
	}

	pthread_join(t1,NULL);
	pthread_join(t2,NULL);

	pthread_exit(NULL);
}
