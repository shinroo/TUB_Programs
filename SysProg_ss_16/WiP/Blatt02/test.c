#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/wait.h>

int main (){

	pid_t pids[10];
	int i;
	int n = 10;
	// Start children
	for ( i = 0; i < n; ++i) {
		if ((pids[i] = fork()) < 0){
			perror("fork\n");
			abort();
		}else if (pids[i] == 0) {
			printf("This is child %d\n", i);
			exit(0);
		}
	}

	int status;
	pid_t pid;
	while (n > 0) {
		pid = wait(&status);
		printf("Child with PID %ld exited with status 0x%x. \n", (long)pid, status);
		--n;
	}
	printf("---END OF PROGRAM---\n");
	return 0;
}
