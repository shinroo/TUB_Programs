#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <gcrypt.h>
#include <unistd.h>
#include <sys/wait.h>


#include "util.h"
#include "blockheader.h"

#define MAX_HASHES 10000

void calculate_hash(long unsigned int nonce, char *hash, Blockheader *blockheader, char *n) {
	// put current nonce in blockheader object
	// first, shift long back to char[4]
	n[0] = nonce >> 24;
	n[1] = nonce >> 16;
	n[2] = nonce >> 8;
	n[3] = nonce;
	// reverse byte order
	byte_reversal(n,sizeof(char)*4);
	// put n into blockheader
	blockheader->nonce[0] = n[0];
	blockheader->nonce[1] = n[1];
	blockheader->nonce[2] = n[2];
	blockheader->nonce[3] = n[3];
	// calculate the hash using the sha-256 hashing algorithm
	size_t size = getData(blockheader,&hash);
	size = sha256_digest(hash,size,&hash);
	// To calculate a valid hash, we need to do two hashing passes
	size = sha256_digest(hash,size,&hash);
	if(check_hash(hash,(int)size))
	{
		printf("%ld : ", nonce);
		print_hash(hash,size);
	}
}

int bitcoin_loop(const unsigned int processcount) {
	printf("\n\nStarting bitcoin_loop\n");

	// Check, if input data are valid
	if ((int) processcount > (int) MAX_HASHES) {
		perror("processcount must be <= MAX_HASHES!!!\n");
		return EXIT_FAILURE;
	}

	// Start, end time
	unsigned long start,end;
	// Set start time
	start = current_time_millis();

	// Creates and retrieves the Block-Header information
	Blockheader * blockheader = malloc(sizeof(Blockheader));
	// The getWork method fills an empty Blockheader struct with all the neccesary information needed to calcualte the Hash of a Block.
	getWork(blockheader);
	// The nonce is the value that is incremented in each run to get a different hash value
	char * n = malloc(sizeof(char)*4);
	memcpy(n,&(blockheader->nonce),sizeof(char)*4);
	// The values in the Blockheader are actually in reverse byte order and need to be reversed in order to increment the nonce value.
	byte_reversal(n,sizeof(char)*4);
	// Convert the 4 byte long raw data into an unsinged long
	unsigned long starting_nonce = n[0] << 24 | n[1] << 16 | n[2] << 8 | n[3];
	// The nonce value we received in the getWork method is the actual starting nonce value. We start to calculate hashes with this initial nonce and increase it by one in each run.
	unsigned long nonce = starting_nonce;
	char * hash;

	// Split the calculation of the hashes into several segments based on the processcount

	// Number of repetitions in each loop
	unsigned int rep = MAX_HASHES / ((int) processcount);
	// The rest of repetitions (will be added to the last run)
	unsigned int rest = MAX_HASHES % ((int) processcount);

	// The outer loop runs n=processcount times
	for(int i = 1; i <= (int) processcount; i++) {
		// The inner loop runs n=rep times
		for (int j = 0; j < rep; j++) {
			// Calculate hash.
			// If a hash has the appropriate difficulty, print it to the console
			calculate_hash(nonce, hash, blockheader, n);
			nonce++;
		}
		// The last outer loop must complete the rest of the missing runs (exampla: 10000/3 = 3333, 1 is missing)
		if (i == processcount)
			for (int j = 0; j < rest; j++) {
				calculate_hash(nonce, hash, blockheader, n);
				nonce++;
			}

	}

	end = current_time_millis();
	printf("Calculation finished after %.3fs\n", (double) (end - start) / 1000);

	return EXIT_SUCCESS;
}

int bitcoin_parallel(const unsigned int processcount) {
	printf("\n\nStarting bitcoin_parallel\n");

	// Check, if input data are valid
	if ((int) processcount > (int) MAX_HASHES) {
		perror("processcount must be <= MAX_HASHES!!!\n");
		return EXIT_FAILURE;
	}

	// Start, end time
	unsigned long start,end;
	// Set start time
	start = current_time_millis();

	// Create a Blockheader object and fill it with the initial data using the getWork Method
	Blockheader * blockheader = malloc(sizeof(Blockheader));
	getWork(blockheader);
	char * n = malloc(sizeof(char)*4);
	memcpy(n,&(blockheader->nonce),sizeof(char)*4);
	byte_reversal(n,sizeof(char)*4);
	unsigned long starting_nonce = n[0] << 24 | n[1] << 16 | n[2] << 8 | n[3];
	unsigned long nonce = starting_nonce;
	char * hash;

	//  Split the calculation of the hashes into several segments based on the processcount

	// Number of repetitions in each loop
	unsigned int rep = MAX_HASHES / ((int) processcount);
	// The rest of repetitions
	unsigned int rest = MAX_HASHES % ((int) processcount);
	// Spawn a process for each segment
	// If a hash has the appropriate difficulty print it on the console using print_hash

	// create a process id table with n=processcount cells
	pid_t pids[(int) processcount];
	int procCount = (int) processcount;
	// The outer loop runs n=processcount times, each run spawns one process
	// If spawn was succesful, perform an inner loop and calculate hash n=rep times
	for(int i = 1; i <= (int) processcount; i++) {
		if ((pids[ i ] = fork()) < 0){
			perror("fork\n");
			abort();
		} else if (pids[ i ] == 0) {
				for (int j = 0; j < rep; j++) {
					if (i != processcount) {
						calculate_hash((nonce), hash, blockheader, n);
						nonce++;
					}
				}

		// The last process must complete the rest of missing runs in addition to rep
				if (i == processcount)
				for (int j = 0; j < rep + rest; j++) {
					calculate_hash((nonce), hash, blockheader, n);
					nonce++;
				}
				exit(0);
			}
			/** Increase nonce by rep after every spawn, so that the next process begins counting
			 *	from the place, where its parallel "predecessor" stopped (/will stop)
		   *	(in fact it's not predecessor, since they all should theoretically run parallel to each other)
			 */
			nonce += rep;
	}

	// Wait until all children finish before exiting
	int status = 0;
	pid_t pid;
	while (procCount > 0) {
		pid = wait(&status);
		--procCount;
	}

	end = current_time_millis();
	printf("Calculation finished after %.3fs\n", (double) (end - start) / 1000);

	return EXIT_SUCCESS;
}


/*
Calculates Blockhashes in a simple loop.
*/
int bitcoin_simple() {
	printf("Starting bitcoin_simple\n");
	// Start, end time
	unsigned long start,end;
	// Set start time
	start = current_time_millis();


	// Creates and retrieves the Block-Header information
	Blockheader * blockheader = malloc(sizeof(Blockheader));
	// The getWork method fills an empty Blockheader struct with all the neccesary information needed to calcualte the Hash of a Block.
	getWork(blockheader);
	// The nonce is the value that is incremented in each run to get a different hash value
	char * n = malloc(sizeof(char)*4);
	memcpy(n,&(blockheader->nonce),sizeof(char)*4);
	// The values in the Blockheader are actually in reverse byte order and need to be reversed in order to increment the nonce value.
	byte_reversal(n,sizeof(char)*4);
	// Convert the 4 byte long raw data into an unsinged long
	unsigned long starting_nonce = n[0] << 24 | n[1] << 16 | n[2] << 8 | n[3];
	// The nonce value we received in the getWork method is the actual starting nonce value. We start to calculate hashes with this initial nonce and increase it by one in each run.
	unsigned long nonce = starting_nonce;
	char * hash;
	// In practice it is very hard to find a valid hash, so in this exercise we will limit the amount of hashes we calculate.
	for(;nonce<=(starting_nonce+MAX_HASHES);nonce++) {
		// put current nonce in blockheader object
		// first, shift long back to char[4]
		n[0] = nonce >> 24;
		n[1] = nonce >> 16;
		n[2] = nonce >> 8;
		n[3] = nonce;
		// reverse byte order
		byte_reversal(n,sizeof(char)*4);
		// put n into blockheader
		blockheader->nonce[0] = n[0];
		blockheader->nonce[1] = n[1];
		blockheader->nonce[2] = n[2];
		blockheader->nonce[3] = n[3];
		// calculate the hash using the sha-256 hashing algorithm
		size_t size = getData(blockheader,&hash);
		size = sha256_digest(hash,size,&hash);
		// To calculate a valid hash, we need to do two hashing passes
		size = sha256_digest(hash,size,&hash);
		if(check_hash(hash,(int)size))
		{
			printf("%ld : ", nonce);
			print_hash(hash,size);
		}
	}

	end = current_time_millis();
	printf("Calculation finished after %.3fs\n", (double) (end - start) / 1000);

	free(blockheader);
	return EXIT_SUCCESS;

}

int main(int argc, char** argv) {

	if(argc != 2) {
		printf("Usage: bitcoin PROCESSCOUNT\n");
		return EXIT_FAILURE;
	}

	unsigned int processcount = strtol(argv[1],NULL,10);

	if(bitcoin_simple() != EXIT_SUCCESS) {
		printf("Error or not implemented.\n\n");
	}

	if(bitcoin_loop(processcount) != EXIT_SUCCESS) {
		printf("Error or not implemented.\n\n");
	}

	if(bitcoin_parallel(processcount) != EXIT_SUCCESS) {
		printf("Error or not implemented.\n\n");
	}

	return 0;
}
