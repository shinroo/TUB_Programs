#ifndef BLOCKHEADER_H_
#define BLOCKHEADER_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "util.h"
/*
This struct contains all the information needed to calculate the hash of a Blockheader.
version: Contains the Version of the Bitcoin protocol that is used
prev_hash: Is the Blockheader hash of the previous Blockheader in the Blockchain. Incorporating this hash into the calculation of the new Blockheader creates a chain of Blocks. 
merkle_root: The Merkle root is a datastructure that hashes data blocks similar to a hashlist. The transactions are the data blocks used to create the merkle root. By this way, the transactions are integrated into a Block. 
time: A timefield
bits: The difficulty of a block
nonce: This value is chosen so that the calculated hash reaches the defined difficulty. 
*/
typedef struct Blockheader{
	char version[4];
	char prev_hash[32];
	char merkle_root[32];
	char time[4];
	char bits[4];
	char nonce[4];
} Blockheader;

/*
Fills an empty Blockheader object with all the neccesary values.
*/
void getWork(Blockheader * header);

/*
Converts a Blockheader object into a raw data array used in the calculations of the sha-256 Hash.
*/
size_t getData(Blockheader * header, char ** dest); 

#endif
