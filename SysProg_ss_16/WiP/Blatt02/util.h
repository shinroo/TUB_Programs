#ifndef UTIL_H_
#define UTIL_H_

#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))


#include <stdio.h>
#include <stdlib.h>


/*
Returns the current time in milliseconds
*/
unsigned long current_time_millis();

/*
This function converts the hexadecimal string into a printable format and prints it on the console using stdio
*/
void print_hash(const unsigned char* string, const int len);

/*
Applies the sha-256 hashing algorithm on the given input. It saves the output in dest and returns the size of the output
*/
size_t sha256_digest(const char * input, const size_t len, char ** dest);

/*
Reverses byte ordering of the given input
*/
void byte_reversal(char * input, size_t size);

/*
Reverses byte ordering when the data is in hexadecimal format
*/
void hex_byte_reversal(char * input, size_t size);

/*
Checks whether the hash has the difficulty as described by the Blockheader
*/
int check_hash(const char * hash, const int len);

#endif