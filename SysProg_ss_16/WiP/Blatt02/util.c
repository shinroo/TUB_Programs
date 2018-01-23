#include "util.h"

#include <stdlib.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <gcrypt.h>
#include <string.h>

unsigned long current_time_millis() {
	struct timeval time;
	gettimeofday(&time, NULL);
	return (time.tv_sec * 1000L + time.tv_usec / 1000L);
}


void print_hash(const unsigned char* string, const int len) {
    size_t size = ((2*len)+1);
	char * mes = malloc(size);
    char * mes_i = 0;
    int i = 0;


    mes_i = mes; //+ 2;
    for (i = 0; i < len; i++) {
        snprintf(mes_i, 3, "%02x", string[i]);
        mes_i += 2;/* XXX assert pointer width equals char width */
    }

    
    char * converter = malloc(size-1);
    memcpy(converter,mes,size-1);
    hex_byte_reversal(converter,size-1);
    memcpy(mes,converter,size-1);
    mes[(2 * len)] = 0;	
    printf(mes);
    printf("\n");
    free(mes);
}


size_t sha256_digest(const char * input, const size_t len,char ** dest) {
   gcry_error_t gError;
   gcry_md_hd_t gHd;
   int algo = GCRY_MD_SHA256; 
   gError = gcry_md_open(
        &gHd, //handle for libgcrypt sha256 hashing algo
        algo,
        0);
    if(gError) {
        printf("libgcrypt error opening sha256 hashing.\n\n");
        return EXIT_FAILURE;
    }
    gcry_md_write(gHd, input, len);
    // prepare the output
    char * output;
    // read the sha256 digest
    output = gcry_md_read(gHd,0);
    // save the size of the hash
    size_t algo_len = gcry_md_get_algo_dlen(algo);
    // save in another object because gcry_md_close frees output...
    char * r = malloc(algo_len);
    memcpy(r,output,algo_len);
    // close the sha256 handle
    gcry_md_close(gHd);
    // save in dest
    *dest = r;
    return algo_len;
}


void byte_reversal(char * input, size_t size) {
    char * output = malloc(size);
    int i = size-1;
    int offset = i;
    for(;i>=0;i--) {
        output[offset-i] = input[i];
    }
    memcpy(input,output,size);
}


void hex_byte_reversal(char * input, size_t size) {
    if(size%2==0) {
        char * output = malloc(size);
        int i = size-1;
        int offset = i;
        for(;i>=0;i-=2) {
            output[offset-i] = input[i];
            output[offset-(i+1)] = input[i-1]; 
        }
        memcpy(input,output,size);
    }
}

int check_hash(const char * hash, const int len){
    int i;
    for(i=len-1;i>=24;i--)
    {
        if(hash[i]!=0)
        {
            return 0;
        }  
    }
    return 1;
}
