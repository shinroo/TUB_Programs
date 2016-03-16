#include <stdio.h>
#include <stdlib.h>

int main() {

    char * title[3]; 
    char * director[3];
    int year[3];

    // terminator
    title[0] = "terminator";
    director[0] = "James Cameron";
    year[0] = 1984;

    // matrix
    title[1] = "Matrix";
    director[1] = "Wachoski";
    year[1] = 1999;

    // star wars
    title[2] = "Star Wars";
    director[2] = "???";
    year[2] = 1977;

    for(int i = 0; i < 3; i++) {
        printf("Title: %s; Director: %s; Year: %d\n", title[i], director[i], year[i]);
    }

	return 0;
}