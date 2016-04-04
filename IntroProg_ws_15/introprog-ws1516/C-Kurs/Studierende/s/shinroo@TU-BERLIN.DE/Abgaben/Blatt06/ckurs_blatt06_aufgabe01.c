#include <stdio.h>
#include <stdlib.h>
#include "input2.h"

int main() {
    	int n = lese_int();
	int laenge = n-1;
	int* array = (int*) malloc(sizeof(int)*laenge);	

	//fill array with 1s
    	for (int filler = 0; filler < laenge; filler++) {
		array[filler] = 1;
	}

	//check for primes
	for (int loop = 0; loop < laenge; loop++){
		if (array[loop] == 1){
			for (int loop2 = loop+1; loop2 < laenge; loop2++){
				if ((loop2 + 2) % (loop + 2) == 0){
					array[loop2] = 0;
				}	
			}
		}	
	}
    	// Mit print_prim Primzahlen ausgeben
    	print_prim(array, laenge);
	free(array);

    	return 0;
}
