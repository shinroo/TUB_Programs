#include <stdio.h>
#include <stdlib.h>

void main(){
	int NUM_THREADS = 4;
	int count = 0;
	for(int i1 = 0; i1 < NUM_THREADS; ++i1){
		for(int i2 = 0; i2 < NUM_THREADS; ++i2){
			if(i1==i2){continue;}
			for(int i3 = 0; i3 < NUM_THREADS; ++i3){
				if(i1==i3 || i2==i3){continue;}
				for(int i4 = 0; i4 < NUM_THREADS; ++i4){
					if(i1==i4 || i2==i4 || i3 == i4){continue;}
					count++;
					printf("%d,%d,%d,%d\n", i1, i2, i3, i4);
				}
			}
		}
	}
	printf("Count is %d\n", count);
}
