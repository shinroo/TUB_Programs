#include <stdio.h>
#include <stdlib.h>

//function to print each value of an array of length len of íntegers

void print_array(int array[], int len) {
    printf("Array: ");

	for (int loop = 0; loop < len; loop++){
		if (loop == len - 1){
			printf(" %d",array[loop]);		
		}
		else{
			printf(" %d,",array[loop]);
		}
	}

	printf("\n");
}

//function to calculate the minimum value of an array of length len of integers

int min(int array[], int len) {
	int min = array[0];
	
	for (int loop = 1; loop < len; loop++){
		if (array[loop] < min){
			min = array[loop];		
		}	
	}

	return min;
}

//function to calculate the maximum value of an array of length len of integers

int max(int array[], int len) {
	int max = 0;
	
	for (int loop = 0; loop < len; loop++){
		if (array[loop] > max){
			max = array[loop];		
		}	
	}

	return max;
}

//function to calculate the sum of the values of an array of length len of integers

void sum(int array[], int len, int *summe){
	
	*summe = 0;

	for (int loop = 0; loop < len; loop++){
		*summe = *summe + array[loop];	
	}	
}

// Schreibe die Funktion "sum", "min" und "max"

int main() {
    int array[] = {9, 4, 7, 8, 10, 5, 1, 6, 3, 2};
    int len = 10;
    int summe;
    print_array(array, len);
    // Gebe hier nacheinander das Minimum, Maximum und die Summe
    // aus. Trenne die Werte durch einen einzelnen Zeilenumbruch.
    printf("Minimum: %d\n", min(array,len));
    printf("Maximum: %d\n", max(array,len));
    sum(array,len,&summe);
    printf("Summe: %d\n",summe);
}
