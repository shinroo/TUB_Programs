#include <stdio.h>
#include <stdlib.h>
#include "input.h" 

int fibonacci(int n){
	if (n == 1 || n == 2){
		return 1;	
	}
	return fibonacci(n-1) + fibonacci(n-2);
}

int main(){
    int n = lese_int(); // Hier rufen wir die Funktion lese_int auf.
    int f = fibonacci(n);
    printf("Fib(%d) = %d\n" , n, f); 
    return 0; 
}
