#include <stdio.h>
#include <stdlib.h>

int euklid_iterativ(int a, int b){
	int h = 0;
	while(b > 0) {
		h = a % b;
		a = b;
		b = h;

	}
	return a;
}

int euklid_rekursiv(int a, int b){
	if(b == 0){
		return a;
	} else {
		return euklid_rekursiv(b, a % b); 
	}

}


int main(int argc, char const *argv[])
{
	int a = 3983987;
	int b = 3983987;
	int loesung_itt;
	int loesung_rekursiv;

	loesung_itt = euklid_iterativ(a, b);
	loesung_rekursiv = euklid_rekursiv(a,b);

	printf("Iterativ :%d\n", loesung_itt);
	printf("Rekursiv :%d\n", loesung_rekursiv);
	return 0;
}