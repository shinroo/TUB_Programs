#include <stdio.h>
#include <stdlib.h>

int
ggt_iter(int a, int b)
{
	while(b != 0){
		int c = a % b;
		a = b;
		b = c;
	}
	return a;
}

int
ggt_rec(int a, int b)
{
	if(b != 0){
		return ggt_rec(b, a%b);
	}
	return a;
}

int
ggt_rec_long(int a, int b)
{
	if(b != 0){
		int c = a%b;
		a = b;
		b = c;
		return ggt_rec_long(a, b);
	}
	return a;
}

int
ggt_goto(int a, int b)
{
loop:
	if(b != 0){
		int c = a % b;
		a = b;
		b = c;
		goto loop;
	}
	return a;
}

int
main(int argc, char **argv)
{

	if(argc != 3)
		return 1;

	int a = atoi(argv[1]), b = atoi(argv[2]);

	printf("ggt(%d, %d) = %d\n", a, b, ggt_iter(a, b));
	printf("ggt(%d, %d) = %d\n", a, b, ggt_rec(a, b));
	printf("ggt(%d, %d) = %d\n", a, b, ggt_goto(a, b));
	printf("ggt(%d, %d) = %d\n", a, b, ggt_rec_long(a, b));
	return 0;
}