#include <stdio.h>
#include <stdlib.h>

int main(int argc, char const *argv[])
{
	FILE *fileptr = fopen(argv[1], "r");
	int c;
	while(!feof(fileptr)) {
		c = fgetc(fileptr);
		printf("%c", c);
	}
	printf("\n");
	fclose(fileptr);
	return 0;
}