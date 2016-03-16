#include <stdio.h>
#include <stdlib.h>

#define MAX_FILELEN 255

void read_file(char *filename, char *output) {
	int c;
	int i = 0;
	FILE *filepointer;
	filepointer = fopen(filename, "r");
	c = fgetc(filepointer);
	do {
		output[i++] = (char) c;
		c = fgetc(filepointer);
	} while(!feof(filepointer));
	printf("\n");
	fclose(filepointer);
}

int main(int argc, char const *argv[])
{
	char *output = (char*) calloc(MAX_FILELEN + 1, sizeof(char));
	read_file("textdatei.txt", output);
	printf("%s\n", output);
	free(output);
	return 0;
}