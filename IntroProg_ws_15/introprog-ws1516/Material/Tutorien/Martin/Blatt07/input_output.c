#include <stdio.h>
#include <stdlib.h>

#define blub 255

int main(int argc, char const *argv[])
{
	FILE *file_in = fopen("hallo.txt", "r");
	FILE *file_out = fopen("out.txt", "w+");
	char *buf = malloc(sizeof(char)*blub);
	char *ausgabe = malloc(sizeof(char)*blub);
	int zahl = 0;
	//fscanf(file_in,"%2s %d", buf, &zahl);
	while(fgets(buf, blub, file_in)!= NULL){

		sscanf(buf, "%s %d", ausgabe, &zahl);

		fprintf(file_out,"%s %d\n", ausgabe, zahl);
	}

	fclose(file_in);
	fclose(file_out);
	free(buf);
	free(ausgabe);
	return 0;
}
