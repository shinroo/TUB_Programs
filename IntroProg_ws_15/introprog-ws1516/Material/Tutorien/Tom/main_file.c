#include <stdlib.h>
#include <stdio.h>
#include "hello.h"

int
main(int argc, char **argv)
{
	if(argc != 2){
		fprintf(stderr, "usage\n");
		exit(1);
	}

	char buffer[1024];
	FILE *file = fopen(argv[1], "r");
	if(file == NULL){
		perror(argv[1]);
		exit(1);
	}
	while(fgets(buffer, sizeof buffer, file) != NULL){
		char name[1024];
		printf("%s",buffer);
		sscanf(buffer, "%s", name);
		say_hello(name);
		say_bye(name);
	}
	fclose(file);
	return 0;
}
