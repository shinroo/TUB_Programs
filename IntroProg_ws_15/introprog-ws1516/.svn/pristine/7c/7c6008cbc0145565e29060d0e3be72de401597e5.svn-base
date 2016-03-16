#include <stdio.h>
#include <stdlib.h>
#include "input_blatt03.h"
#include <string.h>

FILE *file_pointer = NULL;

int read_line(char *filename, char *name, char *author, int *year, long long int *isbn) {
    if (file_pointer == NULL) {
        file_pointer = fopen(filename, "r");
    }
    if (file_pointer == NULL){
        perror(filename);
        exit(1);
    }
    char line[MAX_STR];
    char *title;
    char *writer;
    char *delim = ";";
    char *pos;

    if (fgets(line, MAX_STR, file_pointer)!=NULL)
    {
        if ((pos = strchr(line, '\n')) != NULL)
            *pos = '\0';
        title = strtok(line, delim);
        writer = strtok(NULL, delim);
        *year = atoi(strtok(NULL, delim));
        *isbn = atoll(strtok(NULL, delim));

        strncpy(name, title, strlen(title));
        name[strlen(title)] = '\0';
        strncpy(author, writer, strlen(writer));
        author[strlen(writer)] = '\0';

        return 0;
    }
    fclose(file_pointer);
    file_pointer = NULL;
    return -1;
}
