#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include "input3.h"

int MAX_LAENGE_STR = 255;
int MAX_LAENGE_ARR = 100;

void write_file(char *result[], int len) {
    FILE *fp = fopen("resultat.txt", "w");
    if (fp == NULL){
        perror("resultat.txt");
        exit(1);
    }
    for (int i=0; i<len; i++) {
        fprintf(fp, "%s\n", result[i]);
    }
    fclose(fp);
}

int read_file(char *dateiname, char laender[][MAX_LAENGE_STR], char staedte[][MAX_LAENGE_STR], int bewohner []) {
    FILE *fp = fopen(dateiname, "r");

    if (fp == NULL){
        perror(dateiname);
        exit(1);
    }

    char stadt[MAX_LAENGE_STR];
    char land[MAX_LAENGE_STR];
    int anzahl;
    int i = 0;
    int len;

    while(fscanf(fp, "\"%[^\"]\";\"%[^\"]\";%d\n", land, stadt, &anzahl) != EOF)
    {
        if (i >= MAX_LAENGE_ARR) {
            printf("ERROR: Die Datei ist größer als erwartet!");
            return i;
        }
        len = strlen(land) + 1;
        strncpy(laender[i], land, len-1);
        laender[i][len-1] = '\0';

        len = strlen(stadt) + 1;
        strncpy(staedte[i], stadt, len-1);
        staedte[i][len-1] = '\0';

        bewohner[i] = anzahl;
        i++;
    }
    fclose(fp);
    return i;
}
