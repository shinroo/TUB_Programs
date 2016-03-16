#include <stdio.h>
#include <stdlib.h>
#include "input2.h"

int lese_int() {
    int nummer = 0;
    int ret = 0;
    char c;
    printf("Bitte gebe eine Nummer ein: ");
    while(ret == 0) {
        ret = scanf("%d%c", &nummer, &c);
        while (c != '\n' && getchar() != '\n') { };
        if (ret == 0)
            printf("\nDas war keine Nummer. Versuche es erneut: ");
    }

    return nummer;
}

void print_prim(int *array, int laenge) {
    // Da 2 keine Primzahl ist, startet der Index bei 1
    for (int i=0; i<laenge; i++) {
        if (array[i] == 1) {
            printf("%d, ", i+2);
        }
    }
    printf("\n");
}
