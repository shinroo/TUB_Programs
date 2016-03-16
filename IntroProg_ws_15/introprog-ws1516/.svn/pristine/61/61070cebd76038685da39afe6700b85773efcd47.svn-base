#include <stdio.h>
#include <stdlib.h>
#include "input.h"

int lese_int() {
    int number = 0;
    int ret = 0;
    char c;
    printf("Bitte geben sie eine Nummer ein: ");
    while(ret == 0) {
        ret = scanf("%d%c", &number, &c);
        while (c != '\n' && getchar() != '\n') { };
        if (ret == 0)
            printf("\nDas war keine Nummer. Versuchen sie es erneut: ");
    }

    return number;
}
