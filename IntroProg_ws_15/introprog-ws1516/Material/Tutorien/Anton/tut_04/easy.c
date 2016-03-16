#include <stdio.h>
#include <stdlib.h>

int main() {
    int *mice = malloc(sizeof(int));
    *mice = 3;
    printf("Die Katze frisst %d Mäuse\n", *mice);
    free(mice);
    return 0;
}

