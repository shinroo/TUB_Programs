#include <stdio.h>
#include <stdlib.h>
typedef struct _food food;

struct _food {
    char *name;
    food *next;
};

void print_chain(food* myfood) {
    food *this = myfood;

    while (this != NULL) {
        printf("%s frisst ", this->name);
        this = this->next;
        printf("%s.\n", this->name);
    }
}

void free_chain(food* myfood) {
    food *this;
    food *old;

    for (this = myfood->next; this != NULL; ) {
        old = this;
        this = this->next;
        free(old);
    }
}

void food_chain() {
    food *plankton = malloc(sizeof(food));
    plankton->name = "Plankton";
    plankton->next = NULL;
    food *algae = malloc(sizeof(food));
    algae->name = "Alge";
    algae->next = plankton;
    food *fish = malloc(sizeof(food));
    fish->name = "Fisch";
    fish->next = algae;
    food *bird = malloc(sizeof(food));
    bird->name = "Vogel";
    bird->next = fish;
    food *cat = malloc(sizeof(food));
    cat->name = "Katze";
    cat->next = bird;

    print_chain(cat);
    free_chain(cat);
}

int main() {
    food_chain();

    return 0;
}
