#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>   //definiert den speziellen Wert NaN für floats
#include "input_blatt04.h"

typedef struct _stack stack;
typedef struct _stack_element stack_element;

struct _stack {
    stack_element* top;
};

struct _stack_element {
    stack_element* next;
    float value;
};

/* Füge Element am Anfang des Stack ein
 *
 * astack    - Ein Pointer auf den Stack.
 * value     - Zahl die als neues Element auf den Stack gelegt werden soll.
 */
void stack_push(stack* astack, float value)
{
    /* HIER implementieren */

    //creat new list element
    stack_element* new = (stack_element*) malloc (sizeof(stack_element));
    new->value = value;
    new->next = astack->top;

    //move stack header to the new element
    astack->top = new;
}

/* Nehme das letzte eingefügte Element vom Anfang des Stack
 * Gebe NaN zurück, wenn keine Element vorhanden ist.
 *
 * astack    - Ein Pointer auf den Stack
 *
 * Gebe die im Element enthaltenen Zahl zurück
 */
float stack_pop(stack* astack)
{
    /* HIER implementieren */

    if (astack->top == NULL){
      return NAN;
    }
    else{
      //save output value
      float out = astack->top->value;

      //move list header one back and free popped element
      stack_element* next = astack->top->next;
      free(astack->top);
      astack->top = next;

      return out;
    }
}

/*
 * Führt abhängig von dem Token eine entsprechende Operation auf dem Stacks aus.
 * Wenn es sich bei dem Token um
 *  -> eine Zahl handelt, dann case die Zahl mithilfe von atof zu einem float
 *     und lege sie auf den Stack.
 *  -> einen Operator handelt, dann nehme zwei Zahlen vom Stack, führe die
 *     Operation aus und lege das Resultat auf den Stack.
 *  -> eine nichterkennbare Zeichenkette handelt, dann tue nichts.
 *
 * astack    - Ein Pointer auf den Stack
 * token     - Eine Zeichenkette
 */
void process(stack* astack, char* token)
{
    /* HIER implementieren */

    float x,y = 0;

    //if number
    if (is_number(token)){
      stack_push(astack,atof(token));
    }
    //if add
    else if (is_add(token)){
      x = stack_pop(astack);
      y = stack_pop(astack);
      stack_push(astack,x+y);
    }
    //if sub
    else if (is_sub(token)){
      x = stack_pop(astack);
      y = stack_pop(astack);
      stack_push(astack,y-x);
    }
    //if mult
    else if (is_mult(token)){
      x = stack_pop(astack);
      y = stack_pop(astack);
      stack_push(astack,x*y);
    } //if unrecognized
    else {
      printf("\n<Logik fehlt!>\n");
    }

    return;
    /* Du kannst zur Erkennung der Token folgende Hilfsfunktionen benutzen:
     *
     * Funktion         | Rückgabewert von 1 bedeutet
     * -----------------|----------------------------
     * is_add(token)    | Token ist ein Pluszeichen
     * is_sub(token)    | Token ist ein Minuszeichen
     * is_mult(token)   | Token ist ein
     *                  |  Multiplikationszeichen
     * is_number(token) | Token ist eine Zahl
     */
}

/* Debugausgabe des Stack
 * Diese Funktion kannst du zum debugging des Stack verwenden.
 *
 * astack    - Ein Pointer auf den Stack
 */
void print_stack(stack *astack) {
    int counter = 0;
    printf("\n |xxxxx|xxxxxxxxxxx|xxxxxxxxxxx|xxxxxxxxx|\n");
    printf(" | Nr. | Adresse   | Next      | Wert    |\n");
    printf(" |-----|-----------|-----------|---------|\n");
    for (stack_element* elem=astack->top; elem != NULL; elem = elem->next) {
        printf(" | %3d | %9p | %9p | %7.3f |\n", counter, elem, elem->next, elem->value);
        counter++;
    }
    printf(" |xxxxx|xxxxxxxxxxx|xxxxxxxxxxx|xxxxxxxxx|\n");
}

/* Kreiere einen Stack mit dynamischen Speicher.
 * Initialisiere die enthaltenen Variablen.
 *
 * Gebe einen Pointer auf den Stack zurück.
 */
stack* stack_erstellen() {
    /* HIER implementieren */

    //allocate memory for new stack
    stack* new = (stack*)calloc(1,sizeof(stack));

    //initialise top value
    new->top = NULL;

    return new;
}

int main(int argc, char** args)
{
    stack* astack = stack_erstellen();
    char zeile[MAX_STR];
    char* token;

    intro();
    while (taschenrechner_input(zeile) == 0) {
        // Erstes Token einlesen
        token = strtok(zeile, " ");

        while (token != NULL) {
            printf("Token: %s\n", token);
            // Stackoperationen durchführen
            process(astack, token);
            // Nächstes Token einlesen
            token = strtok(NULL, " ");
            print_stack(astack);
        }

        printf("\nExtrahiere Resultat\n");
        float result = stack_pop(astack);
        print_stack(astack);

        if (astack->top != NULL) {
            while (astack->top != NULL) {
                stack_pop(astack);   //Räume Stack auf
            }
            printf("\nDoes not Compute: Stack nicht leer!\n");
        } else if (result != result) {
            printf("\nDoes not Compute: Berechnung fehlgeschlagen!\n");
        } else {
            printf("\nDein Ergebnis:\t%7.3f\n\n", result);
        }
    }
    free(astack);
}
