#include <stdio.h>

//maximale Länge der Nachricht in der Parameter-Eingabedatei
#define MAX_LEN_MESSAGE 255
//maximale Länge einer Zeile in der Parameter-Eingabedatei
#define MAX_LEN_LINE 1000

//Parameter zum Zeichnen der Weichnachtsbaum-Karte
struct parameters_t {
    //maximale Breite des Weihnachtsbaums: {10,..,60}, default: 42
    int width;
    //Höhe des Weihnachtsbaum: {15,..,80}, default: 17
    int height;
    //Text unterhalb des Weihnachtsbaums: default "There's no place like home."
    char message[MAX_LEN_MESSAGE];
    //Spitze des Weihnachtsbaums: default '*'
    char character;
};

//Um anstatt 'struct parameter_t' auch einfach 'parameter' zu schreiben
typedef struct parameters_t parameters;

//Initialisiert ein alloziertes parameter struct mit Defaultwerten
void init_params_with_defaults(parameters* params);

//Überprüft ob das angegebene parameter struct valide Werte enthält
//Wenn die Parameter korrekt sind wird 1, ansonsten 0 zurückgegeben.
int params_are_valid(parameters* params);

//Gibt die Weihnachts-Karte gemäß params auf file aus
void print_tree(parameters* params, FILE* file);
