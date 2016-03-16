#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void find_html_tag(char* msg, char* result) {
    char* begin_msg_ptr = msg;

    // Spitze Klammer finden (<)
    while (*begin_msg_ptr != '\0' && *begin_msg_ptr != '<') {
        begin_msg_ptr++;
    }

    // Wenn wir beim Ende der Zeile sind, haben wir die Klammer nicht gefunden
    if (*begin_msg_ptr == '\0') {
        snprintf(result, 1000, "Found 'htm_tag' but could not find open bracket!");
        return;
    }

    // Wir fangen eins nach < an
    begin_msg_ptr++;

    // Jetzt suchen wir die Klammer zu (>)
    char* end_msg_ptr = begin_msg_ptr;
    while (*end_msg_ptr != '\0' && *end_msg_ptr != '>') {
        end_msg_ptr++;
    }

    // Wenn wir beim Ende der Zeile sind, haben wir die Klammer nicht gefunden
    if (*end_msg_ptr == '\0') {
        snprintf(result, 1000, "Found 'html_tag' but could not find close bracket!");
        return;
    }

    // '>' durch ein '\0' ersetzen, damit wir beim strcpy dort aufhoeren zu kopieren
    *end_msg_ptr = '\0';

    snprintf(result, 1000, "Found 'html_tag' with tag: %s", begin_msg_ptr);

}


/**
 * Dieses Programm oeffnet die datei "datei.txt" und schreibt die Ausgabe in
 * die Datei die als Parameter angegeben ist oder, wenn kein Argument uebergeben
 * wird, auf die Konsole.
 *
 * Die datei.txt muss so aufgebaut sein, dass in jeder Zeile ein Befehl mit
 * einem oder mehr Argumenten steht.
 *
 * Gueltige Befehle sind 'test', 'blah' und 'html_tag'
 * Bei test und blah wird ein String-Argument angegeben. Beispiel:
 *         blah StringOhneLeerzeichen
 *         test NochEinString
 *
 * Bei 'html_tag' wird nach Text in spitzen klammern <...> gesucht.
 * Der Text kann auch Leerzeichen enthalten. Beispiel:
 *
 *         html_tag <Das ist ein Tag mit Leerzeichen>
 *
 * Dieses Programm achtet kaum auf Randfalle und bietet somit keine Grundlage
 * für die abzugebende Aufgabe ;)
 */
int main(int argc, char** argv) {

    FILE* ausgabe;
    FILE* eingabe;

    // Je nachdem ob ein parameter uebergeben wurde, schreiben wir die Ausgabe mit
    // fprintf entweder in die Datei oder nach stdout
    if (argc == 2) {
        ausgabe = fopen(argv[1], "w+");
    } else {
        ausgabe = stdout;
    }

    // Wenn ausgabe == NULL ist, dann hat das oeffnen der Datei wohl nicht geklappt
    if (ausgabe == NULL) {
        printf("Could not open output file!\n");
        return 1;
    }


    // Oeffnen der Datei datei.txt. Falls das nicht klappt brechen wir ab.
    eingabe = fopen("datei.txt", "r");
    if (eingabe == NULL) {
        fprintf(ausgabe, "Could not open input file!\n");
        return 1;
    }

    // In einer while-Schleife gehen wir hier durch jede Zeile in eingabe
    char line[1000];
    int line_no = 1;
    while (fgets(line, 999, eingabe) != NULL) {

        // Wenn nichts in der Zeile steht ueberspringen wir die Zeile
        if (strlen(line) == 0) {
            line_no++;
            continue;
        }

        char token[1000];
        char value[1000];
        char rest[1000];

        // Wir scanen einen Token, einen Wert und den Rest aus dem line string
        int scan_count = sscanf(line, "%s %s %s", token, value, rest);

        // Wenn wir 2 Elemente gescannt haben koennte die Eingabe korrekt sein
        if (scan_count >= 2) {
            // Wir ueberpruefen ob es unsere zu suchenden strings 'test' oder 'blah' sind
            if (strlen(token) == 4 && strcmp(token, "test") == 0) {
                fprintf(ausgabe, "Token was 'test' and value was '%s'\n", value);
            } else if (strlen(token) == 4 && strcmp(token, "blah") == 0) {
                fprintf(ausgabe, "Token was 'blah' and value was '%s'\n", value);
            } else if (strlen(token) == 8 && strcmp(token, "html_tag") == 0) {
                char* msg_ptr = strstr(line, "html_tag");  // Position von 'html_tag' finden
                msg_ptr+=8; //Position auf _nach_ html_tag schieben

                char parsed_message[1000];

                // Diese funktion findet den String zwischen den Spitzen klammern heraus
                find_html_tag(msg_ptr, parsed_message);

                fprintf(ausgabe, "%s\n", parsed_message);
            } else {
                fprintf(ausgabe, "Token '%s' in line %d did not match any of 'test', 'blah', 'html_tag'\n", token, line_no);
            }
        }

        line_no++;
    }

    return EXIT_SUCCESS;
}
