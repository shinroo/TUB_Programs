#include "blatt05.h"

// Ausgabe für die Suche, damit das Interface selbst etwas kompakter ist.
void find_and_print(bstree* bst, int phone) {
    bst_node *node = find_node(bst, phone);
    if(node == NULL) {
        printf("Telefonnummer %d existiert nicht.\n", phone);
    } else {
        printf("Tel: %d Name: %s\n", node->phone, node->name);
    }
}

void print_node(bst_node *node) {
    printf("Tel: %4d Name: %s\n", node->phone, node->name);
}

// Debug Ausgabe: Erstellt eine Bilddatei, die den ggw. Baum visualisiert.
void debug(bstree *bst) {
    printf("Erstelle zur Visualisierung die Datei 'baum.dot'...");
    print_to_dot(bst->root, "baum.dot");
    printf("Fertig.\n");
#if !defined(_WIN32) || !defined(_WIN64)
    printf("Konvertiere mithilfe von 'dot' die Datei in das 'png' Format...");
    int status = system("dot -Tpng baum.dot -o baum.png");
    if (status == 0) {
        printf("Fertig.\n"
               "Die Debug Ausgabe befindet sich in der Datei 'baum.png'\n");
    } else {
        printf("Fehler!\n"
               "Die Konvertierung ist fehlgeschlagen."
               "Möglicherweise ist 'dot' bzw. Graphviz nicht installiert!'\n");
    }
#else
    printf("Auf Windows Systemen muss die dot Datei per Hand kompiliert werden.\n"
           "Information zu Graphviz lassen sich unter folgender URL finden\n"
           "   http://www.graphviz.org/Download_windows.php\n");
#endif
}

// Parst die Eingabe aus der übergebenen Datei und führt die entsprechenden Operationen aus.
bstree* read_file(char* filename, bstree *bst)
{
    int phone;
    char* name = malloc(sizeof(char) * MAX_STR);
    FILE *fpointer = NULL;

    while ((fpointer = read_line_from_file(filename, &phone, name, fpointer))!=NULL) {
        bst_insert_node(bst, phone, name);
    }

    free(name);
    return bst;
}

void help() {
    printf(" Fernsprech-Datensatz-System\n"
           " ===========================\n"
           "  Füge in das Telefonbuch ein:\t+ <Nummer> <Name>\n"
           "  Gebe das Telefonbuch aus:\tp\n"
           "  Finde den Namen zu:\t\t? <Nummer>\n"
           "  Debugausgabe des Baumes:\td\n"
           "  Beende das System:\t\tq\n\n"
           " Hinweis: Nummern haben maximal 4 Ziffern\n\n"
    );
}

// Kreiert eine Benutzeroberfläche
bstree* interface(bstree *bst) {
    help();
    char *operation = malloc(sizeof(char) * MAX_STR);
    int phone;
    char *name = malloc(sizeof(char) * MAX_STR);
    printf("> ");
    while (read_line_from_stdin(operation, &phone, name) == 0) {
        if (operation != NULL) {
            if (operation[0] == '?' && phone > -1) {
                find_and_print(bst, phone);
            } else if (operation[0] == '+' && phone > -1 && strlen(name) > 0) {
                bst_insert_node(bst, phone, name);
            } else if (operation[0] == 'd') {
                debug(bst);
            } else if (operation[0] == 'p') {
                bst_in_order_walk(bst);
            } else if (operation[0] == 'q') {
                printf("Exiting...\n");
                free(operation);
                free(name);
                return bst;
            } else {
                printf("Inkorrekte Eingabe\n\n");
                help();
            }
        }
        printf("> ");
        phone = -1;
    }
    printf("Exiting...\n");
    free(operation);
    free(name);
    return bst;
}

int main(int argc, char** argv) {
    // Erzeuge leeren Suchbaum
    bstree bst;
    bst.root = NULL;
    bst.count = 0;

    if (argc != 2)
    {
        printf("Nutzung: %s <Dateiname>\n",argv[0]);
        return 1;
    }

    // Lese das Telefonbuch ein
    read_file(argv[1], &bst);

    // Gib die Benutzeroberfläche aus
    interface(&bst);

    // Gib den Speicher frei
    bst_free_tree(&bst);

    return 0;
}
