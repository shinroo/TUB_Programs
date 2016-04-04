#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "input3.h"

/* Die Konstanten:
 *  int MAX_LAENGE_STR - die maximale String Länge
 *  int MAX_LAENGE_ARR - die maximale Array Länge
 *  sind input3.c auf jeweils 255 und 100 definiert
 */

int main(int argc, char **argv) {
	if (argc < 3) {
	        printf("Aufruf: %s <anzahl> <bundesland>\n", argv[0]);
        	printf("Beispiel: %s 100 Bayern\n", argv[0]);
        	printf("Klein-/Großschreibung beachten!\n");
        	exit(1);
    	}	
    	int anzahl = atoi(argv[1]);
    	char *bundesland = argv[2];	

    	// Statisch allokierter Speicher
    	char staedte[MAX_LAENGE_ARR][MAX_LAENGE_STR];
    	char laender[MAX_LAENGE_ARR][MAX_LAENGE_STR];
    	int bewohner[MAX_LAENGE_ARR];
	
    	int len = read_file("staedte.csv", staedte, laender, bewohner);
	
    	// Hier implementieren

	char** Ausgabe = (char **) malloc(len * sizeof(char*));

	for (int loop = 0; loop < len; loop++){
		Ausgabe[loop] = (char *) malloc(100);	
	}

	int AnzahlAusgabe = 0;
	
    	// Mithilfe von write_file(...) soll das Ergebnis in die "resultat.txt"
    	// geschrieben werden. 
	
	for (int Dim1 = 0; Dim1 < len; Dim1++){
		if (strcmp(laender[Dim1],bundesland) == 0){
			if (anzahl <= bewohner[Dim1]){
				sprintf(Ausgabe[AnzahlAusgabe],"Die Stadt %s hat %d Einwohner.",staedte[Dim1],bewohner[Dim1]);
				AnzahlAusgabe++;			
			}		
		}		
	}
	
	write_file(Ausgabe,AnzahlAusgabe);
	
    	// Dynamisch allozierter Speicher muss hier freigegeben werden.
	
	for(int loop2 = 0; loop2 < len ; loop2++){
	    free(Ausgabe[loop2]);
    	}
	free(Ausgabe);
}
