#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main() {
	char wochentag[] = "Freitag";
    	int tag = 13;
    	char monat[] = "Mai";
    	int jahr = 1927;
    	char *string; // Hier soll das Datum hineingeschrieben werden!
	
    	// Hier implementieren und dynamisch Speicher reservieren
	string = (char*) malloc(sizeof(char) * (strlen(wochentag)+2+strlen(monat)+4+9+1));

	sprintf(string,"%s, der %d. %s %d",wochentag,tag,monat,jahr);

    	printf("%s\n", string);
    	// Speicher freigeben
	free(string);
	
    	return 0;
}
