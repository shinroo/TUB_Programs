#include <stdio.h>
#include <stdlib.h>

#define BUFFER_SIZE 255

int main() {
	// Datei öffnen und lesen
	FILE * file = fopen("tut_in.txt", "r");
	if(file == NULL) {
		fprintf(stderr, "Datei nicht vorhanden\n");
		return 1;
	}

	// Datei öffnen/anlegen und schreiben
	FILE * file_out = fopen("tut_out.txt", "w");

	// Buffer initialisieren
	char buffer[BUFFER_SIZE];

	// Einlesen mit fscanf
	while(feof(file) != 1) {
		fscanf(file, "%20s", buffer);
		printf("%s\n", buffer);
	}

	// Datei pointer zurück setzen
	rewind(file);

	// Einlesen mit fgets + sscanf
	while(fgets(buffer, BUFFER_SIZE, file) != NULL) {
		char str_a[BUFFER_SIZE] = "";
		char str_b[BUFFER_SIZE] = "";
		int value;
		sscanf(buffer, "%s %d %s", str_a, &value, str_b);
		printf("PARAM %s\n", str_a);
		printf("VALUE %d\n", value);
		printf("REST %s\n", str_b);
		fprintf(file_out, "%s %d\n", str_a, value);
	}

	// Dateien schließen
	fclose(file);
	fclose(file_out);

	return 0;
}