// Bei Fragen gern eine E-Mail an Alexander Elvers oder nachschauen, ob
// ich im Chat bin.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

void memprint(char *string, int len);

int main() {
	// Wir haben drei besondere "Dateien", nämlich Datenströme, welche
	// mit der Aus- bzw. Eingabe des Programms verbunden sind. Diese
	// sind:
	//   stdin - Standardeingabe
	//   stdout - Standardausgabe
	//   stderr - Standardfehlerausgabe
	// (siehe 'man 3 stdin')
	// Datenströme (also auch Dateien) sind immer einem File Descriptor
	// (einer für das Programm eindeutigen Zahl) zugeordnet. Für stdin,
	// stdout und stderr sind diese 0, 1 und 2.
	//
	// Da man beim Benutzen von Dateien gelegentlich vergisst, diese zu
	// schließen, bietet sich eine kleine Hilfe von valgrind an:
	//   valgrind --track-fds=yes ./prog
	// Durch diesen Parameter gibt valgrind Informationen über offene
	// Dateien am Ende der Ausführung aus. Die Standardein- und -ausgaben
	// sind ebenfalls darin enthalten. Kompiliert man mit -g, sieht man
	// auch, in welcher Zeile die Datei geöffnet wurde.
	//
	// Datei für Schreibzugriff öffnen. Falls die Datei nicht existiert,
	// wird sie angelegt. Falls sie existiert, wird mit dem Überschreiben
	// am Anfang begonnen. Zudem wird die Datei geleert.
	// Andere Modi:
	//   r - Lesezugriff
	//   a - Schreibzugriff ohne Leeren der Datei und mit Anfügen am Ende
	//   r+, w+, a+ - Lese- und Schreibzugriff verschiedener Art
	// (siehe 'man 3 fopen')

	// Die Rückgabe ist ein FILE-Pointer oder NULL, wenn die Datei nicht
	// geöffnet werden konnte.
	FILE *fh = fopen("output.txt", "w");
	if (fh == NULL) {
		// Gibt Fehlermeldungen auf stderr aus. Gibt man eine Nachricht
		// als Parameter an, wird diese im Format "%s: " vorangestellt.
		// (siehe 'man 3 perror')
		perror(NULL);
		// An dieser Stelle möchte man oft aus einer Funktion raus,
		// beispielsweise um das Programm mit einem Fehlercode zu beenden:
		//   return 1;
	} else {
		// Das Schreiben in eine Datei funktioniert mit fprintf wie auch
		// bei printf und sprintf. Der erste Paramter ist der FILE-Pointer.
		// (siehe 'man 3 fprintf')
		// Setzt man fh/den ersten Parameter auf stdout, funktioniert es wie
		// printf, setzt man ihn auf stderr, werden Ausgaben auf der
		// Standardfehlerausgabe getätigt.
		fprintf(fh, "abc\n");
		// Eine offene Datei muss geschlossen werden. Wird sie es nicht, ist
		// sie in valgrind als reservierter Speicher zu erkennen. Wie oben
		// beschrieben, erlangt man weitere Informationen mittels
		// --track-fds=yes
		fclose(fh);
	}

	// Datei wird zum Lesen geöffnet.
	fh = fopen("input.txt", "r");
	if (fh == NULL) {
		perror(NULL);
	} else {
		char buffer[100];
		// Analog zu printf gibt es fscanf, scanf und sscanf. Es bietet sich
		// aber oft an, eine Zeile zunächst mit fgets zu lesen und diese
		// anschließend mit sscanf zu parsen.
		// fgets erwartet als Parameter einen String-Buffer, eine maximale
		// Länge und einen FILE-Pointer. Es liest bis zum Ende der Datei oder
		// bis zum Zeilenumbruch, aber maximal Länge-1 Zeichen. Abschließend
		// wird eine 0-Terminierung hinzugefügt.
		// Die Rückgabe von fgets ist der Buffer oder NULL, wenn ein Fehler
		// auftritt oder keine Zeichen gelesen werden konnten, weil die Datei
		// zu Ende ist. Möchte man überprüfen, welcher Fall eingetreten ist,
		// bietet sich feof/ferror an.
		// (siehe 'man 3 fgets' und 'man 3 feof')

		// Zeile lesen
		if (!fgets(buffer, 100, fh)) {
			// Wenn nicht erfolgreich gelesen wurde
			if (ferror(fh)) {
				perror(NULL); // wäre "Success", wenn wir bei EOF wären
				return 1;
			} else {
				printf("Dateiende erreicht.\n");
				return 0;
			}
		}
		// Haben wir das Zeilenende erreicht, ist dies ebenfalls in buffer
		// enthalten. Will man das nicht, muss man mit strlen die Position des
		// letzten Zeichens ermitteln und auf einen Zeilenumbruch überprüfen
		// und ersetzen.
		printf("Zeile: %s\n", buffer);

		// An dieser Stelle würde man normalerweise mit dem Buffer
		// weiterarbeiten, wir wollen uns aber verschiedene Beispiele ansehen.
		// Daher definieren wir uns einzelne Buffer, die zum jeweiligen
		// Problem passen.

		fclose(fh);
	}

	// Ein paar Aufrufe von sscanf:

	char *buffer = "123 		 \n   	672   Wort1  	Wort2\n";
	char string[16];
	int zahl1, zahl2;
	// Liest zwei Zahlen und einen String aus dem Buffer, welche durch
	// Whitespace (beliebige Anzahl und Kombinationen von Leerzeichen,
	// Tabs, Zeilenumbrüchen) getrennt sind.
	// Die Rückgabe ist die Anzahl der korrekt geparsten Werte. Bei
	// fscanf kann die Rückgabe auch EOF (d.h. -1) sein, dann hat man
	// das Dateiende erreicht. Wir prüfen, ob es dem entspricht, was
	// wir erwarten.
	// Die Syntax des Format-Strings ist nicht immer intuitiv, aber im
	// Folgenden werden einige Beispiele vorgestellt:

	if (sscanf(buffer, "%d %d %s", &zahl1, &zahl2, string) == 3) {
	// if (fscanf(fh, "%d %d %s", &zahl1, &zahl2, string) == 3) {
		printf("Mit \"%%d %%d %%s\":\n");
		printf("zahl1=%d\n", zahl1);
		printf("zahl2=%d\n", zahl2);
		printf("string=\"%s\"\n", string);
		printf("\n");
	} else {
		fprintf(stderr, "Lesen nicht erfolgreich!\n");
	}

	// Sollte man für string mehrere Wörter eingegeben haben, stellt man
	// fest, dass nur ein Wort darin enthalten ist. In den erlaubten
	// Zeichen ist Whitespace nicht vorhanden. Man kann statt dem %s
	// aber Zeichenklassen angeben:
	// "%[0-9a-f]" fügt nur Zeichen von 0 bis 9 und von a bis f zum
	// String hinzu.
	// "%[0-9a-zA-Z \t]" erlaubt auch weitere Zeichen, insbesondere
	// Leerzeichen und Tabs.
	// "%[^\n]" erlaubt alles außer Zeilenumbrüche.

	if (sscanf(buffer, "%d %d %[^\n]", &zahl1, &zahl2, string) == 3) {
		printf("Mit \"%%d %%d %%[^\\n]\":\n");
		printf("zahl1=%d\n", zahl1);
		printf("zahl2=%d\n", zahl2);
		printf("string=\"%s\"\n", string);
		printf("\n");
	} else {
		fprintf(stderr, "Lesen nicht erfolgreich!\n");
	}

	// Für Strings sollte man unbedingt eine maximale Länge angeben
	// (bitte bedenken, dass Strings noch terminiert werden müssen).
	// Das obige Beispiel mit string der Länge 16 wäre also besser mit
	// "%d %d %15s" bzw. "%d %d %15[^\n]". Die dort angegebene Länge ist
	// die Maximallänge.

	// Der Speicher reicht hier für string nicht mehr aus. Ohne die Angabe
	// der maximalen Zeichenanzahl würden wir über unseren Speicher
	// hinausschreiben.
	char *buffer2 = "123 		 \n   	672   Wort1  	Wort2 Wort3\n";
	if (sscanf(buffer2, "%d %d %15[^\n]", &zahl1, &zahl2, string) == 3) {
		printf("Mit \"%%d %%d %%15[^\\n]\":\n");
		printf("zahl1=%d\n", zahl1);
		printf("zahl2=%d\n", zahl2);
		printf("string=\"%s\"\n", string);
		printf("\n");
	} else {
		fprintf(stderr, "Lesen nicht erfolgreich!\n");
	}

	// Möchte man unterdrücken, dass geparste Zeichen in die Variablen
	// geschrieben werden, bietet sich ein * an:
	// "%*d" würde in keine Variable geschrieben werden.

	// In diesem Beispiel erwarten wir zwei Zahlen und dann einen String,
	// wobei nur der String gespeichert wird.
	if (sscanf(buffer, "%*d %*d %15[^\n]", string) == 1) {
		printf("Mit \"%%*d %%*d %%15[^\\n]\":\n");
		printf("string=\"%s\"\n", string);
		printf("\n");
	} else {
		fprintf(stderr, "Lesen nicht erfolgreich!\n");
	}

	// Dies bietet sich auch in folgendem Beispiel an:
	// "%d%*s" parst Zahlen mit beliebigem Text dahinter im gleichen Wort,
	// also z.B. "123abc".


	// Möchten wir die Position eines Strings in einem anderen String
	// ermitteln, geht das mit strstr(haystack, needle).
	// (siehe 'man 3 strstr')
	char *haystack = "Hier steht ein Satz.";
	char *substring = strstr(haystack, "ei");
	printf("strstr: \"%s\" (Index: %d)\n", substring, (int)(substring-haystack));

	// Wir können prüfen, ob ein Zeichen Whitespace (oder anderes) ist.
	// (siehe 'man 3 isspace')
	if (isspace(haystack[3])) {
		printf("'%c' ist Whitespace\n", haystack[3]);
	} else {
		printf("'%c' ist kein Whitespace\n", haystack[3]);
	}
	if (isspace(haystack[4])) {
		printf("'%c' ist Whitespace\n", haystack[4]);
	} else {
		printf("'%c' ist kein Whitespace\n", haystack[4]);
	}
	printf("\n");

	// Möchten wir einen String bei einem bestimmten Zeichen oder einer
	// Menge von Zeichen immer aufteilen (in Tokens splitten), dann bietet sich
	// strtok an. Dies erwartet als ersten Parameter den zu zerlegenden
	// String oder NULL, als zweiten den String, bei dem getrennt werden
	// soll (delimiter).
	// Für strtok(string, delim) ist die Rückgabe der Funktion das erste Token.
	// Für strtok(NULL, delim) wird an der letzten Stelle weitergemacht und
	// das nächste Token zurückgegeben.
	// Ist die Rückgabe NULL, gibt es keine weiteren Tokens mehr.
	// strtok funktioniert leider nicht mit konstanten Strings, daher verwenden
	// wir ein char-Array (notfalls muss man sich eine Kopie anlegen).
	// (siehe 'man 3 strtok')

	char buffer3[] = "alpha,beta.gamma...,delta";
	int buffer3_len = strlen(buffer3) + 1;

	memprint(buffer3, buffer3_len);
	printf("1. Wort: %s\n", strtok(buffer3, ".,"));
	memprint(buffer3, buffer3_len);
	printf("2. Wort: %s\n", strtok(NULL, ".,"));
	memprint(buffer3, buffer3_len);
	printf("3. Wort: %s\n", strtok(NULL, ".,"));
	memprint(buffer3, buffer3_len);
	printf("4. Wort: %s\n", strtok(NULL, ".,"));
	memprint(buffer3, buffer3_len);
	printf("5. Wort: %s\n", strtok(NULL, ".,")); // ist NULL
	// Warum verändert sich der Speicherinhalt wohl?

	printf("\n");

	// In einer for-Schleife wirkt alles gleich viel eleganter:
	char buffer4[] = "alpha,beta.gamma...,delta";
	for (char *t = strtok(buffer4, ".,"); t /*!= NULL*/; t = strtok(NULL, ".,")) {
		printf("%s\n", t);
	}

	return 0;
}


/**
 * Ausgabe des Speichers mit Ersetzung des 0-Bytes
 * @param string Anfang des Speichers
 * @param len Länge des Speichers
 */
void memprint(char *string, int len) {
	printf("Memory =");
	for (int i = 0; i < len; i++) {
		if (string[i] == '\0')
			printf(" \\0");
		else
			printf("  %c", string[i]);
	}
	printf("\n");
}
