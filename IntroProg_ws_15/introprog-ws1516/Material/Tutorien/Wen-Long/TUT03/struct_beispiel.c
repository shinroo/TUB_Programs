#include <stdio.h>
#include <stdlib.h> 

int main(int argc, char* argv[]) {

    /*
     * Definition von neuen Typen:
     * Wir definieren uns hier eigene Typen. Bezueglich der
     * einfach verketteten Liste stellen die Typen folgendes dar:
     * film: Typ der Listenelemente 
     * struct filmliste: Typ der Liste
	 *
     * Bemerkung: Das typedef kann man auch fuer struct filmliste
     * nutzen, damit das struct bei Variablen Definitionen weg-
     * faellt.
     */
	typedef struct film  {
		char *titel;
		int jahr;
		struct film* next;
	} film;

	struct filmliste {
		struct film* first;
		int anzahl;
	};

	/*
	 * Definition von film und struct filmliste Variablen.
	 * Wir erstellen hier eine leere Liste und drei Listenelemente.
	 * auskommentiert: statische Allokation
	 * sonst: dynamische Allokation
	 */
	struct filmliste* filmliste1 = malloc(sizeof(struct filmliste));
	filmliste1->first = NULL;
	filmliste1->anzahl = 0;

	// film film1;
	// film1.titel = "HarryPotter1";
	// film1.jahr = 1999;

	film* film1 = malloc(sizeof(film));
	film1->titel = "HarryPotter1";
	film1->jahr = 1999;
	film1->next = NULL;

	// film film2;
	// film2.titel = "HarryPotter2";
	// film2.jahr = 2000;

	film* film2 = malloc(sizeof(film));
	film2->titel = "HarryPotter2";
	film2->jahr = 2000;
	film2->next = NULL;

	// film film3;
	// film3.titel = "HarryPotter3";
	// film3.jahr = 2001;

	film* film3 = malloc(sizeof(film));
	film3->titel = "HarryPotter3";
	film3->jahr = 2001;
	film3->next = NULL;

	
	/*
	 * Ein kurzes Beispiel, wie man mit Pointer auf struct umgeht:
	 */
	film* film1pointer = film1;

	printf("1) Film: Titel %s in %d\n", film1->titel, film1->jahr);
	printf("2) Film: Titel %s in %d\n", film1pointer->titel, film1pointer->jahr);

	/*
	 * Ein kurzes Beispiel, wie man Arrays von struct Typen definiert:
	 */
	struct film* hp[] = {film1, film2, film3};

	for(int i = 0; i < 3; i++) {
		printf("Titel: %s in %d\n", hp[i]->titel, hp[i]->jahr);
	}

	/*
	 * Erstellung der einfach verketteten Liste:
	 */
	filmliste1->first = film1;
	film1->next = film2;
	film2->next = film3;
	filmliste1->anzahl = 3;

	/*
	 * Ein kurzes Beispiel, wie man durch eine einfach verkettete Liste
	 * iteriert:
	 */
	struct film* hilfspointer = filmliste1->first;
	while(hilfspointer != NULL) {
		printf("Titel: %s\n", hilfspointer->titel);
		hilfspointer = hilfspointer->next;
	}

	/*
	 * Speicher befreien. Auch ueber eine Schleife moeglich.
	 */
	free(film1);
	free(film2);
	free(film3);
	free(filmliste1);

	return 0;
}