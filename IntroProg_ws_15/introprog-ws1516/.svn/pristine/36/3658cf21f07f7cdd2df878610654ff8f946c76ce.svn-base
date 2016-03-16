#include <stdio.h>
#include <stdlib.h>

/*
 * Eine Struktur wird generell durch das Keyword "struct" und einen Namen
 * bezeichnet. Diese müssen immer zusammen angegeben werden, also bspw.
 * als Typ vor einer Variablen oder als Parameter von sizeof().
 *
 * Möchte man einen handlicheren Namen verwenden, bietet sich typedef an.
 * Dieses ist folgendermaßen aufgebaut:
 *   typedef bisherigerTyp neuerName1, neuerName2, ... ;
 *
 * Also heißt beispielsweise
 *   typedef int bla, blub;
 * dass der Typ int nun auch über die Namen bla und blub verwendet werden
 * kann. In unserem Fall ist Folgendes sinnvoll:
 */
typedef struct movie movie;

/*
 * Eine Struktur besteht aus einem oder mehreren Feldern, welche zusammen
 * gespeichert werden. Hat man eine Variable m1 vom Typ "struct movie",
 * kann man mittels m1.title usw. auf die Felder zugreifen. Hat man eine
 * Variable m2 vom Typ "struct movie *", kann man mittels m2->title darauf
 * zugreifen. Äquivalent dazu ist (*m2).title, allerdings wird diese Form
 * bei weiterer Verschachtelung unübersichtlich. Beispiel für den Zugriff
 * auf den Titel des übernächsten Films:
 *   m->next->next->title
 * vs.
 *   (*(*(*m).next).next).title
 * Beachtet bitte beim Dereferenzieren allgemein, dass man sich sicher sein
 * muss, dass die Adresse im Pointer gültig sein muss. Ansonsten stürzt das
 * Programm ab!
 * Konstruiert also eure Schleifen und Abfragen entsprechend.
 */
struct movie {
	char *title;
	char *actor;
	int year;
	/*
	 * Pointer auf den nächsten Film
	 * Statt "movie" könnte hier auch "struct movie" stehen.
	 */
	movie *next;
};

/*
 * Beachtet, dass man typedef auch direkt um die struct-Definition (und nicht
 * nur um die Deklaration) schreiben kann:
 *   typedef struct movie {
 *       char *title;
 *       char *actor;
 *       int year;
 *       struct movie *next; // Hier ist "movie" nicht ausreichend, da die
 *                           // Definition durch typedef noch nicht
 *                           // abgeschlossen ist
 *   } movie;
 *
 *
 * Es ist auch MÖGLICH, dass man typedef so benutzt, dass man Pointer-Typen
 * definiert:
 *   typedef struct movie *movie_ptr;
 * Dann kann man Folgendes schreiben:
 *   movie_ptr m1, m2; // m1 und m2 sind Pointer auf struct movie
 *   m1 = malloc(sizeof(movie));
 *   m1->title = "abc";
 * Allerdings verschleiert das die Funktionalität, man erkennt nicht, dass es
 * ein Pointer ist und baut schneller Fehler ein.
 *
 * Verwendet also typedef mit Pointern BITTE NICHT! :)
 *
 *
 * Manche C-Programmierer gehen sogar so weit, dass sie typedef nur für
 * Funktionen verwenden. Was ich damit meine, könnt ihr bei interesse in der
 * Literatur nachlesen.
 */


/*
 * Hier kann wieder struct movie verwendet werden.
 *
 * Call by value, d.h. die Struktur wird kopiert. Wenn wir etwas an der
 * Struktur ändern, ist es außerhalb der Funktion nicht sichtbar (da es ein
 * anderer Speicher ist).
 */
void print_movie1(movie film) {
	printf("\nFilm: %s\n", film.title);
	printf("Schauspieler: %s\n", film.actor);
	printf("Jahr: %d\n", film.year);
}

/**
 * Call by reference in zwei Varianten.
 */
void print_movie2(movie *film) {
	printf("\nFilm: %s\n", (*film).title);
	printf("Schauspieler: %s\n", (*film).actor);
	printf("Jahr: %d\n", (*film).year);
}

void print_movie3(movie *film) {
	printf("\nFilm: %s\n", film->title);
	printf("Schauspieler: %s\n", film->actor);
	printf("Jahr: %d\n", film->year);
}

/*
 * Wir erstellen einen Film und setzen den next-Pointer auf NULL, damit wir
 * das später nicht vergessen. (Sehr zu empfehlen!)
 *
 * Wir können natürlich nur dynamisch verwalteten Speicher zurückgeben,
 * weshalb wir malloc benutzen.
 */
movie *create_movie(char *title, char *actor, int year) {
	movie *new_movie = malloc(sizeof(movie));
	new_movie->title = title;
	new_movie->actor = actor;
	new_movie->year = year;
	new_movie->next = NULL;
	return new_movie;
}

/*
 * Die Filme werden durchlaufen und ausgegeben.
 *
 * Man könnte die Schleifenbedingung auch anders formulieren und den ersten
 * Film schon vor der Schleife ausgeben. Das hat aber unter Umständen
 * Nachteile.
 *
 * In der Schleife wird man immer das auf NULL prüfen wollen, auf das im
 * Schleifendurchlauf zugegriffen wird. Vgl. Hinweis zur Dereferenzierung oben.
 */
void print_movies(movie *movies) {
	while (movies != NULL) {
		print_movie3(movies);
		movies = movies->next;
	}
}

int main() {
	movie *m1, *m2, *m3;
	m1 = create_movie("Spectre", "Daniel Craig", 2015);
	m2 = create_movie("Skyfall", "Daniel Craig", 2012);
	m3 = create_movie("Goldfinger", "Sean Connery", 1964);
	// Wir verketten hier noch per Hand. Der Nachfolger von m3 bleibt NULL.
	m1->next = m2;
	m2->next = m3;

	print_movies(m1);

	// Das Freigeben des Speichers muss erfolgen, da er dynamisch ist.
	// Man kann das sehr leicht in einer Schleife machen, ähnlich wie bei der
	// Ausgabe. Aber Vorsicht: Wenn m freigegeben ist, kann man nicht mehr
	// auf m->next zugreifen!
	free(m1);
	free(m2);
	free(m3);
	return 0;
}
