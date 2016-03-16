#include <stdio.h>
#include <stdlib.h>

typedef struct movie mov;

struct movie {
	char * title;
	char * director;
	int year;
	struct movie * next;
};

void print_movie(mov film) {
	printf("Title: %s; Director: %s; Year: %d\n", film.title, film.director, film.year);
}

void print_movie_pointer(mov * film) {
	printf("Title: %s; Director: %s; Year: %d\n", (*film).title, film->director, film->year);
	if(film->next != NULL) {
		printf("Nachfolger: %s\n", film->next->title); 
	} else {
		printf("Kein Nachfolger\n");
	} 
}

mov * create_movie(char * title, char * director, int year) {
	mov * neu = (mov *) malloc(sizeof(mov));
	neu->title = title;
	neu->director = director;
	neu->year = year;
	neu->next = NULL;
	return neu;
}

void print_movies(mov * first) {
	while(first != NULL) {
		print_movie_pointer(first);
		first = first->next;
	}
}

void free_movies_rek(mov * first) {
	if(first != NULL) {
		free_movies_rek(first->next);
		free(first);
	}
}

void free_movies(mov * first) {
	mov * tmp; 
	while(first != NULL) {
		tmp = first->next;
		free(first);
		first = tmp;
	}

}

int main() {

	mov * terminator = create_movie("Terminator", "James Cameron", 1984);
	mov * matrix = create_movie("Matrix", "Wachoski", 1999);
	mov * star_wars = create_movie("Star Wars", "George Lucas", 1977);

	terminator->next = matrix;
	matrix->next = star_wars;
	star_wars->next = NULL;

	print_movies(terminator);

	free_movies(terminator);

	return 0;
}