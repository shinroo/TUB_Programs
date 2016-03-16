#include <stdio.h>
#include <stdlib.h>

typedef struct _movie movie;

struct _movie{
	char *title;
	char *director;
	int year;
	movie *sequel;
};

movie *erstelle_film(char* title, char* director, int year) {
	movie *film = (movie*)malloc(sizeof(movie));
	film->title = title;
	film->director = director;
	film->year = year;
	film->sequel = NULL;
	return film;
}

void print_filme1(movie *film){
	printf("Film\n ======== \n");
	printf("Title: %s\n", (*film).title);
	printf("Director: %s\n", (*film).director);
	printf("Year: %d\n", (*film).year);
}

void print_filme2(movie *film){
	printf("Film\n ======== \n");
	printf("Title: %s\n", film->title);
	printf("Director: %s\n", film->director);
	printf("Year: %d\n", film->year);
}

void print_film_and_sequels(movie *film){
	movie *tempfilm = film;
	while(tempfilm != NULL){
		print_filme2(tempfilm);
		tempfilm = tempfilm->sequel;
	}
}

int main(int argc, char const *argv[])
{
	
	movie matrix;
	matrix.title = "The Matrix";
	matrix.director = "2xWachowski";
	matrix.year = 1999;

	movie matrix2;
	matrix2.title = "Matrix Reloaded";
	matrix2.director = "2xWachowski";
	matrix2.year = 2003;
	
	movie filmarray[2];

	filmarray[0] = matrix;
	filmarray[1] = matrix2;

	movie *matrix3 = erstelle_film("Matrix Guck ihn dir nicht an", "2xWachowski", 2003);

	printf("Blub %s\n", filmarray[1].title);
	print_filme1(&matrix);
	print_filme2(&matrix2);
	print_filme2(matrix3);

	matrix.sequel = &matrix2;
	matrix2.sequel = matrix3;
	print_film_and_sequels(&matrix);

	//int dashieristschwachsinn = matrix3->sequel->year; 

	//printf("%d\n", dashieristschwachsinn);

	free(matrix3);

	return 0;
}