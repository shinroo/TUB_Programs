#include <stdio.h>
#include <stdlib.h>

#define BUFFER_SIZE 255
#define Klein_BUFFER_SIZE 5

void lese_1_nummer(){
	int n;
	printf("Bitte geben Sie eine Zahl\n");
	scanf("%d", &n);

	if (n > 0) printf("true\n\n");
	else printf("false\n\n");
}

void lese_n_nummer(){
	int n;
	printf("Bitte geben Sie eine Zahl\n");
	while(scanf("%d", &n) == 1){
		if (n > 0) printf("true\n\n");
		else printf("false\n\n");	
	}
}

void lese_terminal1(){
	char buffer[BUFFER_SIZE];
	printf("Schreiben Sie etwas\n");
	while(scanf("%s", buffer) != EOF){
		printf("%s\n", buffer);
	}
}

void lese_terminal2(){
	char buffer[BUFFER_SIZE];

	FILE * file = fopen("text_w.txt", "w");
	if(file == NULL) {
		fprintf(stderr, "Datei nicht vorhanden\n");
		return;
	}

	printf("Schreiben Sie etwas\n");
	while(scanf("%s", buffer) != EOF){
		//EOF = Ctrl + D
		fprintf(file,"%s\n", buffer);
	}
	fclose(file);
}

void lese_file(){
	char buffer[BUFFER_SIZE];

	FILE * file = fopen("text.txt", "r");
	if(file == NULL) {
		fprintf(stderr, "Datei nicht vorhanden\n");
		return;
	}

	printf("Schreiben Sie etwas\n");
	while(fscanf(file, "%s", buffer) ==1){
		printf("%s\n", buffer);
	}
	fclose(file);
}

void lese_file2(){
	char buffer[BUFFER_SIZE];

	FILE * file = fopen("text.txt", "r");
	if(file == NULL) {
		fprintf(stderr, "Datei nicht vorhanden\n");
		return;
	}

	FILE * file_out = fopen("text_w.txt", "w");

	printf("Schreiben Sie etwas\n");
	while(fscanf(file, "%255s", buffer) ==1){
		fprintf(file_out,"%s", buffer);
	}
	fclose(file);
	fclose(file_out);
}

void lese_file3(){
	FILE * file = fopen("text.txt", "r");
	if(file == NULL) {
		fprintf(stderr, "Datei nicht vorhanden\n");
		return;
	}

	FILE * file_out = fopen("tut_out2.txt", "w");

	char buffer[BUFFER_SIZE];

	while(fgets(buffer, BUFFER_SIZE, file) != NULL) {
		fprintf(file_out,"%s", buffer);
	}

	fclose(file);
	fclose(file_out);

}

void read_and_write3(){

	FILE * file = fopen("text.txt", "r");
	if(file == NULL) {
		fprintf(stderr, "Datei nicht vorhanden\n");
		return;
	}

	FILE * file_out = fopen("tut_out2.txt", "w");

	char buffer[BUFFER_SIZE];

	while(fgets(buffer, BUFFER_SIZE, file) != NULL) {
		char str_a[BUFFER_SIZE] = "";
		char str_b[BUFFER_SIZE] = "";
		int value;
		sscanf(buffer, "%s %s", str_a, str_b);
		printf("PARAM %s\n", str_a);
		printf("REST %s\n", str_b);
		fprintf(file_out, "%s %s\n", str_a, str_b);
	}

	fclose(file);
	fclose(file_out);

}

int main() {

	lese_1_nummer();
	//lese_n_nummer();

	//lese_terminal1();
	//lese_terminal2();

	//lese_file();
	//lese_file2();
	//lese_file3();
		
	//read_and_write3();

	return 0;
}

