#include <stdio.h>
#include <stdlib.h>

int main() {
	int breite = 6;
	int hoehe = 3;
	
	//print eine oberste Reihe von ausschließlich As mit der Breite breite+2 damit das B-Rechteck umrahmt wird
	for (int loop=1; loop<=breite+2;loop++){
		printf("A");	
	}
	printf("\n");

	//print das B-Rechteck und umrahme jeder Zeile mit As
	for (int hoeheCounter=1; hoeheCounter <= hoehe; hoeheCounter++){
		printf("A");
		for (int breiteCounter=1; breiteCounter <= breite; breiteCounter++){
			printf("B");		
		}
		printf("A\n");	
	}

	//print eine unterste Reihe von ausschließlich As mit der Breite breite+2 damit das B-Rechteck umrahmt wird
	for (int loop=1; loop<=breite+2;loop++){
		printf("A");	
	}
	printf("\n");

	return 0;
	
}
