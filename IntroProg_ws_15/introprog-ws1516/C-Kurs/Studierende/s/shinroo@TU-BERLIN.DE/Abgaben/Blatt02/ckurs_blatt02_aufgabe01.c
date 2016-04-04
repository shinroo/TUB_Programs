#include <stdio.h>
#include <stdlib.h>

int main() {
    int nummer = 105; // Probiere verschiedene Werte aus: 101, 103, ...
    printf("Ist %d eine Primzahl?\n", nummer);

    int check = 0;
	
    {

	//suche faktoren von nummer zwischen 2 und nummer/2	

	for (check = 2; check <= nummer/2; check++){
		if (nummer % check == 0){

			//wenn ein faktor gefunden wird, dann ist die Nummer keine Primzahl				

			printf("Nein\n");
			break;		
		}	
	
	}
	
	if (check >= nummer/2){

			//wenn die Schleife fertig ist, und noch kein Faktor gefunden wurde, dann ist die Nummer eine Primzahl

			printf("Ja\n");			
		}
	
	}		
	
	

    return 0;
}

/*
	Die herunterfolgenden Hilfsmittel wurden benutzt:
	http://stackoverflow.com/questions/188425/is-there-a-simple-algorithm-that-can-determine-if-x-is-prime-and-not-confuse-a
	http://stackoverflow.com/questions/5811151/why-do-we-check-upto-the-square-root-of-a-prime-number-to-determine-if-it-is-pri
*/
