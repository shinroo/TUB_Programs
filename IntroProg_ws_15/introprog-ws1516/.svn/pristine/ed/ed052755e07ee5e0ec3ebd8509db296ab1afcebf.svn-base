#include <stdio.h>
#include "input_blatt02.h"

long for_linear(int n, int*befehle) { 
	long sum = 0; 
	(*befehle)++;// Zuweisung (sum)

	(*befehle)++;// Ersten Vergleich
	(*befehle)++;// Zuweisung (i)
	for (int i = 1; i <= n; ++i) { 
		(*befehle)++; // Vergleich

		sum += get_value_one(); 
		(*befehle)++; // Vergrößerung des Wertes (sum)

		(*befehle)++; // Vergrößerung des Werted (i)
	}
	(*befehle)++; // return
	return sum; 
}

long for_quadratisch(int n, int*befehle) { 
	long sum = 0; 
	(*befehle)++;// Zuweisung (sum)

	(*befehle)++;// Ersten Vergleich
	(*befehle)++;// Zuweisung (i)
	for (int i = 1; i <= n; ++i) { 
		(*befehle)++; // Vergleich

		(*befehle)++;// Ersten Vergleich
		(*befehle)++; // Zuweisung (j)
		for (int j = 1; j <= n; ++j) { 
			(*befehle)++; // Vergleich

			sum += get_value_one(); 
			(*befehle)++; // Vergrößerung des Wertes (sum)

			(*befehle)++; // Vergrößerung des Wertes (j)
		}
		(*befehle)++; //Vergrößerung des Wertes (i)
	}
	(*befehle)++; //return
	return sum; 
}

long for_kubisch(int n, int*befehle) { 
	long sum = 0; 
	(*befehle)++; // Zuweisung (sum)
	
	(*befehle)++;// Ersten Vergleich
	(*befehle)++; // Zuweisung (i)
	for (int i = 1; i <= n; ++i) { 
		(*befehle)++; // Vergleich

		(*befehle)++;// Ersten Vergleich
		(*befehle)++; // Zuweisung (j)
		for (int j = 1; j <= n; ++j) { 
			(*befehle)++; // Vergleich

			(*befehle)++;// Ersten Vergleich
			(*befehle)++; // Zuweisung (k)
			for (int k = 1; k <= n; ++k) { 
				(*befehle)++; // Vergleich

				sum += get_value_one(); 
				(*befehle)++; // Vergrößerung des Wertes (sum)

				(*befehle)++; // Vergrüßerung des Wertes (k)
			}
			(*befehle)++; // Vergrößerung des Wertes (j)
		}
		(*befehle)++; // Vergrößerung des Wertes (i)
	}
	(*befehle)++; // return
	return sum; 
}

int WERTE[] = {5,6,7,8,9,10};
int LEN_WERTE = 6;
int LEN_ALGORITHMEN = 3;

int main(int argc, char *argv[]) {

	long befehle_array[LEN_ALGORITHMEN][LEN_WERTE];
	long werte_array[LEN_ALGORITHMEN][LEN_WERTE];
	double laufzeit_array[LEN_ALGORITHMEN][LEN_WERTE];

	for(int j = 0; j < LEN_WERTE; ++j)
	{
		int n = WERTE[j];
		for(int i = 0; i < LEN_ALGORITHMEN; ++i)
		{
			printf("Starte Algorithmus %d mit Wert %d\n", (i+1), n);
			int anzahl_befehle = 0;
			int wert = 0;

            //Starte den Timer
            start_timer();
        
            //Aufruf der entsprechenden Funktion
            if(i==0)
            {
                wert = for_linear(n, &anzahl_befehle);
            }
            else if(i==1)
            {
                wert = for_quadratisch(n, &anzahl_befehle);
            }
            else if(i==2)
            {
                wert = for_kubisch(n, &anzahl_befehle);
            }
 
            //speichere Laufzeit, Rückgabewert und Anzahl ausgeführter Befehle ab
            laufzeit_array[i][j] = end_timer();
            werte_array[i][j] = wert;
            befehle_array[i][j] = anzahl_befehle;
        }
        printf("\n");
    }
    
    //Ausgabe der Rückgabewerte, Anzahl ausgeführter Befehle sowie der gemessenen Laufzeiten (in Millisekunden)
    printf("%3s \t%28s \t%28s \t%28s\n", "","linear", "quadratisch", "kubisch");
    printf("%3s \t %5s %10s %10s\t %5s %10s %10s\t %5s %10s %10s\n", "n","Wert","Befehle","Laufzeit","Wert","Befehle","Laufzeit","Wert","Befehle","Laufzeit");
    
    for(int j = 0; j < LEN_WERTE; ++j)
    {
        printf("%3d \t ",WERTE[j]);
        for(int i = 0; i < LEN_ALGORITHMEN; ++i)
        {
            printf("%5ld %10ld %10.4f \t ", werte_array[i][j], befehle_array[i][j], laufzeit_array[i][j]);
        }            
        printf("\n");
    }
    
    return 0;
}
