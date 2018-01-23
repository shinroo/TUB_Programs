#ifndef GAMEOFLIFE_CONFIG_H
#define GAMEOFLIFE_CONFIG_H

// Diese Datei dient zur Konfiguration des Programms


// Definiert die Anzahl verwendeter Gamer-Threads
#define NUM_GAMER_THREADS	50

// GUI = 1 -> Aktiviert die graphische Darstellung des Spielfeldes
// GUI = 0 -> Ausgabe des Spielfelds auf der Konsole (HINWEIS: Wählen Sie in
// dem Falle ein kleines Spielfeld (zB. 50x50) mit einen kleinen Offset)
#define GUI					1

// Definiert die Breite und Höhe des Spielfeldes
#define GAMEFIELD_WIDTH		100
#define GAMEFIELD_HEIGHT	100

// Verschiebt die Figuren (Glider/Glider Gun) auf dem Spielfeld
#define COL_OFFSET 			10
#define ROW_OFFSET 			10

// Nach wie vielen Millisekunden soll das Spielfeld aktualisiert werden?
#define REFRESH_RATE		50

// Parameter für die graphische Darstellung
#define BORDER_WIDTH		1
#define CELLHEIGHT			10
#define CELLWIDTH 			10


#endif //GAMEOFLIFE_CONFIG_H
