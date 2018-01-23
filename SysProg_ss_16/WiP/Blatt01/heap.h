/*
 * VORGABE MAX-HEAP
 * https://de.wikipedia.org/wiki/Heap_(Datenstruktur)
 */

/*
 * Anzahl der Bytes, die zum reservieren zur verfuegung stehen sollen
 */
#define MEMORY_SIZE 10240
/*
 * Blockgroesse, in die der Speicher zerteilt werden soll. Je groesser die
 * Zahl, desto geringer der Verwaltungsaufwand doch desto groesser die
 * Speicherverschwendung bei Reservierung von Speichereinheiten, die kein
 * ganzzahliges vielfaches dieser Zahl sind.
 */
#define BLOCK_SIZE 128

#include <stdint.h>
#include <stdlib.h>

/*
 * mempos: Hier ist der Pointer auf die Speicheradresse gespeichert, wo der
 *   Speicher reserviert ist.
 * blocks: Hier ist die Anzahl der Bloecke gespeichert, die reserviert wurden
 * key: Hier ist die Prioritaet gepeichert, die dieser Node hat.
 * type: Nur fuer printHeap relevant. Hier wird der Typ der Daten 
 *   gespeichert, die in dem reservierten Speicher hinterlegt sind. Nur so
 *   kann printHeap die Daten korrekt ausgeben. Das Format entspricht hier
 *   dem von printf "%i" steht fuer Integer, "%s" fuer strings. Der
 *   Einfachheit halber sind nur "%i", "%s", "c", "%d" und "%p" zulaessig.
 *   Beachten Sie: das letzte Zeichen ist immer "\0", um den String zu
 *   terminieren!
 */

typedef struct node
{
	void* mempos; // Pointer to the node in memory
	int blocks;
	int key;
	char type[3];
} node;

/*
 * mem: Hier soll der zu verwaltende Speicher reserviert werden in der Größe
 *   MEMORY_SIZE
 * currMemPos: Der einfachheit halber, und weil ohnehin kein Speicher 
 *   freigegeben werden können soll, wird neuer Speicherplatz immer direkt
 *   nach dem letzten benutzten Speichersegment reserviert. Wo zuletzt
 *   Speicher reserviert wurde steht in currPosMem
 * heap: Hier wird das Array, das den Max-Heap repräsentiert, gespeichert.
 *   Um den Vewaltungsaufwand zu reduzieren, ist der Speicher in Bloecke
 *   aufgeteilt. Jeder Node im Heap verwaltet mindestens einen Block, daher
 *   gibt es maximal MEMORY_SIZE/BLOCK_SIZE Knoten. Es kann aber auch mehr
 *   als ein Block von einem Node verwaltet werden, z.B. wenn große
 *   Speicherbloecke fuer Arrays reserviert werden.
 * heapSize: Hier ist die momentane Anzahl aller Knoten im Heap gespeichert.
 */

typedef struct heap
{
	void* mem; // Pointer to the beginning of the memory block
	void* currMemPos; // Last used memory block index
	node heap[MEMORY_SIZE/BLOCK_SIZE];
	int heapSize;
} heap;

/*
 * Hier werden alle Daten gespeichert. Muss noch mit initMem initialisiert
 * werden.
 */
heap* Mem;

/*
 * Initialisiert Mem und sorgt dafuer, dass das Betriebssystem verwaltbaren
 * Speicher reserviert.
 */
int initMem();
/*
 * Reserviert Speicher aus dem Pool, der in initMem zur verfuegung gestellt
 * wird. Erzeugt ausserdem Verwaltungsdaten, die in den Heap eingefuegt
 * werden. Fuegt diese Daten in den Heap ein. Sorgt dafuer, dass die Heap-
 * Eigenschaft beim Einfuegen erhalten bleibt. Entspricht groesstenteils
 * der Insert-Funktion eines Heaps.
 *
 * len: Anzahl der Bytes, die reserviert werden sollen.
 * key: Prioritaet, die der reservierte Speicher haben soll. Je hoeher die
 *   Prioritaet, desto schneller kann der Knoten gefunden werden, der den
 * Speicher verwaltet.
 */
void* _malloc(int len, int key);
/*
 * Fuer printHeap benoetigt.
 * type: "%i", "%s", "c", "%d" und "%p" sind zulaessig. Gibt an, von welchem
 *   typen die Daten im Speicher vom Knoten mit dem Schluessel Key sind, damit
 *   printHeap diese korrekt ausgeben kann.
 */
int setType(char* type, int key);
/*
 * Gibt den Knoten mit dem hoechsten Schluessel zurueck.
 */
node getHighestPrio();
/*
 * Gibt allen Speicher frei, der in initMem reserviert wurde.
 */
void freeMem();

void printMem();
void printHeap();
node getRemHighestPrio();
