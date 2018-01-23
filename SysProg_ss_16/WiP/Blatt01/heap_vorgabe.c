#include "heap.h"
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

int parent(int i)
{
	return (i-1)/2;
}

int left(int i)
{
	return ((i*2)+1);
}

int right(int i)
{
	return ((i*2)+2);
}
// swap the content
void swap(int j, int i)
{
	/*
	temp.mempos = Mem->heap[i].mempos;
	temp.blocks = Mem->heap[i].blocks;
	temp.key = Mem->heap[i].key;
	strncpy(temp.type, Mem->heap[i].type, 3);

	Mem->heap[i].mempos = Mem->heap[j].mempos;
	Mem->heap[i].blocks = Mem->heap[j].blocks;
	Mem->heap[i].key = Mem->heap[j].key;
	strncpy( Mem->heap[i].type, Mem->heap[j].type, 3);

	Mem->heap[j].mempos = temp.mempos;
	Mem->heap[j].blocks = temp.blocks;
	Mem->heap[j].key = temp.key;
	strncpy(Mem->heap[j].type, temp.type, 3);
	*/
	node temp = Mem->heap[i];
	Mem->heap[i] = Mem->heap[j];
	Mem->heap[j] = temp;
}

/*
 * Sucht das groesste Element zwischen dem Knoten an der Array-Position i und
 * seinen bis zu zwei Kindern.
 */
 int getGreatest(int i)
 {
	 int greatest = i;
	 if( left(i) < Mem->heapSize && Mem->heap[greatest].key < Mem->heap[left(i)].key){
		 greatest = left(i);
	 }

	 if( right(i) < Mem->heapSize && Mem->heap[greatest].key < Mem->heap[right(i)].key){
		 greatest = right(i);
	 }

	 return greatest;
 }

/*
 * Sorgt dafuer, dass die Heap-Bedingung wieder stimmt, falls die Heap-
 * Bedingung bei allen Kind-Baeumen erfuellt ist. (Wenn man von unten nach
 * oben geht, kann man so automatisch die komplette Bedingung wieder
 * herstellen, da die Heap-Eigenschaft auf Blaettern immer erfuellt ist).
 */
void heapify(int i) {
    int greatest = getGreatest(i);
    if (greatest != i) 
    {
        swap(i, greatest);
        heapify(greatest);
    }
}
/* 
 * Loescht den Knoten mit der hoechsten Prioritaet und gibt ihn zurueck.
 */
node getRemHighestPrio() {
	node en;
	en.blocks = -1;
	en.mempos = NULL;
	en.key = -1;
	memcpy(en.type, ((char [3]){'%', 'p', '\0'}), 3*sizeof(char));
    if (Mem->heapSize <= 0)
        return en;

    node highestPrio = Mem->heap[0];
    Mem->heapSize--;
    Mem->heap[0] = Mem->heap[Mem->heapSize];
    heapify(0);
    return highestPrio;
}

void* _malloc( int len, int key ) {
	/* Number of needed blocks, add 1 if len is not exact block size */
	int nBlocks;
	if( len % BLOCK_SIZE == 0){
		nBlocks = len/BLOCK_SIZE;
	} else {
		nBlocks = len/BLOCK_SIZE + 1;
	}

	/* If requesting more memory than is available */
	/* Remaining free memory in heap */
	int freeMem = MEMORY_SIZE - (Mem->currMemPos - Mem->mem);
	/* requested memory */
	int reqMem = nBlocks*BLOCK_SIZE;
	if (reqMem > freeMem){
		printf("Not enough memory!!!!\n");
		return NULL;
	}

	// assign n blocks of memory

	//Mem->heap[Mem->heapSize].mempos = (int*) 0x1234;
	Mem->heap[Mem->heapSize].mempos = Mem->currMemPos;
	Mem->heap[Mem->heapSize].blocks = nBlocks;
	Mem->heap[Mem->heapSize].key = key;

	Mem->currMemPos += nBlocks*BLOCK_SIZE;

	Mem->heapSize++;
	for(int i = parent(Mem->heapSize -1); i >= 0; --i){
		heapify(i);
	}

	return Mem->currMemPos - nBlocks*BLOCK_SIZE;

	/* TODO */
	/* Wie viele bloecke muessen reserviert werden? */
	/* Was muss in den Heap eingefuegt werden? */
	/* Wo muss der Speicher reserviert werden? */
	/* Initialisiere den Datentypen Inhalts des Speichers mit z.B. Integer.  Reine Willkuer. */
	/* Wohin sollte Mem->currMemPos jetzt zeigen? */
	/* Was soll zurueckgegeben werden? */
}

 /* Improved and proved by a professional engineer */
int initMem()
{
	/* Reserve memory for a heap */
	Mem = (heap*) malloc (sizeof(heap));
	if (Mem == NULL) {
		printf("ERROR: no available memory\n");
		return -1;
	}
	/* Reserve memory for the pointers */
	Mem->mem = malloc(MEMORY_SIZE);
	if (Mem->mem == NULL) {
		printf("ERROR: no available memory\n");
		return -1;
	}
	Mem->currMemPos = Mem->mem;

	/* Set heapSize, initially empty */
	Mem->heapSize = 0;

	/* Fills Mem->heap with 0s */
	memset(Mem->heap, 0, sizeof(Mem->heap));

	return MEMORY_SIZE;
}

node getHighestPrio()
{	/* Funktioniert das??? TODO */
	return Mem->heap[0];
}

node find(int key)
{
	/* TODO */
	/* Just run through heap as array until key is found */
	for(int index = 0; index < Mem->heapSize; ++index){
		if (Mem->heap[ index ].key == key){
			return Mem->heap[ index ];
		}
	}
	// If not found
	fprintf(stderr, "Node not found, have another on the house\n");
	return Mem->heap[0];
}

void freeMem()
{

	/* TODO:  Is that everything?	*/
	free(Mem->mem);
	free(Mem);
}

/* VORGABE */
void printHeap()
{
	printf("Heapsize: %i\n", Mem->heapSize);
	printf("Key\tBlocks\n");
	int heapHeight = ceil(log2(Mem->heapSize+1));
	int newline = 1;
	int spaceCnt = heapHeight;
	for (int i = 0; i < Mem->heapSize; ++i)
	{
		if (newline)
		{
			spaceCnt --;
			for (int j = 0; j < 1 << (heapHeight-2); ++j)
			{
				printf("       ");
			}
			newline = 0;
			printf("%i %i", Mem->heap[i].key, Mem->heap[i].blocks);
			for (int j = 0; j < spaceCnt*2; ++j)
			{
				printf("  ");
			}
		} else
		{
			for (int j = 0; j < spaceCnt*2; ++j)
			{
				printf("  ");
			}
			if (i % 2)
			{
				for (int j = 0; j < spaceCnt; ++j)
				{
					printf("  ");
				}
			}
			printf("%i %i", Mem->heap[i].key, Mem->heap[i].blocks);
			for (int j = 0; j < spaceCnt*2; ++j)
			{
				printf("  ");
			}
		}
		if (floor(log2(i+2)) == log2(i+2))
		{
			printf("\n");
			newline = 1;
		}
	}
	printf("\n");
}

/* VORGABE */
int setType(char type[3], int key)
{
	for (int i = 0; i < Mem->heapSize; ++i) {
		if (Mem->heap[i].key == key)
		{
			Mem->heap[i].type[0] = type[0];
			Mem->heap[i].type[1] = type[1];
			return 0;
		}
	}
	return -1;
}

/*VORGABE */
void printMem()
{
	node vals[MEMORY_SIZE/BLOCK_SIZE];
	for (int i = 0; i < MEMORY_SIZE/BLOCK_SIZE; ++i)
	{
		vals[i].mempos = NULL;
		vals[i].type[0] = '%';
		vals[i].type[1] = 'i';
		vals[i].type[2] = '\0';
	}
	for (int i = 0; i < Mem->heapSize; ++i)
	{
		int valsPos = (Mem->heap[i].mempos - Mem->mem)/BLOCK_SIZE;
		vals[valsPos] = Mem->heap[i];
		for (int j = 1; j < Mem->heap[i].blocks; ++j)
		{
			vals[valsPos+j].mempos = (void*) INT_MIN;
		}
	}
	printf("+--------------------+\n");
	for (int j = 0; j < MEMORY_SIZE/BLOCK_SIZE; ++j)
	{
		if (j != 0)
		{
			printf("|--------------------|\n");
		}
		if (vals[j].mempos == NULL)
		{
			printf("|        NULL        |\n");
		}
		else if (vals[j].mempos == (void*) INT_MIN)
		{
			printf("|        -''-        |\n");
		}
		else
		{
			printf("|");
			if (!strcmp(vals[j].type, "%i"))
			{
				printf(vals[j].type, *((int *) vals[j].mempos));
			} else if (!strcmp(vals[j].type, "%s"))
			{
				printf(vals[j].type, (char *) vals[j].mempos);
			} else if (!strcmp(vals[j].type, "%c"))
			{
				printf(vals[j].type, *((char*) vals[j].mempos));
			} else if (!strcmp(vals[j].type, "%d"))
			{
				printf(vals[j].type, *((double*) vals[j].mempos));
			} else if (!strcmp(vals[j].type, "%p"))
			{
				printf(vals[j].type, 
					*((unsigned long **) vals[j].mempos));
			}
			printf("|\n");
		}
	}
	printf("+--------------------+\n");
}

/* HIER KANN AUSPROBIERT WERDEN - AENDERUNGEN ERLAUBT UND ERWUENSCHT */
int main()
{
	initMem();
	int** i = (int**) _malloc(sizeof(int*), 10);
	setType("%p",10);
	*i = (int*) 0x1234;
	char* str = (char*) _malloc(sizeof(char) * 500, 11);
	for (int j = 0; j < 499; ++j)
	{
		str[j] = 'x';
	}
	str[499] = '\0';
	setType("%s",11);

	int* k = (int*) _malloc(sizeof(int), 9);
	*k =  10;
	setType("%i", 9);

	int* l = (int*) _malloc(sizeof(int), 13);
	*l =  13;
	setType("%i", 13);

	int* m = (int*) _malloc(sizeof(int), 2);
	*m =  1000;
	setType("%i", 2);

	int* n = (int*) _malloc(sizeof(int), 99);
	*n =  50;
	setType("%i", 99);

	int* o = (int*) _malloc(sizeof(int), 15);
	*o =  42;
	setType("%i", 15);

	int* p = (int*) _malloc(sizeof(int), 14);
	*p =  41;
	setType("%i", 14);

	printHeap();
	printMem();
	getRemHighestPrio();
	printHeap();
	printMem();
	freeMem();
	return 0;
}
