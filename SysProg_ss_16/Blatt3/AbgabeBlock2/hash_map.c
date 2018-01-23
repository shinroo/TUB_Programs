// loosely based on: https://gist.github.com/tonious/1377667#file-hash-c

#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
#include <string.h>

#include "hash_map.h"

/* Create a new hashtable. */
hashtable_t *ht_create(){

	hashtable_t *hashtable;

	/* Allocate the table itself. */
	if( ( hashtable = malloc( sizeof( hashtable_t ) ) ) == NULL ) {
		return NULL;
	}

	/* Allocate pointers to the head nodes. */
	if( ( hashtable->table = malloc( sizeof(entry_t *) * SIZE ) ) == NULL ) {
		return NULL;
	}
	int i;
	for(i = 0; i < SIZE; i++ ) {
		hashtable->table[i] = NULL;
	}

	hashtable->size = SIZE;

	return hashtable;
}

/* Hash a string for a particular hash table. */
int ht_hash(char *str) {
	unsigned long hash = 5381;
	int c;

	while (c = *str++){
		hash = ((hash << 5) + hash) + c; /* hash * 33 + c */
	}
	hash = hash % SIZE;

	return hash;
}

/* Create a key-value pair. */
entry_t *ht_newpair(char *key, char *value){
	entry_t *newpair;

	if((newpair = malloc(sizeof(entry_t))) == NULL){
		exit(1);
	}

	char* new_key = (char*) malloc(strlen(key)*sizeof(char)); 
	strcpy(new_key, key);

	char* new_value = (char*) malloc(strlen(value)*sizeof(char)); 
	strcpy(new_value, value);

	newpair->key = new_key;
	newpair->value = new_value;
	newpair->next = NULL;

	return newpair;
}

/* Free all things */
void ht_free(entry_t *to_delete){
	free(to_delete->next);
	free(to_delete->key);
	free(to_delete->value);
	free(to_delete);
}

/* Delete a key-value pair. */
void ht_delete( hashtable_t * hashtable, char *key ) {
	entry_t *delete_pair = ht_find(hashtable, key);
	if(delete_pair != NULL){
		entry_t *last = NULL;

		int bin = ht_hash(key);

		last = hashtable->table[bin];

		if(delete_pair == hashtable->table[bin]){ // beginning of list
			hashtable->table[bin] = delete_pair->next;
			ht_free(delete_pair);
		}else if(delete_pair->next == NULL){ // end of list
			while(last->next != delete_pair){
				last = last->next;
			}
			last->next = NULL;
			ht_free(delete_pair);
		}else{
			while(last->next != delete_pair){
				last = last->next;
			}
			last->next = delete_pair->next;
			ht_free(delete_pair);
		}
	}else{
		printf("Pair didn't exist in the first place...\n");
	}
}

/* Insert a key-value pair into a hash table. */
void ht_set( hashtable_t *hashtable, char *key, char *value ) {
	entry_t* set_pair = ht_find(hashtable, key);

	if(set_pair == NULL){ // pair doesn't exist
		entry_t *next = NULL;
		entry_t *last = NULL;

		set_pair = ht_newpair(key, value);;
		int bin = ht_hash(key);

		next = hashtable->table[bin];

		while(next != NULL){
			last = next;
			next = next->next;
		}

		if(next == hashtable->table[bin]){
			set_pair->next = NULL;
			hashtable->table[bin] = set_pair;
		}else{
			last->next = set_pair;
		}
	}else{
		free(set_pair->value);
		char* new_value = (char*) malloc((strlen(value)+1)*sizeof(char));
		strcpy(new_value, value);
		set_pair->value = new_value;
	}
}

/* Retrieve a key-value pair from a hash table. */
char *ht_get( hashtable_t *hashtable, char *key ) {
	entry_t *get_pair = ht_find(hashtable, key);
	if(get_pair != NULL){
		return get_pair->value;
	}else{
		return NULL;
	}
}

/* Retrieve pointer to pair, NULL if not found */
entry_t *ht_find(hashtable_t *hashtable, char *key){
	entry_t *pair;
	int bin = -1; // poison the value

	bin = ht_hash(key);
	pair = hashtable->table[bin];

	while(pair != NULL && pair->key != NULL && strcmp(key, pair->key) > 0){
		pair = pair->next;
	}

	if(pair == NULL || pair->key == NULL || strcmp(key, pair->key) != 0){
		return NULL;
	}else{
		return pair;
	}
}
