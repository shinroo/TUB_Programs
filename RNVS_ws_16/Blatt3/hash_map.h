//Our hash header file

#ifndef hashmap_h
#define hahmap_h

#define SIZE 1337

struct entry_s {
	char *key;
	char *value;
	struct entry_s *next;
};

typedef struct entry_s entry_t;

struct hashtable_s {
	int size;
	struct entry_s **table;
};

typedef struct hashtable_s hashtable_t;

hashtable_t *ht_create();
void ht_delete(hashtable_t * hashtable, char *key);
void ht_set(hashtable_t *hashtable, char *key, char *value);
char *ht_get(hashtable_t *hashtable, char *key);
entry_t *ht_find(hashtable_t *hashtable, char *key);
entry_t *ht_newpair(char *key, char *value);
void ht_free(entry_t *to_delete);

#endif
