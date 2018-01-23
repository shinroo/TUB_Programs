
//Our client header file

#ifndef client_h
#define client_h

#define	MESSAGE_LENGTH 1024
#define FILMS 25
#define MAX_ATTEMPTS 10

typedef struct film_s {
	char *key;
	char *value;
} film_t;

typedef struct packet_s{
	ex_header_t *header;
	film_t *film;
} packet_t;

typedef enum { DELETE=1, SET=2, GET=4 } RPType;

packet_t* rp_connect(RPType type, film_t *film);
int run(RPType type, film_t *film);
int set_film(film_t *film);
int delete_film(film_t *film);
int get_film(film_t *film);

#endif
