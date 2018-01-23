#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>

#include "client.h"

int set_film(film_t *film){
	int status = run(SET, film);
	return status;
}

int delete_film(film_t *film){
	int status = run(DELETE, film);
	return status;
}

int get_film(film_t *film){
	int status = run(GET, film);
	return status;
}
short get16bit(unsigned char *array)
{
	short temp = (short)(((short)(array[0])<<8)+((short)(array[1])));
	return temp;
}
void put16bit(short to_put, unsigned char *array)
{
	unsigned char *temp;
	temp = (unsigned char *)&to_put;
	array[0] = temp[1];
	array[1] = temp[0];
}
void print_packet(packet_t *packet){
	// 1. Print header
	char get = ((packet->header->control&4) != 0) ? 1 : 0;
	char set = ((packet->header->control&2) != 0) ? 1 : 0;
	char delete = ((packet->header->control&1) != 0) ? 1 : 0;
	char ack = ((packet->header->control&8) != 0) ? 1 : 0;
	printf("Key Length: %d\n", get16bit(packet->header->key));
	printf("Value Length: %d\n", get16bit(packet->header->value));
	printf("set %d \nget %d\ndelete %d\nack %d\n", set, get, delete, ack);

	// 2. If film != null, print film
	if(packet->film != NULL){
		printf("Key: %s\n", packet->film->key);
		printf("Value: %s\n", packet->film->value);
		printf("Value Length: %d\n", (int)strlen(packet->film->value));
	}
	printf("---------------\n");
}

void free_packet(packet_t *packet){
	free(packet->header);
	if(packet->film != NULL){
		free(packet->film->key);
		free(packet->film->value);
		free(packet->film);
	}
	free(packet);
}

int run(RPType type, film_t *film){
	if (film == NULL)
		return -1;
	packet_t *rec_pack = NULL;
	int attempts;
	for(attempts = 0; attempts < MAX_ATTEMPTS; attempts++){
		rec_pack = rp_connect(type, film);
		if(rec_pack != NULL){
			break;
		}
		sleep(2);
	}

	if(rec_pack == NULL){
		return -1;
	}else{
		print_packet(rec_pack);
		free_packet(rec_pack);
		return 0;
	}
}

packet_t* rp_connect(RPType type, film_t *film){
	int status, i;
	header_t header;

	struct addrinfo hints;
	struct addrinfo *serverinfo;

	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;

	if ((status = getaddrinfo(SERVER, PORT, &hints, &serverinfo)) != 0){
		fprintf(stderr, "Error resolving host name or IP address\n");
		exit(1);
	}

	int sockfd = socket(serverinfo->ai_family, serverinfo->ai_socktype, serverinfo->ai_protocol);
	int conn_res = connect(sockfd, serverinfo->ai_addr, serverinfo->ai_addrlen);
	if(conn_res == -1 || sockfd == -1){
		perror("connect or socket");
		return NULL;
	}

	header.control = type;
	header.transID = 0; // by default
	short key_ = (short)(strlen(film->key));
	short value_ = (short)(strlen(film->value));
	put16bit(key_, header.key);
	put16bit(value_, header.value);
	printf("Value_len to send: %d\n", value_);

	if (send(sockfd, &header, sizeof(header), 0) == -1) {
			printf("At iteration %d:", i);
			perror("send: header");
			return NULL;
		}

	if (send(sockfd, film->key, strlen(film->key), 0) == -1) {
		printf("At iteration %d:", i);
		perror("send: key");
		return NULL;
	}
	if (send(sockfd, film->value, strlen(film->value), 0) == -1) {
		printf("At iteration %d:", i);
		perror("send: value");
		return NULL;
	}

	packet_t *rec_packet = malloc(sizeof(packet_t));
	rec_packet->header = malloc(sizeof(header_t));
	rec_packet->film = NULL;

	if(recv(sockfd, rec_packet->header, sizeof(header_t), 0) == -1){
		perror("receive: header");
		return NULL;
		}

	short value = get16bit(rec_packet->header->value);
	short key = get16bit(rec_packet->header->key);

	if(type == GET){
		rec_packet->film = malloc(sizeof(film_t));
		rec_packet->film->key = malloc(key+1);
		rec_packet->film->value = malloc(value+1);
		if(recv(sockfd, rec_packet->film->key, key, 0) == -1){
			perror("receive: film.key");
			return NULL;
		}
		if(recv(sockfd, rec_packet->film->value, value, 0) == -1){
			perror("receive: film.value");
			return NULL;
		}
		rec_packet->film->key[key] = '\0';
		rec_packet->film->value[value] = '\0';
		printf("IN HEADER VAL LEN: %d\n", value);
	}
	freeaddrinfo(serverinfo);
	close(sockfd);
	return rec_packet;
}

int main(int argc, char* argv[]){
	if (argc != 2){
		printf("Expected usage: ./qotd filename\n");
		exit(1);
	}

	char *filename = argv[1];
	FILE *fp;
	size_t leng = 0;
	ssize_t read;
	char *line = NULL;

	if ((fp = fopen(filename, "r")) == NULL) {
		perror("opening file");
		exit(1);
	}

	// Get the first 25 lines of the file and convert them to keys and valuse, store them in 2 separate arrays
	int count = 0;
	char *keys[FILMS], *values[FILMS];
	int key_len[FILMS], value_len[FILMS];
	film_t **films = malloc(sizeof(film_t*)*FILMS);
	while ((read = getline(&line, &leng, fp)) != -1 && count < FILMS) {
		char key[leng], value[leng];

		sscanf(line, "%[^;];%[^\t]", key, value);

		key_len[count] = strlen(key);
		value_len[count] = strlen(value);

		keys[count] = malloc(key_len[count]+1);
		values[count] = malloc(value_len[count]+1);

		strncpy(keys[count], key, key_len[count]);
		keys[count][key_len[count]] ='\0';
		strncpy(values[count],value, value_len[count]);
		values[count][value_len[count]] = '\0';

		film_t *film = malloc(sizeof(film_t));
		film->key = keys[count];
		film->value = values[count];
		films[count] = film;

		count++;
	}
	fclose(fp);
	/////////// TODO: freeeeeee!!! but not here, at the end :)
	if (line)
		free(line);
	/* send the set-request #FILMS times */
	int i;
	for (i = 0; i < FILMS; i++){
		set_film(films[i]);
		//sleep(2);
	}
	for (i = 0; i < FILMS; i++){
		get_film(films[i]);
	}

	for (i = 0; i < FILMS; i++){
		delete_film(films[i]);
	}
	for (i = 0; i < FILMS; i++){
		get_film(films[i]);
	}
	for (i = 0; i < FILMS; i++){
		free(films[i]->value);
		free(films[i]->key);
		free(films[i]);
	}
	free(films);

}
