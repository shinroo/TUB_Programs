#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>

#include "server.h"
#include "client.h"

char *clientIP, *clientPort, *serverIP, *serverPort;

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

void print_packet(packet_t *packet){
	// 1. Print header
	char get = ((packet->header->control&4) != 0) ? 1 : 0;
	char set = ((packet->header->control&2) != 0) ? 1 : 0;
	char delete = ((packet->header->control&1) != 0) ? 1 : 0;
	char ack = ((packet->header->control&8) != 0) ? 1 : 0;
	printf("HAPPY PRINT TIME\n");
	printf("Key Length: %d\n", packet->header->key);
	printf("Value Length: %d\n", packet->header->value);
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
		printf("Key Length: %d\n", rec_pack->header->key);
		printf("Value Length: %d\n", rec_pack->header->value);
		print_packet(rec_pack);
		free_packet(rec_pack);
		return 0;
	}
}

packet_t* rp_connect(RPType type, film_t *film){
	int status, i;
	ex_header_t header;

	struct addrinfo hints;
	struct addrinfo *serverinfo;

	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_DGRAM;

	if ((status = getaddrinfo(serverIP, serverPort, &hints, &serverinfo)) != 0){
		fprintf(stderr, "Error resolving host name or IP address\n");
		exit(1);
	}

	int sockfd = socket(serverinfo->ai_family, serverinfo->ai_socktype, serverinfo->ai_protocol);

	header.control = type;
	printf("Our control is %d\n", header.control);
	header.int_cont = 0; // by default

	header.key = (short)(strlen(film->key));
	header.value = (short)(strlen(film->value));

	printf("Value_len to send: %d\n", header.value);

	struct sockaddr_in inc_server, their_addr;

	socklen_t addrlen = sizeof(struct sockaddr_in);

	inc_server.sin_family = AF_INET;
	inc_server.sin_port = htons(atoi(serverPort));
	inet_aton(serverIP, &inc_server.sin_addr);

	char get = ((header.control&4) != 0) ? 1 : 0;
	char set = ((header.control&2) != 0) ? 1 : 0;
	char delete = ((header.control&1) != 0) ? 1 : 0;
	char ack = ((header.control&8) != 0) ? 1 : 0;
	printf("Key Length: %d\n", header.key);
	printf("Value Length: %d\n", header.value);
	printf("set %d \nget %d\ndelete %d\nack %d\n", set, get, delete, ack);
	char buffer[10000];
	memcpy(buffer, &header, sizeof(ex_header_t));
	ex_packet_t packet;
	packet.header = header;
	strcpy(packet.key, film->key);
	strcpy(packet.value, film->value);

	if(sendto(sockfd, &packet, sizeof(ex_packet_t), 0, (struct sockaddr *)&inc_server, addrlen) == -1) {
			printf("At iteration %d:", i);
			perror("send: header");
			return NULL;
	}

	packet_t *out_packet = malloc(sizeof(packet_t));
	out_packet->header = malloc(sizeof(ex_header_t));
	out_packet->film = NULL;
	ex_packet_t rec_packet;

	printf("Waiting to receive\n");
	int numbytes = recvfrom(sockfd, &rec_packet, sizeof(ex_packet_t), 0, (struct sockaddr *)&their_addr,  &addrlen);
	if(numbytes == -1){
		perror("receive: header");
		return NULL;
	}
	printf("Packet is %d bytes long\n", numbytes);

	short value = rec_packet.header.value;
	short key = rec_packet.header.key;
	printf("Immediately after recvfrom: %d, %d\n", key, value);

	if(type == GET){
		out_packet->film = malloc(sizeof(film_t));
		out_packet->film->key = malloc(key+1);
		out_packet->film->value = malloc(value+1);

		strncpy(out_packet->film->key, rec_packet.key, key);
		strncpy(out_packet->film->value, rec_packet.value, value);

		out_packet->film->key[key] = '\0';
		out_packet->film->value[value] = '\0';
	}
	//memcpy(out_packet->header, &rec_packet.header, sizeof(ex_packet_t)); 
	out_packet->header->control = rec_packet.header.control;
	out_packet->header->key = key;
	out_packet->header->value = value;

	freeaddrinfo(serverinfo);
	close(sockfd);
	return out_packet;
}

int main(int argc, char* argv[]){
	if (argc != 6){
		printf("Expected usage: ./client clientIP clientPort serverIP serverPort\n");
		exit(1);
	}

	char *filename = argv[1];
	clientIP = argv[2];
	clientPort = argv[3];
	serverIP = argv[4];
	serverPort = argv[5];
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
	/*
	set_film(films[0]);
	get_film(films[0]);
	delete_film(films[0]);
	set_film(films[0]);
	*/

	for (i = 0; i < FILMS; i++){
		free(films[i]->value);
		free(films[i]->key);
		free(films[i]);
	}
	free(films);

}
