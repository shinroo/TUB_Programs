/*
 * Our RPC Server
 * Mikolaj Walukiewicz, Robert Focke, Hristo Filaretov
 * The implementation is based on Beej's guide: http://beej.us/guide/bgnet/
 * Hashmap implementation from https://gist.github.com/tonious/1377667#file-hash-c
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <time.h>
#include <math.h>

#include "hash_map.h"

#define BACKLOG 0
#define MAX_CHARS 1024

hashtable_t *hashtable;

typedef struct __attribute__((packed)) header_s {
	unsigned char control;
	unsigned char transID;
	unsigned char key[2];
	unsigned char value[2];
} header_t;

void hash_set(char* key, char* value){
	ht_set(hashtable, key, value);
}

char* hash_get(char* key){
	char* temp = ht_get(hashtable, key);
	return temp;
}

void hash_delete(char* key){
	ht_delete (hashtable, key);
}

// checks if ipv4 or ipv6
void *get_in_addr(struct sockaddr *sa){
	if (sa->sa_family == AF_INET) {
		return &(((struct sockaddr_in*)sa)->sin_addr);
	}

	return &(((struct sockaddr_in6*)sa)->sin6_addr);
}

short get16bit(unsigned char *array)
{
	printf("GET16BIT: %02x %02x", array[0], array[1]);
	short temp = (short)(((short)(array[0])<<8)+((short)(array[1])));
	short temp1 = (short)(array[0])<<8;
	printf("   %d, temp1 = %d\n", temp, temp1);
	return temp;
}
void put16bit(short to_put, unsigned char *array)
{
	unsigned char *temp;
	temp = (unsigned char *)&to_put;
	array[0] = temp[1];
	array[1] = temp[0];
}

int main(int argc, char* argv[]){

	if (argc != 2){
		printf("Expected usage: ./server port\n");
		exit(1);
	}

	char* port = argv[1];
	int event_counter = 0;
	header_t header;
	int sockfd, conn_fd, i;
	struct addrinfo hints, *servinfo, *p;
	struct sockaddr_storage their_addr;
	socklen_t sin_size;
	int yes=1;
	char s[INET6_ADDRSTRLEN];
	int rv;

	// set hints
	memset(&hints, 0, sizeof hints);
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_flags = AI_PASSIVE;

	if ((rv = getaddrinfo(NULL, port, &hints, &servinfo)) != 0){
		fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
		return 1;
	}

	for(p = servinfo; p != NULL; p = p->ai_next){
		if((sockfd = socket(p->ai_family, p->ai_socktype,
						p->ai_protocol)) == -1){
			perror("server: socket");
			continue;
		}

		if(setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &yes,
					sizeof(int)) == -1){
			perror("setsockopt");
			exit(1);
		}

		if(bind(sockfd, p->ai_addr, p->ai_addrlen) == -1){
			close(sockfd);
			perror("server: bind");
			continue;
		}

		break;
	}

	freeaddrinfo(servinfo);

	if(p == NULL){
		fprintf(stderr, "server: failed to bind\n");
		exit(1);
	}

	if(listen(sockfd, BACKLOG) == -1){
		perror("listen");
		exit(1);
	}

	hashtable = ht_create();
	printf("server: waiting for connections...\n");

	while(1){
		sin_size = sizeof their_addr;
		conn_fd = accept(sockfd, (struct sockaddr *)&their_addr, &sin_size);
		if(conn_fd == -1){
			perror("accept");
			continue;
		}
		event_counter++;
		printf("Event %d:\n", event_counter);
		inet_ntop(their_addr.ss_family,
				get_in_addr((struct sockaddr *)&their_addr),
				s, sizeof s);
		printf("server: got connection from %s\nReceiving...\n", s);

		// Get the header, divided in sections [Control | transID | keyLength | valueLength]
		if(recv(conn_fd, &header, sizeof(header), 0) == -1)
				perror("receive: header.control");
		unsigned char *array;
		array = (unsigned char *)&header;
		for (i = 0; i < sizeof(header); i++) {

			printf("0x%02x ",array[i]);
		}
		printf("\n");
	/*	if(recv(conn_fd, &header.control, sizeof(header.control), 0) == -1)
			perror("receive: header.control");

		if(recv(conn_fd, &header.transID, sizeof(header.transID), 0) == -1)
			perror("receive: header.transID");

		if(recv(conn_fd, &header.key, sizeof(header.key), 0) == -1)
			perror("receive: header.key");


		if(recv(conn_fd, &header.value, sizeof(header.value), 0) == -1)
			perror("receive: header.value");*/



		short keyLength = get16bit(header.key);

		short valueLength = get16bit(header.value);
		if (valueLength < 0)
			printf("Something went wrong! valueLength is < 0!!!\n");
		// Check, which operation needs to be done, and set its flag to 1, otherwise to 0
		char get = ((header.control&4) != 0) ? 1 : 0;
		char set = ((header.control&2) != 0) ? 1 : 0;
		char delete = ((header.control&1) != 0) ? 1 : 0;
		printf("key_len: %d\n", keyLength);
		printf("val_len: %d\n", valueLength);
		printf("set: %d\nget: %d\ndelete: %d\n", set, get, delete);

		// Receive the key and make a string from it
		char key[keyLength+1];  // single bits of the key, will be sent back if necessary
		for (i = 0; i < keyLength; i++) {
			if(recv(conn_fd, &key[i], 1, 0) == -1)
				perror("receive: key\n");
		}
		strncpy(&key[keyLength], "\0", 1);
		printf("key: %s\n", key);
		// Receive the value and save it as a string
		char value[valueLength+1];  // store the value, will be sent back if necessary
		for (i = 0; i < valueLength; i++) {
			if(recv(conn_fd, &value[i], 1, 0) == -1)
				perror("receive: value\n");
		}
		strncpy(&value[valueLength], "\0", 1);
		printf("value: %s\n", value);
		printf("We received everything\n---\n");

		// Set the Acknowledgement Bit to 1
		header.control = header.control|8;

		// set implementation
		// it's assumed that set cannot fail
		if (set == 1){
			hash_set(key, value);
		}

		// get implementation
		char *get_pointer = "";
		char *get_buffer = "";
		if (get == 1){
			get_pointer = hash_get(key);
			if(get_pointer == NULL){
				header.control = header.control^8;
			}else{
				//printf("Strlen(get_pointer) is %d\n", strlen(get_pointer));
				get_buffer = malloc((strlen(get_pointer)+1)*sizeof(char));
				strcpy(get_buffer, get_pointer);
			}
		}

		// delete implementation
		/* it's assumed that trying to
		   delete a non-existing pair
		   is not a failure
		   */
		if (delete == 1){
			hash_delete(key);
		}

		char ack = ((header.control&8) != 0) ? 1 : 0;

		// check if get was successful
		if(get == 1 && ack == 1){
			short value = strlen(get_buffer)+1;
			printf("We need %d memory!\n", value);
			put16bit(value, header.value);
		}else{
			put16bit(0, header.key);
			put16bit(0, header.value);
			//printf("We're sending: %d, %d\n", header.key, header.value);
		}
		printf("Sending...\n");
		printf("We want to send\n");
		if(send(conn_fd, &header, sizeof(header), 0) == -1)
			perror("send: header");

			array = (unsigned char *)&header;
			for (i = 0; i < sizeof(header); i++) {

				printf("0x%02x ",array[i]);
			}
			printf("\n");
		/*
		if(send(conn_fd, &header.control, 1, 0) == -1)
			perror("send: header.control");
		if(send(conn_fd, &header.transID, 1, 0) == -1)
			perror("send: header.transID");
		if(send(conn_fd, &header.key, 2, 0) == -1)
			perror("send: header.key");
		if(send(conn_fd, &header.value, 2, 0) == -1)
			perror("send: header.value");*/
		if(get == 1 && ack == 1){
			//printf("We're sending %s\n", get_buffer);
			if(send(conn_fd, key, keyLength, 0) == -1)
				perror("send: key");
			if(send(conn_fd, get_buffer, strlen(get_buffer), 0) == -1)
				perror("send: value");

			printf("key: %s\n", key);
			printf("value: %s\n", get_buffer);

			free(get_buffer);
		}
		printf("set: %d\nget: %d\ndelete: %d\nack: %d\n", set, get, delete, ack);
		printf("key_len: %d\n", get16bit(header.key));
		printf("value_len: %d\n", get16bit(header.value));


		array = (unsigned char *)&header;
		for (i = 0; i < sizeof(header); i++) {

			printf("0x%02x ",array[i]);
		}
		printf("\n");

		printf("--------------\n");
		close(conn_fd);
	}

	close(sockfd);
	return 0;
}
