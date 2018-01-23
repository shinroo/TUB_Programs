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

short get16bit(unsigned char *array) {
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

int main(int argc, char* argv[]){
	if (argc != 3){
		printf("Expected usage: ./client serverIP serverPort\n");
		exit(1);
	}

	int sockfd;
	serverIP = argv[1];
	serverPort = argv[2];

	int status, i;
	ex_header_t header;

	struct addrinfo hints;
	struct addrinfo *serverinfo;
	struct sockaddr_in their_addr;
	struct addrinfo *p;

	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_DGRAM;

	if ((status = getaddrinfo(serverIP, serverPort, &hints, &serverinfo)) != 0){
		fprintf(stderr, "Error resolving host name or IP address\n");
		exit(1);
	}

	for(p = serverinfo; p != NULL; p = p->ai_next) {
		if ((sockfd = socket(p->ai_family, p->ai_socktype,
						p->ai_protocol)) == -1) {
			perror("listener: socket");
			continue;
		}

		if (bind(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
			close(sockfd);
			perror("listener: bind");
			continue;
		}
		struct sockaddr_in *sockad = (struct sockaddr_in*)(p->ai_addr);
		printf("%s, %d\n", inet_ntoa(sockad->sin_addr), htons(sockad->sin_port));
		break;
	}

	//int yes = 1;
	//setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

	socklen_t addrlen = sizeof(struct sockaddr_in);

	char buffer[10000];

	if(recvfrom(sockfd, buffer, 10000, 0, (struct sockaddr *)&their_addr,  &addrlen) == -1){
		perror("receive: header");
	}
	char control, control2;
	control = buffer[0];
	control2 = buffer[1];
	char key_array[2];
	char val_array[2];
	memcpy(key_array, &buffer[2], 2);
	memcpy(val_array, &buffer[4], 2);
	short key = get16bit(key_array);
	short val = get16bit(val_array);
	char key_text[10000];
	char value_text[10000];
	strncpy(key_text, &buffer[sizeof(ex_header_t)], key);
	strncpy(value_text, &buffer[sizeof(ex_header_t)+1000], val);

	printf("Key: %s\n", key_text);
	printf("Key: %s\n", value_text);

	/*
	if (sendto(sockfd, &packet, (sizeof(ex_packet_t)), 0, (struct sockaddr *)&inc_server, addrlen) == -1) {
			printf("At iteration %d:", i);
			perror("send: header");
			return NULL;
	}
	*/

	freeaddrinfo(serverinfo);
	close(sockfd);
}
