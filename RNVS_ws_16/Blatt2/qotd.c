#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h> 

#define	MESSAGE_LENGTH 1024

int main(int argc, char* argv[]){
	if (argc != 3){
		printf("Expected usage: ./qotd host port\n");
		exit(1);
	}

	char* node = argv[1];
	char* service = argv[2];
	int status;

	struct addrinfo hints;
	struct addrinfo *serverinfo;

	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;

	if ((status = getaddrinfo(node, service, &hints, &serverinfo)) != 0){
		fprintf(stderr, "Error resolving host name or IP address\n");
		exit(1);
	}

	int sockfd = socket(serverinfo->ai_family, serverinfo->ai_socktype, serverinfo->ai_protocol);
	int conn_res = connect(sockfd, serverinfo->ai_addr, serverinfo->ai_addrlen);

	void* buf;
	send(sockfd, "", 10, 0);

	int len = MESSAGE_LENGTH;
	buf = (char*) malloc (len);
	int bytes_rec = recv(sockfd, buf, len, 0);
	close(sockfd);

	printf("%s\n", (char*) buf);
}
