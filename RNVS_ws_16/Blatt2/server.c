/*
 * This should be our QotD server
 * The implementation is based on Beej's guide: http://beej.us/guide/bgnet/
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

#define BACKLOG 0

#define MAX_CHARS 1024

void get_quote(char* buf, char* file, int num_lines){
	// Changes seed every second
	srand(time(NULL));
	int line = rand() % num_lines;
	int count = 0;
	FILE *fp = fopen(file, "r");

	while(fgets(buf, MAX_CHARS, fp) != NULL){
		if(count == line){
			fclose(fp);
			return;
		}else{
			count++;
		}
	}
}

int get_num_lines(char* file){
	FILE *fp = fopen(file, "r");
	int count = 0;
	char buf[MAX_CHARS];
	while(fgets(buf, MAX_CHARS, fp) != NULL){
		count++;
	}
	fclose(fp);
	return count;
}

// checks if ipv4 or ipv6
void *get_in_addr(struct sockaddr *sa){
	if (sa->sa_family == AF_INET) {
		return &(((struct sockaddr_in*)sa)->sin_addr);
	}

	return &(((struct sockaddr_in6*)sa)->sin6_addr);
}

int main(int argc, char* argv[]){

	if (argc != 3){
		printf("Expected usage: ./server quotes_file port\n");
		exit(1);
	}

	char* quotes_file = argv[1];
	char* port = argv[2];

	int num_lines = get_num_lines(quotes_file);

	int sockfd, conn_fd;
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

	printf("server: waiting for connections...\n");

	while(1){
		sin_size = sizeof their_addr;
		conn_fd = accept(sockfd, (struct sockaddr *)&their_addr, &sin_size);
		if(conn_fd == -1){
			perror("accept");
			continue;
		}

		inet_ntop(their_addr.ss_family,
				get_in_addr((struct sockaddr *)&their_addr),
				s, sizeof s);
		printf("server: got connection from %s\n", s);

		char* buf = malloc(MAX_CHARS);
		get_quote(buf, quotes_file, num_lines);
		if(send(conn_fd, buf, MAX_CHARS, 0) == -1)
			perror("send");
		free(buf);
		close(conn_fd);
	}

	close(sockfd);
	return 0;
}
