#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>
#include <time.h>

#define	MESSAGE_LENGTH 1024
#define BILLION  1000000000L

int main(int argc, char* argv[]){
	struct timespec start, stop;
	double sec, ns;

	char* node = argv[1];
	char* service = argv[2];
	int status;

	struct addrinfo hints;
	struct addrinfo *serverinfo, recmsg;

	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_DGRAM;

	if ((status = getaddrinfo(node, service, &hints, &serverinfo)) != 0){
		fprintf(stderr, "Error resolving host name or IP address\n");
		exit(1);
	}

	int sockfd = socket(serverinfo->ai_family, serverinfo->ai_socktype, serverinfo->ai_protocol);

  	if( clock_gettime( CLOCK_REALTIME, &start) == -1 ) {
    		perror( "clock gettime" );
    		exit( EXIT_FAILURE );
	}
	sendto(sockfd, "test", 10, 0, serverinfo->ai_addr, serverinfo->ai_addrlen);
	void* buf;

	int len = MESSAGE_LENGTH;
	buf = (char*) malloc (len);
	int bytes_rec = recvfrom(sockfd, buf, len, 0, serverinfo->ai_addr, &recmsg.ai_addrlen);
	if( clock_gettime( CLOCK_REALTIME, &stop) == -1 ) {
      		perror( "clock gettime" );
      		exit( EXIT_FAILURE );
    	}
	close(sockfd);

	printf("%s\n", (char*) buf);
	sec = ( stop.tv_sec - start.tv_sec );
  	ns = ( stop.tv_nsec - start.tv_nsec );
    	printf( "%lfs %lfms\n", sec, ns/1000000 );

	return 0;
}
