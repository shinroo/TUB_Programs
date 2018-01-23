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
#include "client.h"

#define BILLION 1000000000
long int get_delay(times_t times){
	//(t4-t1) - (t3-t2);

}

long int get_offset(times_t times){

}

unsigned long NTP_to_ns(unsigned long timestamp){
	unsigned long s = (((timestamp & 0x00000000ffffffff) * BILLION) / 0xffffffff);
	unsigned long ns = (BILLION * (timestamp >> 32));
	return s + ns;
}
/* Formats the timespec structure into the time structure used by NTP servers */
unsigned long time_to_NTP(struct timespec timestamp){
	unsigned long sec = (timestamp.tv_sec << 32);
	unsigned long ns_frac = timestamp.tv_nsec * 0xffffffff / BILLION;
	return sec | ns_frac;
}
int main(){
	struct timespec t1_time, t4_time, test_time;
	header_t header;
	times_t timestamps;
	double sec, ns;
	test_time.tv_sec = 1;
	test_time.tv_nsec = 10;
	printf ("result: %ld\n", NTP_to_ns(time_to_NTP(test_time)));

	//char* node = "130.149.17.21"; //server_list[3];
	char* node = "ntps1-0.cs.tu-berlin.de"; //server_list[3];
	char* service = "123";
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

	if( clock_gettime( CLOCK_REALTIME, &t1_time) == -1 ) {
		perror( "clock gettime" );
		exit( EXIT_FAILURE );
	}
	/* Version set to 4 and Mode to 3 (Client), rest to 0 */
	header.control = 0x23000000;
	header.origin_t = time_to_NTP(t1_time);
	printf("Sending header...\n");
	sendto(sockfd, &header, sizeof(header_t), 0, serverinfo->ai_addr, serverinfo->ai_addrlen);

	timestamps.t1 = header.origin_t;
	printf("Waiting to receive header...\n");
	int bytes_rec = recvfrom(sockfd, &header, sizeof(header_t), 0, serverinfo->ai_addr, &recmsg.ai_addrlen);
	if( clock_gettime( CLOCK_REALTIME, &t4_time) == -1 ) {
		perror( "clock gettime" );
		exit( EXIT_FAILURE );
	}
	close(sockfd);
	timestamps.t2 = header.receive_t;
	timestamps.t3 = header.transmit_t;
	timestamps.t4 = time_to_NTP(t4_time);
	printf("%ld\n", timestamps.t3);

	return 0;
}
