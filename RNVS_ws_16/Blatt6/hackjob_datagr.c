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

#define	MESSAGE_LENGTH 1024
#define BILLION  1000000000
 long net_to_host( long t) {
	return (long )(ntohl((int)(t >> 32))) | ((long )(ntohl((int)(t))) << 32);
}
 long get_delay(times_t times){
	return (times.t4 - times.t1) - (times.t3 - times.t2);
}

long int get_offset(times_t times){
	long t21 = (times.t2 - times.t1);
	long t34 = (times.t3 - times.t4);
	return (t21 + t34)/2;
}

 long NTP_to_ns( long timestamp){
	 long s = (((timestamp & 0x00000000ffffffff) * BILLION) / 0xffffffff);
	 long ns = (BILLION * (timestamp >> 32));
	return s + ns;
}
/* Formats the timespec structure into the time structure used by NTP servers */
 long time_to_NTP(struct timespec timestamp){
	 long sec = (timestamp.tv_sec << 32);
	 long ns_frac = timestamp.tv_nsec * 0xffffffff / BILLION;
	return sec | ns_frac;
}

int main(int argc, char* argv[]){
	char chosen_server[500];
	int lowest_stratum = 1000;
	int j;
	for (j = 4; j >= 0; j--) {
		struct timespec t1_time, t4_time;
		double sec, ns;

		header_t header;
		memset(&header, 0, sizeof(header_t));
		times_t timestamps;

		char* node = server_list[j];
		printf("------------\n");
		printf("Testing server %s\n", server_list[j]);
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

		// Start time
	  	if( clock_gettime( CLOCK_REALTIME, &t1_time) == -1 ) {
			perror( "clock gettime" );
			exit( EXIT_FAILURE );
		}

		/* Version set to 4 and Mode to 3 (Client), rest to 0 */
		header.control[0] = 35;
		header.control[1] = 0;
		header.control[2] = 0;
		header.control[3] = 0;
		int i;
		printf("Control: ");
		for (i = 0; i < 4; i++)
			printf("%x|", header.control[i]);
		header.origin_t = time_to_NTP(t1_time);
		printf("\nSending header...\n");
		int send_res;
		send_res = sendto(sockfd, &header, sizeof(header_t), 0, serverinfo->ai_addr, serverinfo->ai_addrlen);

		printf("Sent\n");

		// Prepare receive
		timestamps.t1 = header.origin_t;
		printf("Waiting to receive header...\n");
		//header_t rec_header;
		int bytes_rec;
		bytes_rec = recvfrom(sockfd, &header, sizeof(header_t), 0, serverinfo->ai_addr, &recmsg.ai_addrlen);
		printf("Received, result: %d\n", bytes_rec);

		// End time
		if( clock_gettime( CLOCK_REALTIME, &t4_time) == -1 ) {
			perror( "clock gettime" );
			exit( EXIT_FAILURE );
		}
		// Use netohost() to convert from the network byte order to host byte order!
		close(sockfd);
		timestamps.t2 = net_to_host(header.receive_t) - ((long )(2208988800) << 32);
		timestamps.t3 = net_to_host(header.transmit_t) - ((long )(2208988800) << 32);
		timestamps.t4 = time_to_NTP(t4_time);
		/*
		printf("Timestamps:\n");
		printf("t1: %u %u\n", (int)(timestamps.t1 >> 32), (int)(timestamps.t1));
		printf("t2: %u %u\n", (int)(timestamps.t2 >> 32), (int)(timestamps.t2));
		printf("t3: %u %u\n", (int)(timestamps.t3 >> 32), (int)(timestamps.t3));
		printf("t4: %u %u\n[all ns above were ns_frac]\n", (int)(timestamps.t4 >> 32), (int)(timestamps.t4));
		*/
		long delay = NTP_to_ns(get_delay(timestamps));
		long offset = NTP_to_ns(get_offset(timestamps));
		printf("Delay: %u s %u ns\n", (int)(delay >> 32), (int)(delay));
		printf("Offset: %d s  %d ns\n", (int)(offset >> 32), (int)(offset));
		printf("Stratum: %d\n", header.control[1]);
		if (header.control[1] < lowest_stratum) {
			lowest_stratum = header.control[1];
			strcpy(chosen_server, server_list[j]);
		}
		printf("------------\n");
	}
	printf("Chosen server with Stratum = %d: %s\n", lowest_stratum, chosen_server);
	return 0;
}
