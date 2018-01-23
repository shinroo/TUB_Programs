/*
 ** listener.c -- a datagram sockets "server" demo
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include "server.h"
#include "hash_map.h"

hashtable_t *hashtable;

void test_print(ex_packet_t *ex_packet){
	printf("**********************\n");
	printf("Test printing the ex_packet\n");
	printf("Control byte: %d\n", ex_packet->header.control);
	printf("Internal control: %d\n", ex_packet->header.int_cont);
	printf("Key length: %d\n", ex_packet->header.key);
	printf("Value length: %d\n", ex_packet->header.value);
	printf("Key: %s\n", ex_packet->key);
	printf("Value: %s\n", ex_packet->value);
	printf("**********************\n");
}

int hash(char *str) {
	unsigned long hash = 5381;
	int c;

	while (c = *str++){
		hash = ((hash << 5) + hash) + c; /* hash * 33 + c */
	}
	hash = hash % SIZE;

	return hash;
}

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

short get16bit(unsigned char *array) {
	//printf("GET16BIT: %02x %02x", array[0], array[1]);
	short temp = (short)(((short)(array[0])<<8)+((short)(array[1])));
	short temp1 = (short)(array[0])<<8;
	//printf("   %d, temp1 = %d\n", temp, temp1);
	return temp;
}

void put16bit(short to_put, unsigned char *array) {
	unsigned char *temp;
	temp = (unsigned char *)&to_put;
	array[0] = temp[1];
	array[1] = temp[0];
}

void sendMsg (int sockfd, struct sockaddr_in *where, header_t *header) {
	struct sockaddr_in sendd;
	sendd.sin_family = AF_INET;
	sendd.sin_port = htons(where->sin_port);
	sendd.sin_addr.s_addr = where->sin_addr.s_addr;
	int sent2 = sendto(sockfd, header, sizeof(header_t), 0,
			(struct sockaddr *)&sendd, sizeof(struct sockaddr_in));
	if(sent2 == -1){
		perror("Error: sending message");
	}
	printf("SendMsg: type %x, to: %d, %s\n",header->control, where->sin_port, inet_ntoa(where->sin_addr));
}

void print_n(node_t *me) {
	printf("Me: %s %d %d\n", inet_ntoa(me->me.sin_addr), me->me.sin_port, me->id);
	printf("Suc: %s %d %d\n", inet_ntoa(me->suc.sin_addr), me->suc.sin_port, me->suc_id);
	printf("Pre: %s %d %d\n", inet_ntoa(me->pred.sin_addr), me->pred.sin_port, me->pred_id);
}

int main(int argc, char *argv[])
{
	int sockfd;
	struct addrinfo hints, *servinfo, *p;
	char *my_ip, *my_port, *suc_ip, *suc_port;
	unsigned short id;
	int rv;
	int numbytes;
	struct sockaddr_in their_addr;
	struct sockaddr_in my_addr;
	struct sockaddr_in suc_addr;
	socklen_t addr_len;
	char rob[INET6_ADDRSTRLEN];
	node_t me;

	memset(&hints, 0, sizeof hints);
	hints.ai_family = AF_UNSPEC; // set to AF_INET to force IPv4
	hints.ai_socktype = SOCK_DGRAM;
	hints.ai_flags = AI_PASSIVE; // use my IP

	switch(argc-1){
		case 2: // ip port
			{
				my_ip = argv[1];
				my_port = argv[2];
				inet_aton(my_ip, &my_addr.sin_addr);
				my_addr.sin_port = atoi(my_port);
				id = 0;

				//node shit
				me.pred_id = -1;
				me.suc_id = -1;
				me.id = id;

				break;
			}
		case 3: // ip port id
			{
				my_ip = argv[1];
				my_port = argv[2];
				inet_aton(my_ip, &my_addr.sin_addr);
				my_addr.sin_port = atoi(my_port);
				id = atoi(argv[3]);

				//node shit
				me.pred_id = -1;
				me.suc_id = -1;
				me.id = id;

				break;
			}
		case 5: // ip port ip port id
			{
				my_ip = argv[1];
				my_port = argv[2];
				inet_aton(my_ip, &my_addr.sin_addr);
				my_addr.sin_port = atoi(my_port);
				suc_ip = argv[3];
				suc_port = argv[4];
				inet_aton(suc_ip, &suc_addr.sin_addr);
				suc_addr.sin_port = atoi(suc_port);
				printf("suc_port: %s, %d\n", suc_port, atoi(suc_port));
				id = atoi(argv[5]);

				//node shit
				me.pred_id = -1;
				me.suc_id = -1;
				me.id = id;

				break;
			}
		default:
			{
				printf("usage: %s (ip) (port) [ip] [port] [id]\n", argv[0]);
				exit(1);
			}
	}

	hashtable = ht_create();

	me.me = my_addr;
	if ((rv = getaddrinfo(inet_ntoa(my_addr.sin_addr), my_port, &hints, &servinfo)) != 0) {
		fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
		return 1;
	}

	// loop through all the results and bind to the first we can
	for(p = servinfo; p != NULL; p = p->ai_next) {
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
	int yes = 1;
	setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

	if (p == NULL) {
		fprintf(stderr, "listener: failed to bind socket\n");
		return 2;
	}

	freeaddrinfo(servinfo);
	struct addrinfo *serverinfo;
	if(argc == 6){
		header_t join_header;
		join_header.control = 129; // intern bit and j = 1
		memcpy(&join_header.ip, &my_addr.sin_addr, sizeof(my_addr.sin_addr));
		memcpy(&join_header.port, &my_addr.sin_port, sizeof(my_addr.sin_port));
		//printf("header.port: %d\n",get16bit(join_header.port));
		join_header.id = id;
		addr_len = sizeof(struct sockaddr);
		char buf[sizeof(header_t)];
		memcpy(buf, &join_header, sizeof(header_t));


		int status = getaddrinfo(suc_ip,
				suc_port, &hints, &serverinfo);

		int sent = (sendto(sockfd, buf, sizeof(header_t), 0,
					serverinfo->ai_addr, serverinfo->ai_addrlen));
		if(sent == -1){
			perror("Error: sending join");
		}
		printf("Sent join: %d\n", sent);
	}

	while(1){
		char buffer[10000];
		char control_byte;
		printf("listener: waiting to recvfrom...\n");
		print_n(&me);
		if ((numbytes = recvfrom(sockfd, &buffer, 10000, 0,
						(struct sockaddr *)&their_addr, &addr_len)) == -1) {
			perror("recvfrom - rekt_header");
			continue;
		}

		control_byte = buffer[0];

		printf("listener: packet is %d bytes long\n", numbytes);
		char j = ((control_byte&1) != 0) ? 1 : 0;
		char n = ((control_byte&2) != 0) ? 1 : 0;
		char s = ((control_byte&4) != 0) ? 1 : 0;
		char internal = ((control_byte&128) != 0) ? 1 : 0;
		if (internal == 1) {
			header_t rekt_header;
			rekt_header.control = control_byte;
			memcpy(&rekt_header.ip, &buffer[1], sizeof(long));
			memcpy(&rekt_header.port, &buffer[9], sizeof(short));
			memcpy(&rekt_header.id, &buffer[11], sizeof(short));
			printf("Header ID: %d, my id: %d\nControl %x\n", rekt_header.id, me.id, rekt_header.control);
			printf("%02x\n",rekt_header.control);
			if(!(j ^ n ^ s)){
				continue;
			}
			if (j == 1) {
				//second node added to ring
				if (me.suc_id == -1 && me.pred_id == -1){
					printf("Received first join: j = %d, n = %d\n", j, n);
					me.suc_id = rekt_header.id;
					me.pred_id = rekt_header.id;

					me.suc.sin_addr.s_addr = rekt_header.ip;
					me.suc.sin_port = rekt_header.port;

					me.pred.sin_addr.s_addr = rekt_header.ip;
					me.pred.sin_port = rekt_header.port;

					header_t notify_header;
					notify_header.control = 130; // intern bit and n = 1
					memcpy(&notify_header.ip, &my_addr.sin_addr, sizeof(my_addr.sin_addr));
					memcpy(&notify_header.port, &my_addr.sin_port, sizeof(my_addr.sin_port));
					notify_header.id = me.id;

					struct sockaddr_in sendd;
					sendd.sin_port = rekt_header.port;
					sendd.sin_addr.s_addr = rekt_header.ip;
					sendMsg(sockfd, &sendd, &notify_header);

					printf("Sending stabilize... \n");
					header_t stab_header;
					stab_header.control = 132;
					stab_header.ip = my_addr.sin_addr.s_addr;
					stab_header.port = my_addr.sin_port;
					stab_header.id = me.id;
					sendMsg(sockfd, &me.suc, &stab_header);
					printf("Sent stabilize: %d, %s\n", stab_header.port, inet_ntoa(me.suc.sin_addr));

					// send notify to the caller, so that it knows that now he is the
				}

				else if (rekt_header.id > me.id) {
					if (me.id < me.pred_id && rekt_header.id > me.pred_id) {
						// add to beginning of ring
						printf("Sending notify to joining node\n");
						header_t notify_header;
						notify_header.control = 130; // intern bit and n = 1

						notify_header.ip = rekt_header.ip;
						notify_header.port = rekt_header.port;
						notify_header.id = rekt_header.id;

						struct sockaddr_in sendd;
						sendd.sin_port = me.pred.sin_port;
						sendd.sin_addr.s_addr = me.pred.sin_addr.s_addr;
						sendMsg(sockfd, &sendd, &notify_header);

						notify_header.ip = my_addr.sin_addr.s_addr;
						notify_header.port = my_addr.sin_port;
						notify_header.id = me.id;

						me.pred.sin_addr.s_addr = rekt_header.ip;
						me.pred.sin_port = rekt_header.port;
						me.pred_id = rekt_header.id;

						sendd.sin_port = rekt_header.port;
						sendd.sin_addr.s_addr = rekt_header.ip;
						sendMsg(sockfd, &sendd, &notify_header);
					}else{
						// send to successor
						sendMsg(sockfd, &me.suc, &rekt_header);
						printf("Pass forward: %d, %s\n", rekt_header.port, inet_ntoa(me.suc.sin_addr));
						printf("Sending stabilize... \n");
						header_t stab_header;
						stab_header.control = 132;
						stab_header.ip = my_addr.sin_addr.s_addr;
						stab_header.port = my_addr.sin_port;
						stab_header.id = me.id;
						sendMsg(sockfd, &me.suc, &stab_header);
						printf("Sent stabilize: %d, %s\n", stab_header.port, inet_ntoa(me.suc.sin_addr));
					}
				}else if (rekt_header.id < me.id) {
					if(rekt_header.id > me.pred_id){
						//add
						printf("Sending notify to joining node\n");
						header_t notify_header;
						notify_header.control = 130; // intern bit and n = 1

						notify_header.ip = rekt_header.ip;
						notify_header.port = rekt_header.port;
						notify_header.id = rekt_header.id;

						struct sockaddr_in sendd;
						sendd.sin_port = me.pred.sin_port;
						sendd.sin_addr.s_addr = me.pred.sin_addr.s_addr;
						sendMsg(sockfd, &sendd, &notify_header);

						notify_header.ip = my_addr.sin_addr.s_addr;
						notify_header.port = my_addr.sin_port;
						notify_header.id = me.id;

						me.pred.sin_addr.s_addr = rekt_header.ip;
						me.pred.sin_port = rekt_header.port;
						me.pred_id = rekt_header.id;

						sendd.sin_port = rekt_header.port;
						sendd.sin_addr.s_addr = rekt_header.ip;
						sendMsg(sockfd, &sendd, &notify_header);
					}else if(me.id < me.pred_id){ // beginning of list with new minimum
						printf("Sending notify to joining node\n");
						header_t notify_header;
						notify_header.control = 130; // intern bit and n = 1

						notify_header.ip = rekt_header.ip;
						notify_header.port = rekt_header.port;
						notify_header.id = rekt_header.id;

						struct sockaddr_in sendd;
						sendd.sin_port = me.pred.sin_port;
						sendd.sin_addr.s_addr = me.pred.sin_addr.s_addr;
						sendMsg(sockfd, &sendd, &notify_header);

						notify_header.ip = my_addr.sin_addr.s_addr;
						notify_header.port = my_addr.sin_port;
						notify_header.id = me.id;

						me.pred.sin_addr.s_addr = rekt_header.ip;
						me.pred.sin_port = rekt_header.port;
						me.pred_id = rekt_header.id;

						sendd.sin_port = rekt_header.port;
						sendd.sin_addr.s_addr = rekt_header.ip;
						sendMsg(sockfd, &sendd, &notify_header);
					}else{
						// send to successor
						sendMsg(sockfd, &me.suc, &rekt_header);
						printf("Pass forward: %d, %s\n", rekt_header.port, inet_ntoa(me.suc.sin_addr));
						printf("Sending stabilize... \n");
						header_t stab_header;
						stab_header.control = 132;
						stab_header.ip = my_addr.sin_addr.s_addr;
						stab_header.port = my_addr.sin_port;
						stab_header.id = me.id;
						sendMsg(sockfd, &me.suc, &stab_header);
						printf("Sent stabilize: %d, %s\n", stab_header.port, inet_ntoa(me.suc.sin_addr));
					}

				}else{
					exit(1);
				}
			}
			if (n == 1) {
				printf("received notify!!!\n");

				if(me.suc_id == -1){
					me.suc.sin_addr.s_addr = rekt_header.ip;
					me.suc.sin_port = rekt_header.port;
					me.suc_id = rekt_header.id;
					printf("me.suc.ip: %s\nme.suc.port: %d\nme.suc_id: %d\n", inet_ntoa(me.suc.sin_addr), me.suc.sin_port, me.suc_id);
				}else if(rekt_header.id != me.id){
					me.suc.sin_addr.s_addr = rekt_header.ip;
					me.suc.sin_port = rekt_header.port;
					me.suc_id = rekt_header.id;
					printf("me.suc.ip: %s\nme.suc.port: %d\nme.suc_id: %d\n", inet_ntoa(me.suc.sin_addr), me.suc.sin_port, me.suc_id);

					header_t stab_header;
					stab_header.control = 132; // stabilize
					stab_header.ip = my_addr.sin_addr.s_addr;
					stab_header.port = my_addr.sin_port;
					stab_header.id = me.id;

					struct sockaddr_in sendd;
					sendd.sin_port = rekt_header.port;
					sendd.sin_addr.s_addr = rekt_header.ip;
					sendMsg(sockfd, &sendd, &stab_header);

					printf("Sent stabilize: %d, %s\n", stab_header.port, inet_ntoa(me.suc.sin_addr));

				}
			}
			if (s == 1) {
				printf("received stabilize!!!\n");

				if(me.pred_id == -1){
					me.pred.sin_addr.s_addr = rekt_header.ip;
					me.pred.sin_port = rekt_header.port;
					me.pred_id = rekt_header.id;
					printf("me.pred.ip: %s\nme.pred.port: %d\nme.pred_id: %d\n", inet_ntoa(me.suc.sin_addr), me.pred.sin_port, me.pred_id);
				}else{
					header_t notify_header;
					notify_header.control = 130; // intern bit and n = 1
					notify_header.ip = me.pred.sin_addr.s_addr;
					notify_header.port = me.pred.sin_port;
					notify_header.id = me.pred_id;

					struct sockaddr_in sendd;
					sendd.sin_port = rekt_header.port;
					sendd.sin_addr.s_addr = rekt_header.ip;
					sendMsg(sockfd, &sendd, &notify_header);
				}
			}
		}else if(internal == 0){
			printf("\n--------------------");
			/* Helper values */ 
			int i_addr_len = sizeof(struct sockaddr_in);
			int size_ex_header = sizeof(ex_header_t);

			printf("Got external packet\n");

			/* Declaring ex_header in which to store incoming buffer */ 
			ex_header_t ex_header;
			ex_packet_t ex_packet;
			memset(&ex_header, 0, sizeof(ex_header_t));
			memset(&ex_packet, 0, sizeof(ex_packet_t));

			/* 1. Control byte */
			ex_header.control = control_byte;
			//printf("Control byte is: %x\n", ex_header.control);

			/* 2. Internal control byte */ 
			ex_header.int_cont = buffer[1];
			//printf("Internal byte is: %x\n", ex_header.int_cont);

			/* 3. Key length */
			short temp_key_s;
			memcpy(&temp_key_s, &buffer[2], 2);
			ex_header.key = temp_key_s;
			//printf("The key length we got is: %d\n", ex_header.key);

			/* 4. Value length */
			short temp_val_s;
			memcpy(&temp_val_s, &buffer[4], 2);
			ex_header.value = temp_val_s;
			//printf("The key length we got is: %d\n", ex_header.value);

			/* 5. Incoming server sockaddr */
			memcpy(&ex_header.inc_server, &buffer[6], i_addr_len);

			/* 6. Client sockaddr */
			memcpy(&ex_header.client, &buffer[6+i_addr_len], i_addr_len);

			/* Parsing control byte instructions */
			char del = ((ex_header.control&1) != 0) ? 1 : 0;
			char set = ((ex_header.control&2) != 0) ? 1 : 0;
			char get = ((ex_header.control&4) != 0) ? 1 : 0;
			char ack = ((ex_header.control&8) != 0) ? 1 : 0;
			//printf("set %d \nget %d\ndelete %d\nack %d\n", set, get, del, ack);

			/* Parsing internal control byte instructions */
			char ready = ((ex_header.int_cont&2) != 0) ? 1 : 0;
			char present_client = ((ex_header.int_cont&1) != 0) ? 1 : 0;

			/* Parse key text if any, sanitize if not */
			char key[ex_header.key+1];
			if(ex_header.key > 0){
				strncpy(key, &buffer[size_ex_header], ex_header.key);
				strncpy(&key[ex_header.key], "\0", 1);
			}else if(ex_header.key == 0){
				strncpy(key, "", 1);
			}
			//printf("Key: %s\n", key);

			/* Parse value text if any, sanitize if not */
			char value[ex_header.value+1];
			if(ex_header.value > 0){
				strncpy(value, &buffer[size_ex_header+TEXT_LENGTH], ex_header.value);
				strncpy(&value[ex_header.value], "\0", 1);
			}else if(ex_header.value == 0){
				strncpy(value, "", 1);
			}
			//printf("Value: %s\n", value);

			/* Building the ex_packet */
			ex_packet.header = ex_header;
			strncpy(ex_packet.key, key, ex_header.key);
			strncpy(ex_packet.value, value, ex_header.value);
			if(!present_client){
				ex_packet.header.client = their_addr;
				ex_packet.header.inc_server = my_addr;
				ex_packet.header.int_cont = 1;
			}

			/* Test printing the ex_packet */
			
			//printf("TEST PRINT BEFORE EVALUATE\n");
			//test_print(&ex_packet);

			/* Calculate hash to see if this node has to evaluate the packet */
			int hash_value = hash(key);
			printf("HASH value is: %d\n", hash_value);

			/* Check if hash value is between me and successor */
			int hash_within_range = ((hash_value >= me.id) && (hash_value < me.suc_id));
			/* Check if hash value is bigger than me and if the successor is smaller */
			int hash_at_end = ((hash_value >= me.id) && (me.id > me.suc_id));
			/* Current node is responsible for hash value */
			int responsible_for_hash = hash_within_range || hash_at_end;

			/* If it does, evaluate it */
			if(responsible_for_hash && !ready){
				/* Evaluate the received packet */

				/* Set the Acknowledgement Bit to 1 */
				ex_packet.header.control = ex_packet.header.control|8;

				/* Received set!
				 * it's assumed that set cannot fail
				 */
				if (set == 1){
					hash_set(ex_packet.key, ex_packet.value);
				}

				/* Received get! */
				char *get_pointer = NULL;
				char *get_buffer = NULL;
				if (get == 1){
					get_pointer = hash_get(ex_packet.key);
					if(get_pointer == NULL){
						ex_packet.header.control = ex_packet.header.control^8;
					}else{
						//printf("Strlen(get_pointer) is %d\n", strlen(get_pointer));
						get_buffer = malloc((strlen(get_pointer)+1)*sizeof(char));
						strcpy(get_buffer, get_pointer);
					}
				}

				/* Received delete!
				 * it's assumed that trying to
				 * delete a non-existing pair
				 * is not a failure
				 */
				if (del == 1){
					hash_delete(ex_packet.key);
				}

				/* Check if operation was successful */
				char ack_local = ((ex_packet.header.control&8) != 0) ? 1 : 0;

				/* Check if get was successful 
				 * and set value length accordingly
				 */
				if(get == 1 && ack_local == 1){
					short s_value = strlen(get_buffer);
					printf("We need %d memory!\n", s_value);
					ex_packet.header.value = s_value;
					strncpy(ex_packet.value, get_buffer, strlen(get_buffer));
					free(get_buffer);
				}else{
					ex_packet.header.key = strlen(ex_packet.key);
					ex_packet.header.value = 0;
					strncpy(ex_packet.value, "", 1);
				}

				/* Set ready bit to 1 */
				ex_packet.header.int_cont = 3;

				/* Check if current node is origin server */
				int ports_match = (my_addr.sin_port == ex_packet.header.inc_server.sin_port);
				uint32_t my_ip_check = my_addr.sin_addr.s_addr;
				uint32_t inc_ip_check = ex_packet.header.inc_server.sin_addr.s_addr;
				int ips_match = (my_ip_check == inc_ip_check); 
				int addrs_match = (ips_match && ports_match);
				printf("The ips are: %s and %s\n", inet_ntoa(my_addr.sin_addr), inet_ntoa(ex_packet.header.inc_server.sin_addr));
				printf("The ports are: %u and %u\n", my_addr.sin_port , ex_packet.header.inc_server.sin_port);

				/* If we are origin server 
				 * Send the packet back to the client
				 */
				if(addrs_match){
					printf("We're the origin server!\n");
					struct sockaddr_in sendd;
					sendd.sin_family = AF_INET;
					sendd.sin_port = ex_packet.header.client.sin_port;
					sendd.sin_addr.s_addr = ex_packet.header.client.sin_addr.s_addr;
					printf("Client ip: %s\n",inet_ntoa(ex_packet.header.client.sin_addr));
					printf("Client port: %d\n", ex_packet.header.client.sin_port);
					printf("This is what we're sending back\n");
					//test_print(&ex_packet);
					int sent_bytes = sendto(sockfd, &ex_packet, sizeof(ex_packet_t), 0,
							(struct sockaddr *)&sendd, sizeof(struct sockaddr_in));
					if(sent_bytes == -1){
						perror("Error: sending external message");
					}
					printf("Package is on its way!\n");
					/* Send back to origin server */
				}else{
					printf("Send back to the origin server!\n");
					struct sockaddr_in sendd;
					sendd.sin_family = AF_INET;
					sendd.sin_port = htons(ex_packet.header.inc_server.sin_port);
					sendd.sin_addr.s_addr = ex_packet.header.inc_server.sin_addr.s_addr;
					int sent_bytes = sendto(sockfd, &ex_packet, sizeof(ex_packet_t), 0,
							(struct sockaddr *)&sendd, sizeof(struct sockaddr_in));
					if(sent_bytes == -1){
						perror("Error: sending external message");
					}

				}
			}else if(!ready){
				// weiterschicken
				printf("Sending to successor!\n");
				struct sockaddr_in sendd;
				sendd.sin_family = AF_INET;
				sendd.sin_port = htons(me.suc.sin_port);
				sendd.sin_addr.s_addr = me.suc.sin_addr.s_addr;
				int sent_bytes = sendto(sockfd, &ex_packet, sizeof(ex_packet_t), 0,
						(struct sockaddr *)&sendd, sizeof(struct sockaddr_in));
				if(sent_bytes == -1){
					perror("Error: sending external message");
				}
			}

			if(ready){
				printf("Sending info back to client\n");
				// back to client
				struct sockaddr_in sendd;
				sendd.sin_family = AF_INET;
				sendd.sin_port = (ex_header.client.sin_port);
				sendd.sin_addr.s_addr = ex_header.client.sin_addr.s_addr;
				int sent2 = sendto(sockfd, &ex_packet, sizeof(ex_packet_t), 0,
						(struct sockaddr *)&sendd, sizeof(struct sockaddr_in));
				if(sent2 == -1){
					perror("Error: sending external message");
				}
			}
		}
		printf("\n\n-----------------\n\n");
	}

	close(sockfd);

	return 0;
}
