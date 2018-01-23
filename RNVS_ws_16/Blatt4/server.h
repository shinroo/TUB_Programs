#ifndef server_h
#define server_h

#define TEXT_LENGTH 1000

typedef struct node_s{
	struct sockaddr_in pred;
	struct sockaddr_in suc;
	struct sockaddr_in me;
	short pred_id;
	short suc_id;
	short id;
} node_t;

typedef struct __attribute__((packed)) header_s {
	unsigned char control;
	unsigned long ip;
	unsigned short port;
	unsigned short id;
} header_t;

typedef struct __attribute__((packed)) ex_header_s {
	unsigned char control;
	unsigned char int_cont;
	unsigned short key;
	unsigned short value;
	struct sockaddr_in inc_server;
	struct sockaddr_in client;
} ex_header_t;

typedef struct ex_packet_s{
	ex_header_t header;
	char key[TEXT_LENGTH];
	char value[TEXT_LENGTH];
} ex_packet_t;

#endif
