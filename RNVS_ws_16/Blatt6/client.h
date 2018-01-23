//Our client header file

#define SERVER "127.0.0.1"
#define PORT "123"

typedef struct __attribute__((packed)) headet_s {
	char control[4];
	unsigned int root_delay;
	unsigned int root_dispersion;
	unsigned int reference_id;
	unsigned long int reference_t;
	unsigned long int origin_t;
	unsigned long int receive_t;
	unsigned long int transmit_t;
} header_t;

typedef struct times_s {
	 long int t1;
	 long int t2;
	 long int t3;
	 long int t4;
} times_t;

char *server_list[] = {
	"0.de.pool.ntp.org",
	"1.de.pool.ntp.org",
	"2.de.pool.ntp.org",
	"3.de.pool.ntp.org",
	"ntps1-0.cs.tu-berlin.de"
};

//long int get_delay(const header_t header);
//long int get_offset(const header_t header);
