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
#define FILMS 25

typedef struct __attribute__((packed)) header_s {
	unsigned char control;
	unsigned char transID;
	short key;
	short value;
} header_t;

int main(int argc, char* argv[]){
	if (argc != 4){
		printf("Expected usage: ./qotd host port filename\n");
		exit(1);
	}

	char* node = argv[1];
	char* service = argv[2];
	char *filename = argv[3];
	int status, i;
	header_t header;

	struct addrinfo hints;
	struct addrinfo *serverinfo;

	FILE *fp;
	size_t leng = 0;
	ssize_t read;
	char *line = NULL;

	if ((fp = fopen(filename, "r")) == NULL) {
		perror("opening file");
		exit(1);
	}

	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;

	if ((status = getaddrinfo(node, service, &hints, &serverinfo)) != 0){
		fprintf(stderr, "Error resolving host name or IP address\n");
		exit(1);
	}

	int sockfd = socket(serverinfo->ai_family, serverinfo->ai_socktype, serverinfo->ai_protocol);
	int conn_res = connect(sockfd, serverinfo->ai_addr, serverinfo->ai_addrlen);


// Get the first 25 lines of the file and convert them to keys and valuse, store them in 2 separate arrays
	int count = 0;
	char *keys[FILMS], *values[FILMS];
	int key_len[FILMS], value_len[FILMS];
	while ((read = getline(&line, &leng, fp)) != -1 && count < FILMS) {
				char key[leng], value[leng];

        sscanf(line, "%[^;];%[^\t]", key, value);

				key_len[count] = strlen(key);
				value_len[count] = strlen(value);

				keys[count] = malloc(key_len[count]+1);
				values[count] = malloc(value_len[count]+1);

				strncpy(keys[count], key, key_len[count]);
				keys[count][key_len[count]] ='\0';
				strncpy(values[count],value, value_len[count]);
				values[count][value_len[count]] = '\0';
				count++;
    }
		fclose(fp);
		/////////// TODO: freeeeeee!!! but not here, at the end :)
    if (line)
      free(line);
			/* send the set-request #FILMS times */
		for (i = 0; i < FILMS; i++) {
			// Set only the control bit to 1
			char control = 2;
			char transID = 0; // by default
			short key_ = (short)(key_len[i]);
			short value_ = (short)(value_len[i]);
			value_ = (value_>>8) | (value_<<8);
			key_ = (key_>>8) | (key_<<8);

			if (send(sockfd, &control, 1, 0) == -1) {
				printf("At iteration %d:", i);
				perror("send: control");
			}

			if (send(sockfd, &transID, 1, 0) == -1) {
				printf("At iteration %d:", i);
				perror("send: transID");
			}
			if (send(sockfd, &key_, 2, 0) == -1) {
				printf("At iteration %d:", i);
				perror("send: key_len");
			}

			if (send(sockfd, &value_, 2, 0) == -1) {
				printf("At iteration %d:", i);
				perror("send: value_len");
			}

			if (send(sockfd, keys[i], key_len[i], 0) == -1) {
				printf("At iteration %d:", i);
				perror("send: key");
			}
			if (send(sockfd, values[i], value_len[i], 0) == -1) {
				printf("At iteration %d:", i);
				perror("send: value");
			}


				// receive acknowledgement data from the server
		}
	void* buf;
	send(sockfd, "", 10, 0);

	int len = MESSAGE_LENGTH;
	buf = (char*) malloc (len);
	int bytes_rec = recv(sockfd, buf, len, 0);
	close(sockfd);

	printf("%s\n", (char*) buf);
}
