#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>

#define URL "www.ub.tu-berlin.de"
#define PORT "80"
#define	MESSAGE_LENGTH 1024 

const char* extract(char keyword[], char* message){
	char temp[32000];
	while(strncmp(message, keyword, strlen(keyword)) != 0){
		strcpy(temp, message);
		sscanf(temp, "%*[^<] %*[<]%32000c", message);
	}
	static char extracted_text[1000];
	sscanf(message, "%*[^>] %*[>] %[^<]", extracted_text);

	return extracted_text;
}

int main(int argc, char* argv[]){
	int status;
	int len = MESSAGE_LENGTH;

	char request[MESSAGE_LENGTH] = "GET /index.php?id=8339&type=100 HTTP/1.1\r\nHost: www.ub.tu-berlin.de\r\n\r\n";

	struct addrinfo hints;
	struct addrinfo *serverinfo;

	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;

	if ((status = getaddrinfo(URL, PORT, &hints, &serverinfo)) != 0){
		fprintf(stderr, "Error resolving host name or IP address\n");
		exit(1);
	}

	int sockfd = socket(serverinfo->ai_family, serverinfo->ai_socktype, serverinfo->ai_protocol);
	int conn_res = connect(sockfd, serverinfo->ai_addr, serverinfo->ai_addrlen);

	int sent_bytes = send(sockfd, request, len, 0);

	char message[32000] = "";
	char* received_message = malloc (2048);

	for(int i = 0; i < 8; i++){ // 8 messages are assumed to be enough
		int bytes_rec = recv(sockfd, received_message, 2048, 0);
		strcat(message, received_message);
	}

	close(sockfd);
	freeaddrinfo(serverinfo);

	// The order of the items cannot be changed
	extract("item", message); // call extract to move to first item tag
	printf("Title: %s\n", extract("title", message));
	printf("Link: %s\n", extract("link", message));
	printf("Description: %s\n", extract("description", message));
	printf("Date: %s\n", extract("pubDate", message));

	free(received_message);

	return 0;
}
