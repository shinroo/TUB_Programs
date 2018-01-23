server: server.c hash_map.c
	gcc hash_map.c server.c -o server

client: client.c
	gcc client.c -o client

debug: server.c hash_map.c client.c
	gcc -g server.c hash_map.c -o server
	gcc -g client.c -o client

test: test_server.c
	gcc test_server.c -o test_server
