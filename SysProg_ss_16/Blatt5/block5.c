/** Licensed under AGPL 3.0. (C) 2010 David Moreno Montero. http://coralbits.com */
#include <onion/onion.h>
#include <onion/log.h>
#include <onion/request.h>
#include <onion/types_internal.h>
#include <onion/types.h>
#include <onion/dict.h>
#include <onion/block.h>
#include <onion/low.h>
#include <onion/url.h>
#include <signal.h>
#include <netdb.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include "cJSON.h"

#define BUF_LEN 1000

void error_message(onion_response *res, const char *message){
	onion_response_set_header(res, "Server", message);
	onion_response_set_code(res, 501);
}

void error(onion_response *res){
	onion_response_set_header(res, "Server",
			"Wop bop a loo bop a lop bom bom! Tutti frutti, oh rutti");
	onion_response_set_code(res, 501);

	printf("Error\n");
}

void reply_ok(onion_response *res){
	onion_response_set_header(res, "Server", "ok?");
	onion_response_set_code(res, 200);
}

char* strip_char(char* source, const char strip){
	int len = strlen(source);
	int new_len = len;
	int i;
	for(i = 0; i < len; i++){
		if(source[i] == strip) new_len--;
	}
	char* stripped = malloc(new_len+1);

	int k = 0;
	for(i = 0; i < len; i++){
		if(source[i] != strip){
			stripped[k] = source[i];
			k++;
		}
	}
	stripped[k] = '\0';

	return stripped;
}
int test(void *p, onion_request *req, onion_response *res){

	onion_response_write0(res,"$ugar what");
	int flag = (int) onion_request_get_flags(req) - 16;
	printf("%s\n", onion_request_get_fullpath(req));
	printf("%d\n", flag);
	switch(flag){
		case 0: printf("GET\n"); break;
		case 1: printf("POST\n"); break;
		case 5: printf("PUT\n"); break;
		case 6: printf("DELETE\n"); break;
	}

	onion_response_printf(res,"<p>Client description: %s",onion_request_get_client_description(req));

	onion_response_set_header(res, "KURWA", "KURWWA");

	return OCS_PROCESSED;
}

char* get_first_path(const char* fullpath){
	int len = strlen(fullpath)-1;
	int i;
	for(i = 1; i < len; i++){
		if(fullpath[i] == '/') break;
	}
	i--;
	char* first_path = malloc(i+1);
	strncpy(first_path, fullpath+1, i); 
	first_path[i] = '\0';
	printf("%s\n", first_path);

	return first_path;

}

char* get_last_path(const char* fullpath){
	int len = strlen(fullpath)-1;
	int i = len;
	for(i; i> 0; i--){
		if(fullpath[i] == '/') break;
	}
	int new_len = len - i;
	char* last_path = malloc(new_len+1);
	strncpy(last_path, fullpath+i+1, new_len); 
	last_path[new_len] = '\0';
	printf("%s\n", last_path);

	return last_path;

}

int countDirs(char* fullpath){

	int len = strlen(fullpath);
	int dircount = 0;
	int i;
	for (i = 0; i < len; i++){
		if (fullpath[i] == '/') dircount++;
	}

	return dircount;
}

int get_collection(onion_request *req, onion_response* res, char* collection){

	FILE *fp = fopen(collection, "r");

	char* buffer = malloc(BUF_LEN);
	fscanf(fp, "%s", buffer);

	cJSON *root = cJSON_Parse(buffer);
	char *formatted_output = cJSON_Print(root);

	//strcpy(res->buffer,formatted_output);
	onion_response_write(res, formatted_output, strlen(formatted_output));

	free(buffer);
	free(formatted_output);

	return 0;
}

int get_element(onion_request *req, onion_response* res, char* collection, char* element){

	FILE *fp = fopen(collection, "r");

	char* buffer = malloc(BUF_LEN);
	fscanf(fp, "%s", buffer);

	cJSON *root = cJSON_Parse(buffer);

	if(cJSON_GetObjectItem(root, element) == NULL)
		return 1;

	cJSON *elem = cJSON_GetObjectItem(root, element);

	char *formatted_output = cJSON_Print(elem);

	//strcpy(res->buffer,formatted_output);
	onion_response_write(res, formatted_output, strlen(formatted_output));

	free(buffer);
	free(formatted_output);

	return 0;

}

int get(onion_request *req, onion_response *res){
	printf("GET\n");

	char* fullpath = onion_request_get_fullpath(req);
	int dirs = countDirs(fullpath);

	char *collection;

	if(dirs == 1)
		collection = strip_char(fullpath,'/');
	else if(dirs ==2)
		collection = get_first_path(fullpath);

	printf("%s %d\n", collection, dirs);

	if(access(collection, F_OK) == -1){
		error_message(res, "Collection doesn't exist");
		return 1;
	}

	if (dirs == 1){
		printf("Getting collection at %s\n", fullpath);
		get_collection(req, res, collection);
	} else if(dirs == 2) {
		printf("Getting element at %s \n", fullpath);
		char* element = get_last_path(fullpath);
		if (get_element(req, res, collection, element) == 1){
			error_message(res, "Element doesn't exist");
			return 1;
		}
	} else {
		error_message(res, "Mate, you fucked up somehow");
		return 1;
	}

	reply_ok(res);

	free(collection);

	return 0;
}



int replace_in_collection(const char *collection, const char *key, const char *value){
	// Open old file
	FILE *fp = fopen(collection, "r"); // fail if not exist

	// Load old info
	char* buffer = malloc(BUF_LEN);
	fscanf(fp, "%s", buffer);

	cJSON *root = cJSON_Parse(buffer);

	// Check if it does exist
	if(cJSON_GetObjectItem(root, key) == NULL){
		return 1;
	}

	cJSON_ReplaceItemInObject(root, key, cJSON_CreateString(value));

	char* unformatted_json = cJSON_PrintUnformatted(root);

	// Create the new json file (collection)
	fp = freopen(collection, "w", fp);
	if(fputs(unformatted_json, fp) == EOF) exit(1);
	fclose(fp); // should be written now

	// Take care of frees
	cJSON_Delete(root);
	free(unformatted_json);
	free(buffer);
	return 0;

}

int put(onion_request *req, onion_response *res){
	printf("PUT\n");
	char* key = get_last_path(onion_request_get_fullpath(req));
	char* collection = get_first_path(onion_request_get_fullpath(req));
	printf("Got: %s,%s\n", key, collection);

	if(access(collection, F_OK) == -1){
		error_message(res, "Collection doesn't exist");
		return 1;
	}

	// Received string
	onion_dict* post_dict = (onion_dict*) onion_request_get_post_dict(req);
	onion_block* post_block = onion_dict_to_json(post_dict);
	const char* rec_string = onion_block_data(post_block);
	int string_length = strlen(rec_string)-6;
	char* temp = malloc(string_length);
	strncpy(temp, rec_string+2, string_length-1);
	temp[string_length-1] = '\0';
	char* value = strip_char(temp, '\\');
	char* value_final = strip_char(value, '\"');
	printf("Final: %s\n", value_final);

	if(replace_in_collection(collection, key, value_final) == 1){
		error_message(res, "Item doesn't exist");
	}else{
		reply_ok(res);
	}

	free(temp);
	free(value);
	free(value_final);
	free(key);
	free(collection);

	onion_block_free(post_block);
	onion_dict_free(post_dict);
	return 0;
}

int delete_item(onion_request *req, onion_response *res){
	printf("DELETE ITEM\n");
	char* key = get_last_path(onion_request_get_fullpath(req));
	char* collection = get_first_path(onion_request_get_fullpath(req));
	printf("Got: %s,%s\n", key, collection);

	if(access(collection, F_OK) == -1){
		error_message(res, "Collection doesn't exist");
		return 1;
	}

	// Open old file
	FILE *fp = fopen(collection, "r"); // fail if not exist

	// Load old info
	char* buffer = malloc(BUF_LEN);
	fscanf(fp, "%s", buffer);

	cJSON *root = cJSON_Parse(buffer);

	// Check if it does exist
	if(cJSON_GetObjectItem(root, key) == NULL){
		error_message(res, "Item doesn't exist");
		return 1;
	}

	cJSON_DeleteItemFromObject(root, key);

	char* unformatted_json = cJSON_PrintUnformatted(root);

	// Create the new json file (collection)
	fp = freopen(collection, "w", fp);
	if(fputs(unformatted_json, fp) == EOF) exit(1);
	fclose(fp); // should be written now

	// Take care of frees
	cJSON_Delete(root);
	free(unformatted_json);
	free(buffer);
	reply_ok(res);
	return 0;
}

int delete_collection(onion_request *req, onion_response *res){
	printf("DELETE COLLECTION\n");
	int len = strlen(onion_request_get_fullpath(req));
	char* collection = malloc(len);
	strncpy(collection, onion_request_get_fullpath(req)+1, len-1);
	collection[len-1] = '\0';
	printf("This is our collection:%s\n",collection);

	if(access(collection, F_OK) == -1){
		error_message(res, "Collection doesn't exist");
		return 1;
	}

	remove(collection);

	if(access(collection, F_OK) == -1){
		reply_ok(res);
		return 0;
	}
	error_message(res, "Something went wrong when deleting, sorry mate\n");
	return 1;
}


int edit_collection(const char* path, const char* json_string){
	// Get the JSON to save
	cJSON *new_item = cJSON_Parse(json_string);

	// Open old file
	FILE *fp = fopen(path, "r"); // fail if not exist
	if(fp == NULL) return 2;

	// Load old info
	char* buffer = malloc(BUF_LEN);
	fscanf(fp, "%s", buffer);
	cJSON *root = cJSON_Parse(buffer);

	// Check if it doesn't exist
	if(cJSON_GetObjectItem(root, new_item->child->string) != NULL){
		return 1;
	}

	cJSON_AddItemToObject(root, new_item->child->string, cJSON_CreateString(new_item->child->valuestring));

	char* unformatted_json = cJSON_PrintUnformatted(root);

	// Create the new json file (collection)
	fp = freopen(path, "w", fp);
	if(fputs(unformatted_json, fp) == EOF) exit(1);
	fclose(fp); // should be written now

	// Take care of frees
	cJSON_Delete(root);
	free(unformatted_json);
	free(buffer);
	return 0;
}

int create_collection(const char* path, const char* json_string){
	// Get the JSON to save
	cJSON *root = cJSON_Parse(json_string);
	char* unformatted_json = cJSON_PrintUnformatted(root);

	// Create the new json file (collection)
	FILE *fp = fopen(path, "w");
	if(fputs(unformatted_json, fp) == EOF) exit(1);
	fclose(fp); // should be written now

	// Take care of frees
	cJSON_Delete(root);
	free(unformatted_json);
	return 0;
}

int post(onion_request *req, onion_response *res){
	printf("POST\n");

	// Received string
	onion_dict* post_dict = (onion_dict*) onion_request_get_post_dict(req);
	onion_block* post_block = onion_dict_to_json(post_dict);
	const char* rec_string = onion_block_data(post_block);
	int string_length = strlen(rec_string)-6;
	char* temp = malloc(string_length);
	strncpy(temp, rec_string+2, string_length-1);
	temp[string_length-1] = '\0';

	char* json_string = strip_char(temp, '\\');
	printf("Final: %s\n", json_string);

	// PATH
	int len = strlen(onion_request_get_fullpath(req));
	char* path = malloc(len);
	strncpy(path, onion_request_get_fullpath(req)+1, len-1);
	path[len-1] = '\0';
	printf("This is our path:%s\n",path);


	// DOES EXIST???
	if (access(path, F_OK) != -1){
		printf("Existing collection\n");
		if(edit_collection(path, json_string) != 0){
			error_message(res, "Item already exists");
		}else{
			reply_ok(res);
		}
	}else{
		printf("New collection\n");
		if(create_collection(path, json_string) != 0){
			error(res);
		}else{
			reply_ok(res);
		}
	}

	free(temp);
	free(json_string);
	free(path);

	onion_block_free(post_block);
	onion_dict_free(post_dict);
}

int fuck_off(void *p, onion_request *req, onion_response *res){
	printf("Gotcha, you sneaky rodent\n");
	onion_response_write0(res,"Invalid URL\n");
	//onion_response_printf(res,"<p>Client description: %s\n",onion_request_get_client_description(req));
	error(res);

	return OCS_PROCESSED;
}

int parse_collection(void *p, onion_request *req, onion_response *res){
	int flag = (int) onion_request_get_flags(req) - 16;
	char buffer[BUF_LEN];
	switch(flag){
		case 0: get(req, res); break;
		case 1: post(req, res); break;
		case 6: delete_collection(req, res); break;
		default: error(res); return 1; /* FUCK OFF */ 
	}

	//onion_response_printf(res,"<p>Client description: %s\n",onion_request_get_client_description(req));

	return OCS_PROCESSED;
}

int parse_element(void *p, onion_request *req, onion_response *res){
	int flag = (int) onion_request_get_flags(req) - 16;
	char buffer[BUF_LEN];
	switch(flag){
		case 0: get(req, res); break;
		case 5: put(req, res); break;
		case 6: delete_item(req, res); break;
		default: error(res); return 1;/* FUCK OFF */ break; 
	}

	//onion_response_printf(res,"<p>Client description: %s\n",onion_request_get_client_description(req));

	return OCS_PROCESSED;
}

onion *o=NULL;

static void shutdown_server(int _){
	if (o)
		onion_listen_stop(o);
}

int main(int argc, char **argv){
	signal(SIGINT,shutdown_server);
	signal(SIGTERM,shutdown_server);

	o=onion_new(O_POOL);
	onion_set_timeout(o, 5000);
	onion_set_hostname(o,"127.0.0.1");
	onion_set_port(o, "8080");
	onion_url *urls=onion_root_url(o);

	onion_url_add(urls, "^$", fuck_off);
	//onion_url_add(urls, "^\\S+\\/\\S+$", parse_element);
	onion_url_add(urls, "^\\S+\\/\\S+\\/\\S+$", fuck_off);
	onion_url_add(urls, "^\\S+\\/\\S+$", parse_element);
	onion_url_add(urls, "^\\S+$", parse_collection);

	onion_listen(o);
	onion_free(o);
	return 0;
}
