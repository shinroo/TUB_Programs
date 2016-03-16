#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_STR 255

int main() {

	char * str = "Hallo Welt!";

	printf("%s\n", str);

	char str_new[MAX_STR];

	int i;
	for(i = 0; str[i] != '\0' && i < MAX_STR-1; i++) {
		str_new[i] = str[i];
	}
	str_new[i] = '\0';

	printf("%s\n", str_new);

	char str_alt[MAX_STR];

	strncpy(str_alt, str, MAX_STR-1);
	str_alt[MAX_STR-1] = '\0';

	printf("%s\n", str_alt);
}