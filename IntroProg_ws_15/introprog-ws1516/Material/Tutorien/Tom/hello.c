#include <stdio.h>
int printf(const char *, ...);

void
say_hello(char *name)
{
	fprintf(stdout, "Hello, %s!\n", name);
}

void
say_bye(char *name)
{
	printf("Bye bye, %s!\n", name);
}
