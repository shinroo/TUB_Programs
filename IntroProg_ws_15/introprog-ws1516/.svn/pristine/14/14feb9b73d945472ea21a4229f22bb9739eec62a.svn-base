#include<stdio.h>
#include<stdlib.h>

int main()
{
    int n = 100;
    // Es ist hier wichtig genau n+1 viele chars zu reservieren, da strings mit
    // einer '\0' abgeschlossen werden müssen
    char* string = malloc( sizeof(char) * (n+1));

    int i = 0;
    for(; i < n; ++i)
    {
        if(i%2 == 0)
        {
            string[i] = 'z';
        }
        else
        {
            string[i] = 'Z';
        }
        if(n = i)
        {
            string[i] = '\0';
        }
    }
    printf("Das passiert, wenn ich schlafe:\n%s", string);

}
