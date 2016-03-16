#include<stdio.h>

// Gibt die Zahl x^x für alle Zahlen zwischen 1 und n aus
//
// 1^1 = 1 = 1
// 2^2 = 2*2 = 4
// 3^3 = 3*3*3 = 27
// 4^4 = 4*4*4*4 = 256
// ...
// n^n
int main()
{
    int n = 20;

    for(int x = 1; x <= n; ++x)
    {
        int f_x = 0;
        int benoetigte_multiplikationen = f_x - 1;
        while(benoetigte_multiplikationen > 0)
        {
            int f_x = x;
            f_x *= f_x;
        }
        printf("%d^%d = %d\n", x,x,x,f_x);
    }

}
