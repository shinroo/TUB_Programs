#include "input_blatt07.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>  


//Initialisiert ein bereits alloziertes parameter struct mit Defaultwerten
void init_params_with_defaults(parameters* params)
{
    params->width = 42;
    params->height = 17;
    strcpy(params->message, "There's no place like home.");
    params->character = '*';
}

//Überprüft ob die übergebenen Parameter valide Werte enthält
int params_are_valid(parameters* params)
{
    if(params->width < 10 || params->width > 60)
    {
        return 0;
    }
    if(params->height < 15 || params->height > 80)
    {
        return 0;
    }
    if(isspace(params->character))
    {
        return 0;
    }
    
    int non_whitespace = 0;
    //make sure to find a non whitespace character and a '\0' afterwards
    for(int i = 0; i < MAX_LEN_MESSAGE; ++i)
    {
        if(!isspace(params->message[i]))
        {
            non_whitespace = 1;
        }
        if(non_whitespace == 1 && params->message[i] == '\0')
        {
            return 1;
        }
    }
    return 0;
}

//Gibt die Weihnachts-Karte gemäß der Parameter params auf dem File-Stream file aus
void print_tree(parameters* params, FILE* file)
{
    fprintf(file, "Dumping parameters..\n");
    fprintf(file, "\t %10s: <%d>\n",    "width",        params->width);
    fprintf(file, "\t %10s: <%d>\n",    "height",       params->height);
    fprintf(file, "\t %10s: <%c>\n",    "character",    params->character);
    fprintf(file, "\t %10s: <%s>\n\n",  "message",      params->message);
    
    //überprüfe ob die Parameter valide sind
    if(!params_are_valid(params))
    {
        //beende das gesamt Programm, falls das nicht der Fall ist
        exit(1);
    }
    
    
    //Es folgen einige Berechnungen..
    
    int width = params->width;
    int height = params->height;
    
    if(width % 2 == 0)
    {
        //wir erzwingen, dass die Breite ungerade ist
        width++;
    }
    
    //Berechne die Breite und Höhe des Baumstumpfes
    int stump_width  = (int)(ceil(width * 0.1));
    if(stump_width % 2 == 0)
    {
        //auch wieder ungerade Breite..
        stump_width++;
    }
    int stump_height = (int)(ceil(sqrt(width * height) * 0.05)); 
    
    //wirkliche Höhe und Breite für die String Buffer
    int total_height = height + stump_height + 1;   //für das Zeichen auf der Spitze
    int total_width = width + 2;    //für die Zeichen \n\0
    
    //Speicher für die erzeugten Zeilen
    char** buffer_to_write = malloc(sizeof(char*) * (total_height));
    
    //Berechne jede Zeile der Ausgabe
    for(int y = 0; y < total_height; ++y)
    {
        //alloziere Speicher
        buffer_to_write[y] = malloc(sizeof(char) * total_width);

        //initialisiere den Speicher mit Leerzeichen ...
        for(int i = 0; i < width; ++i) buffer_to_write[y][i] = ' ';
        //... und beende den String mit '\n', '\0'
        buffer_to_write[y][total_width -2] = '\n';
        buffer_to_write[y][total_width -1] = '\0';
        
        //Mitte der Ausgabe
        int middle = width / 2;
        
        if(y == 0)
        {
            //Schreibe das Zeichen params->character auf die Spitze
            buffer_to_write[y][middle] = params->character;
        }
        else if(y <= height)
        {
            //Dieser Code schreibt den Baum (ohne Stumpf)
            
            //berechne die Anzahl an Feldern links und rechts vom Baum, welche 'begrünt' sind
            int greens = floor((width / 2.0) / height * y);
            
            //schreibe an die Mitte des Baums immer ein | ...
            buffer_to_write[y][middle] = '|';
            
            //... und links und rechts immer ein slash bzw. ein backslash
            
            for(int i = 1; i <= greens; ++i)
            {
                buffer_to_write[y][middle - i] = '/';
                buffer_to_write[y][middle + i] = '\\';
            }
            
            //schmücke den Baum mit Kerzen!
            if(y > 2 && y <= height)
            {
                if(y % 2 == 0)
                {                
                    //platziere Kerzen auf einer Ebene weiter oben
                    buffer_to_write[y-1][middle + greens] = 'i';
                    buffer_to_write[y-1][middle - greens] = 'i';
                }
            }
        }
        //schreibe den Baumstumpf
        else if(y < total_height)
        {
            buffer_to_write[y][middle] = '|';
            for(int i = 1; i <= stump_width / 2; ++i)
            {
                buffer_to_write[y][middle+i] = '|';
                buffer_to_write[y][middle-i] = '|';
            }
        }
    }
    //Um den Baum abzurunden, schneiden wir etwas des gezeichneten Baums wieder weg
    //Konkret: gemäß einer Parabelgleichung (in Abhängigkeit von der Entfernung zum Baumstumpf)
    //wird berechnet wieviele Zeichen zu entfernen sind.
    double d_dx = width/2.0 - stump_width/2.0;
    double d_stump_height = (double) stump_height;
    for(int x = 0; x < width/2 - stump_width/2 - 1; ++x)
    {
        int y_offset = (int)ceil(d_stump_height/(d_dx*d_dx) * x * x - 2.5 * d_stump_height * x/ d_dx + 1*d_stump_height); 
        for(int y = height-y_offset; y <= height; ++y)
        {
            buffer_to_write[y][x] = ' ';
            buffer_to_write[y][width-x-1] = ' ';
        } 
    }
    
    //nachdem alles für die Ausgabe vorbereitet wurde, werden die berechneten
    //Zeilen nacheinander ausgegeben und der allozierte Speicher freigegeben
    for(int y = 0; y < total_height; ++y)
    {
        fprintf(file, "%s", buffer_to_write[y]);
        free(buffer_to_write[y]);
    }
    free(buffer_to_write);
    fprintf(file, "\n");
    
    //Wir geben die Nachricht unter dem Weihnachtsbaum aus.
    //Damit es schöner aussieht versuchen wir die Nachricht zu zentrieren,
    //indem wir eine Anzahl an Leerzeichen ausgeben.
    int lala = (width - strlen(params->message)) / 2;
    for(int i = 0; i < lala; ++i)
    {
        fprintf(file, " ");
    }
    //Ausgabe der Nachricht..
    fprintf(file, "%s\n", params->message);
}

