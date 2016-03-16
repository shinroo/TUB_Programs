#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include "input_blatt07.h"

//idea found on http://stackoverflow.com/questions/13084236/function-to-remove-spaces-from-string-char-array-in-c
/*this code was adapted to not only remove spaces from a string but all whitespace using the isspace() function*/
char * deblank(char *str){
  //initialise local pointers
  char *out = str, *put = str;

  //remove whitespace in string
  for(; *str != '\0'; ++str){
    if(!(isspace(*str)))
      *put++ = *str;
  }

  //delimit string
  *put = '\0';

  return out;
}

//idea found on http://stackoverflow.com/questions/19555434/extract-a-substring-from-a-string-in-c
/*this code extracts a substring between quotes from a string using pointers*/
void extractBetweenQuotes(char* s, char* dest){
   //initialise local variables
   int inQ = 0;
   *dest = 0;

   //find and copy text between quotation marks
   while(*s != 0){
      if(inQ){
         if(*s == '"') return;
         dest[0]=*s;
         dest[1]=0;
         dest++;
      }
      else if(*s == '"') inQ=1;
      s++;
   }

   /*inQ is basically a boolean variable which is true when the pointer is currently between quotation marks*/
}

int main(int argc, char *argv[]){

  //check if the program was called correctly
  if(argc < 2 || argc > 3){
   fprintf(stdout, "usage: ./introprog_blatt07_aufgabe01 <input-file> [output-file]\n");
   exit(2);
  }

  //declare and open files
  FILE *fp_out;
  FILE *fp_in = fopen(argv[1], "r");

  //check if input file exists
  if(fp_in == NULL){
    perror("Could not open input file!");
    return(1);
  }

  //if argc is 3 it means an output file was specified
  if(argc == 3){
    fp_out = fopen(argv[2], "w");

    //check if output file exists
    if(fp_out == NULL){
      perror("Could not open output file!");
      return(1);
    }
  }

  //variable declarations
  char *str = (char*) malloc (sizeof(char)*MAX_LEN_LINE);
  char *comp = (char*) malloc (sizeof(char)*MAX_LEN_LINE);
  char *errors = (char*) malloc (sizeof(char)*MAX_LEN_LINE);
  char *character = (char*) malloc (sizeof(char)*MAX_LEN_LINE);
  char *fq = (char*) malloc (sizeof(char)*MAX_LEN_LINE);
  char *lq = (char*) malloc (sizeof(char)*MAX_LEN_LINE);
  char *temp = (char*) malloc (sizeof(char)*MAX_LEN_LINE);
  char *copy = (char*) malloc (sizeof(char)*MAX_LEN_LINE);
  int width = 0;
  int height = 0;
  char *message = (char*) malloc (sizeof(char)*MAX_LEN_LINE);
  int numline = 0;

  //create tree and initialise with default parameters
  parameters *tree = (parameters*) malloc(sizeof(parameters));
	init_params_with_defaults(tree);

  //read data from parameter file
  while(fgets(str,MAX_LEN_LINE,fp_in)){

    //increment line counter
    numline++;

    //interpret string
    if(str != NULL){

      strcpy(temp,str);
      strcpy(copy,str);
      strcpy(str,deblank(copy));

      //if it is the character
      if( sscanf(str, "%[character]%1s%s", comp, character, errors) == 2 ){
        if (strcmp(comp,"character") == 0){
          tree->character = character[0];
        }
        else{
          fprintf(stderr, "Error while reading line %d\n", numline);
        }
      }

      //if it is the width
      else if( sscanf(str, "%[width]%d%s", comp, &width, errors) == 2 ){
        if (strcmp(comp,"width") == 0){
          //check if width is within acceptable limits
          if(width >= 10 && width <= 60){
				    tree->width = width;
			    }
          else{
				    fprintf(stderr, "Error while reading line %d\n", numline);
          }
        }
        else{
          fprintf(stderr, "Error while reading line %d\n", numline);
        }
      }

      //if it is the height
      else if( sscanf(str, "%[height]%d%s", comp, &height, errors) == 2 ){
        if (strcmp(comp,"height") == 0){
          //check if height is within acceptable limits
          if(height >= 15 && height <= 80){
            tree->height = height;
          }
          else{
            fprintf(stderr, "Error while reading line %d\n", numline);
          }
        }
        else{
          fprintf(stderr, "Error while reading line %d\n", numline);
        }
      }


      //if it is the message
      else if( sscanf(str, "%[message]%[\"]%[^\"]%[\"]%s", comp, fq, message, lq, errors) == 4 ){
        if (strcmp(comp,"message") == 0){

          //extract correct message from temp string
          extractBetweenQuotes(temp,message);
          /* we use this function, because we dealt with white space by removing it;
            however this also removes the whitespace from the message, which is
            unintended. this function retrieves the correct message from a copy
            of the string which was originally read */

          //check if message is the right length
          if(strlen(message)<MAX_LEN_MESSAGE){
            sprintf(tree->message,"%s",message);
          }

        }
        else{
          fprintf(stderr, "Error while reading line %d\n", numline);
        }
      }

      //ignore blank lines and eof
      else if( (sscanf(str, " %s ", errors) == 0) || (sscanf(str, "%s", errors) == EOF)){
        //do nothing
      }

      //handle errors
      else{
        fprintf(stderr, "Error while reading line %d\n", numline);
      }
    }
    else{
      fprintf(stderr, "Error while reading line %d\n", numline);
    }
  }

  //output tree
  if(argc == 3){
    //print to ouput file
    print_tree(tree, fp_out);
  }
  else{
    //print to stdout
    print_tree(tree, stdout);
  }

  //free tree
  free(tree);

  //free malloc'd variables
  free(str);
  free(comp);
  free(errors);
  free(character);
  free(message);
  free(fq);
  free(lq);
  free(temp);
  free(copy);

  //close files
  if(fp_in != NULL) fclose(fp_in);
  if((fp_out!= NULL) && (argc == 3)) fclose(fp_out);

  return(0);

}
