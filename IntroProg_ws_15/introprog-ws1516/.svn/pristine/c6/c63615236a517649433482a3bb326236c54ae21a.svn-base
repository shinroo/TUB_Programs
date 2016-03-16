#include "blatt05.h"

FILE* read_line_from_file(char *filename, int *phone, char *name, FILE *fpointer) {
    if (fpointer == NULL) {
        fpointer = fopen(filename, "r");
    }
    if (fpointer == NULL){
        perror(filename);
        exit(1);
    }
    char line[MAX_STR];
    char *delim = ";";
    char *_name;
    char *pos;

    if (fgets(line, MAX_STR, fpointer)!=NULL)
    {
        if ((pos = strchr(line, '\n')) != NULL)
            *pos = '\0';
        *phone = atol(strtok(line, delim));
        _name = strtok(NULL, delim);

        strncpy(name, _name, strlen(_name));
        name[strlen(_name)] = '\0';
        return fpointer;
    }
    fclose(fpointer);
    fpointer = NULL;
    return NULL;
}

int is_pos_int_number(char *string) {
    int len = strlen(string);
    for (int i=0; i<=len; i++) {
        if (string[0] >= '0' && string[0] <= '9') {
            return 0;
        }
    }
    return 1;
}

void strncpy_or_null(char *dst, char *src) {
    if (src != NULL && strlen(src) < MAX_STR) {
        strncpy(dst, src, strlen(src));
        dst[strlen(src)] = '\0';
    } else {
        dst[0] = '\0';
    }
}

int atoi_with_sanity(char *src) {
    if (src != NULL && strlen(src) < 5 && is_pos_int_number(src) == 0) {
        return atoi(src);
    }
    return -1;
}

int min(int a, int b) {
    if (a < b) { return a; }
    return b;
}

int read_line_from_stdin(char *operation, int *number, char *name) {
    char line[MAX_STR];
    char *delim = " ";
    char *pos;
    int copy_len = 0;
    int total_len = 0;
    char *tmp;

    if (fgets(line, MAX_STR, stdin)!=NULL)
    {
        if ((pos = strchr(line, '\n')) != NULL) {
            *pos = '\0';
        }
        strncpy_or_null(operation, strtok(line, delim));
        *number = atoi_with_sanity(strtok(NULL, delim));
        while((tmp = strtok(NULL, delim)) != NULL) {
            copy_len = min(MAX_STR-total_len-1, strlen(tmp));
            strncpy(&(name[total_len]), tmp, copy_len);

            // reintroduce space
            name[total_len+copy_len] = ' ';
            total_len++;
            name[total_len+copy_len] = '\0';
            total_len += copy_len;
            if (total_len+1 >= MAX_STR) {
                break;
            }
        }
        return 0;
    }
    return -1;
}

void dot_print_null(int key, int nullcount, FILE* stream)
{
    fprintf(stream, "    null%d [shape=point];\n", nullcount);
    fprintf(stream, "    %d -> null%d;\n", key, nullcount);
}

void dot_print_aux(DOT_NODETYPE* node, FILE* stream)
{
    static int nullcount = 0;

    fprintf(stream, "    %d [label=\"%s\\n(%d)\"];\n", node->DOT_KEY, node->DOT_VALUE,  node->DOT_KEY);
    if (node->DOT_LCHILD)
    {
        fprintf(stream, "    %d -> %d [label=\"<\"];\n", node->DOT_KEY, node->DOT_LCHILD->DOT_KEY);
        dot_print_aux(node->DOT_LCHILD, stream);
    }
    else
       dot_print_null(node->DOT_KEY, nullcount++, stream);

    if (node->DOT_RCHILD)
    {
        fprintf(stream, "    %d -> %d [label=\">\"];\n", node->DOT_KEY, node->DOT_RCHILD->DOT_KEY);
        dot_print_aux(node->DOT_RCHILD, stream);
    }
    else
       dot_print_null(node->DOT_KEY, nullcount++, stream);

    if (node->DOT_PARENT)
    {
        if (node->DOT_PARENT->DOT_LCHILD != node && node->DOT_PARENT->DOT_RCHILD != node)
        {
           fprintf(stream, "    %d -> %s;\n", node->DOT_KEY, "\"broken parent pointer\"");
        }
    }
}

/*
Diese Funktion gibt die übergebene baumstruktur als dot-file aus. Mit
    dot -O -Tpng filename
  können diese dot-files gezeichnet werden.

  (Benötigt graphviz)
*/
void print_to_dot(DOT_NODETYPE* tree, char* filename)
{
    FILE* fd = fopen(filename, "w");
    if (fd == NULL) {
        printf("dot output: could not open file %s", filename);
        return;
    }

    fprintf(fd, "digraph BST {\n");
    fprintf(fd, "    node [fontname=\"Arial\"];\n");

    if (!tree)
        fprintf(fd, "\n");
    else if (!tree->DOT_RCHILD && !tree->DOT_LCHILD)
        fprintf(fd, "    %d;\n", tree->DOT_KEY);
    else
        dot_print_aux(tree, fd);

    fprintf(fd, "labelloc=\"t\"\n");
    //fprintf(fd, "\" %s\"\n", title);
    fprintf(fd, "}\n");
    fclose(fd);
}
