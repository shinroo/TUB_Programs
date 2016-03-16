extern int MAX_LAENGE_ARR;
extern int MAX_LAENGE_STR;

void write_file(char *result[], int len);
int read_file(char *dateiname, char laender[][MAX_LAENGE_STR], char staedte[][MAX_LAENGE_STR], int bewohner []);
