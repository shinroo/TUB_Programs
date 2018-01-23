/*
	Onion HTTP server library
	Copyright (C) 2012 David Moreno Montero

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU Affero General Public License as
	published by the Free Software Foundation, either version 3 of the
	License, or (at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU Affero General Public License for more details.

	You should have received a copy of the GNU Affero General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
	*/

#include <stdlib.h>
#include <onion/log.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <libgen.h>
#include <stdarg.h>

#include "updateassets.h"

#define INSERT_HERE_MARK "// New data will be inserted over this line. DO NOT MODIFY (SPECIALLY THIS LINE)."

struct onion_assets_file_t{
	char *filename;
	char **head;
	char **tail;
	char **lines;
	int lines_count;
	int lines_capacity;
};

// Forces add.
void onion_assets_file_add_line(onion_assets_file *f, const char *line);
void onion_assets_file_add_linef(onion_assets_file *f, const char *line, ...);

void onion_assets_file_add_linef(onion_assets_file *f, const char *line, ...){
	char tmp[1024];
	va_list ap;
	va_start(ap, line);
	vsnprintf(tmp,sizeof(tmp),line, ap);
	va_end(ap);
	onion_assets_file_add_line(f, tmp);
}

// Converts the string to a valid define like string.
static char *to_define_able(const char *str){
	char *FILENAME=strdup(str);
	int i=0;
	while (FILENAME[i]){
		if (isalpha(FILENAME[i]))
			FILENAME[i]=toupper(FILENAME[i]);
		else if (isdigit(FILENAME[i]))
			FILENAME[i]=FILENAME[i];
		else
			FILENAME[i]='_';
		
		i++;
	}
	return FILENAME;
}

static void onion_assets_file_set_head(onion_assets_file *f){
	ONION_DEBUG("Set header mark");
	onion_assets_file_add_line(f,NULL);
	f->head=f->lines;
	f->lines_capacity=16;
	f->lines_count=0;
	f->lines=malloc(sizeof(const char *)*f->lines_capacity);
}
static void onion_assets_file_set_tail(onion_assets_file *f){
	ONION_DEBUG("Found tail mark");
	onion_assets_file_add_line(f,NULL);
	f->tail=f->lines;
	f->lines_capacity=16;
	f->lines_count=0;
	f->lines=malloc(sizeof(const char *)*f->lines_capacity);
}


void onion_assets_file_add_line(onion_assets_file *f, const char *line){
	if (line && !f->head && strcmp(line, INSERT_HERE_MARK)==0){
		onion_assets_file_set_head(f);
	}
	
	if (line){ // Check line not yet in.
		if (f->head){
			char **ll=f->head;
			while (*ll){
				if (strcmp(*ll, line)==0)
					return;
				++ll;
			}
		}
		
		
		int i;
		for (i=0;i<f->lines_count;i++){
			if (strcmp(f->lines[i], line)==0) // Already in
				return;
		}
	}
	
	if (f->lines_count==f->lines_capacity){
		if (f->lines_capacity < 2048) 
			f->lines_capacity*=2;
		else
			f->lines_capacity+=2048;
		f->lines=realloc(f->lines, sizeof(const char *)*f->lines_capacity);
	}
	if (line)
		f->lines[f->lines_count++]=strdup(line);
	else
		f->lines[f->lines_count++]=NULL;
	ONION_DEBUG("Add line: %s", line);
}

onion_assets_file *onion_assets_file_new(const char *filename){
	onion_assets_file *ret=malloc(sizeof(onion_assets_file));
	FILE *file=fopen(filename, "rt");
	ret->filename=strdup(filename);
	ret->lines_count=0;
	ret->lines_capacity=16;
	ret->lines=malloc(sizeof(const char *)*ret->lines_capacity);
	ret->head=NULL;
	ret->tail=NULL;

	if (file){
		char buffer[4096];
		int r=0;
		int o=0;
		int total=0;
		do{
			r=fread(&buffer[o], 1, sizeof(buffer)-o, file);
			int i;
			o=0;
			for (i=0;i<r;i++){
				if (buffer[i]=='\n'){
					buffer[i]=0;
					ONION_DEBUG("Read original line <%s>", &buffer[o]); 
					onion_assets_file_add_line(ret, &buffer[o]);
					total+=strlen(&buffer[o]);
					o=i+1;
				}
			}
		}while(r>0);
		onion_assets_file_set_tail(ret);
		fclose(file);
		if (total>0){
			return ret;
		}
	}
	
	char *FILENAME=to_define_able(filename);
	
	onion_assets_file_add_line(ret,"/* Autogenerated by onion assets */");
	onion_assets_file_add_linef(ret,"#ifndef __ASSETS_H__%s",FILENAME);
	onion_assets_file_add_linef(ret,"#define __ASSETS_H__%s",FILENAME);
	onion_assets_file_add_line(ret,"# ifdef __cplusplus"); 
	onion_assets_file_add_line(ret,"extern \"C\"{");
	onion_assets_file_add_line(ret,"# endif");
	onion_assets_file_add_line(ret,"#include <onion/types.h>");
	onion_assets_file_add_line(ret,INSERT_HERE_MARK);
	onion_assets_file_add_line(ret,"# ifdef __cplusplus "); // Space trick important to allow to dup line.
	onion_assets_file_add_line(ret,"} // extern \"C\"");
	onion_assets_file_add_line(ret,"# endif ");
	onion_assets_file_add_line(ret,"#endif");
	
	onion_assets_file_set_tail(ret);
	
	free(FILENAME);
	return ret;
}

int onion_assets_file_update(onion_assets_file* file, const char* line){
	int i;
	for (i=0;i<file->lines_count;i++){
		if (strcmp(file->lines[i],line)==0)
			return 0;
	}
	onion_assets_file_add_line(file, line);
	return 1;
}

static void onion_assets_write_and_free_until_null(char **lines, FILE *file){
	char **ll=lines;
	while (*ll){
		char *l=*ll;
		ONION_DEBUG("Write: %s", l);
		size_t length=strlen(l);
		size_t wlength=fwrite(l, 1, length, file);
		if (wlength!=length){
			ONION_ERROR("Could not write all data. Aborting");
			abort();
		}
		wlength=fwrite("\n",1, 1, file);
		if (wlength!=1){
			ONION_ERROR("Could not write all data. Aborting");
			abort();
		}
		free(l);
		++ll;
	}
	free(lines);
}

int onion_assets_file_free(onion_assets_file *f){
	onion_assets_file_add_line(f, NULL); // Seal this lines list.
	
	FILE *file=fopen(f->filename,"wt");
	if (!file){
		ONION_WARNING("Could not open %s asset file for updating.", f->filename);
		return -1;
	}
	
	if (f->head)
		onion_assets_write_and_free_until_null(f->head, file);
	onion_assets_write_and_free_until_null(f->lines, file);
	if (f->tail)
		onion_assets_write_and_free_until_null(f->tail, file);
	
	assert(fclose(file)==0);
	free(f->filename);
	free(f);
	return 0;
}

