/***************************************************************************
 *
 * This file is part of the ProtSim framework developed by TKN for a
 * practical course on basics of simulation and Internet protocol functions
 *
 * Copyright:   (C)2004-2007 Telecommunication Networks Group (TKN) at
 *              Technische Universitaet Berlin, Germany.
 *
 * Authors:     Lars Westerhoff, Guenter Schaefer
 *
 **************************************************************************/

/*  MOMBASA Software Environment
 *
 *  Copyright (C) 2001  Lars Westerhoff <westerhoff@ee.tu-berlin.de>
 *                      Technical University Berlin
 *                      Faculty IV Electrical Engineering and Computer Science
 *                      Institute for Telecommunication Systems
 *                      Telecommunication Networks Group
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 *  Log:
 *  Lars Westerhoff         : First public release
 */

/** \file
 *  \ingroup Support
 *  Implementation of Parser.
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
// #include <cstdlib>
#include <omnetpp.h>

#include <cstring>
#include <cerrno>

// #ifndef WIN32
namespace std {
#include <cstdlib>
};
using namespace std;
// #include <sys/socket.h>
// #include <netinet/in.h>
// #include <arpa/inet.h>

#include "Parser.h"
#include "../common/protsim_defines.h"

//  class Parser {
//  private:
//      FILE *         fp;
//      Translation *  transtable;

//      char *         line;
//      size_t         line_size;
//      unsigned int   lineno;

//      int            argcount;
//      int            token;
//      char *         keyword;
//      char *         args[16];

//  public:
//      struct Translation {
//  	char * keyword;
//  	int    token;
//  	int    min_arg;
//  	int    max_arg; // limitted to 16 arguments
//      };

//      enum RetCode { eNO_ERROR = 0, eIO_ERROR = -1, eSYNTAX_ERROR = -2,
//		       eARGUMENT_COUNT_ERROR = -3, eUNKNOWN_KEYWORD_ERROR = -4,
// 		       eINDEX_ERROR = -5, eCONVERSION_ERROR = -6 };

string Parser::Error::getCodeStr(Code _code) {
    switch (_code) {
    case eNO_ERROR:
	return string("Okay");
    case eIO_ERROR:
	return string("IO error");
    case eSYNTAX_ERROR:
	return string("Syntax error");
    case eARGUMENT_COUNT_ERROR:
	return string("Invalid number of arguments");
    case eUNKNOWN_KEYWORD_ERROR:
	return string("Unknown keyword");
    case eINDEX_ERROR:
	return string("Invalid index");
    case eCONVERSION_ERROR:
	return string("Conversion error");
    case eMISC_ERROR:
	return string("Miscellaneous error");
    default:
	return string("Unknown error");
    }
}


Parser::Parser(const Translation table[]) {
    line = NULL;
    line_size = 0;
    lineno = 0;
    argcount = 0;
    token = 0;
    keyword = NULL;
    transtable = NULL;

    for (int i = 0; table[i].keyword != NULL; i++) {
	if ((table[i].min_arg < 0) || (table[i].max_arg > 16))
	    throw MiscError("Parser","Argument count must be between 0 and 16");
    }
    transtable = table;
}

unsigned int Parser::nextLine() {
    ssize_t len;

    argcount = 0;
    token    = 0;
    keyword  = NULL;


    while (keyword == NULL) {
	errno = 0;
	len = readLine();
	if (len == 0) return 0;

	++lineno;

	// eliminate end of line
	if (line[len-1] == '\n') {
	  line[len-1] = '\0';
	}
	// eliminate comment

	char * p;
	if ((p = strchr(line, '#')) != NULL) *p = '\0';
	// We need ASCII 1-3 for special purposes
	if (strpbrk(line,"\001\002\003") != NULL)
	    throw SyntaxError("Parser::nextLine",
			      "ASCII codes 1-3 are reserved", lineno);
	saveSeparatorsInStrings();

	// read keyword; empty lines and comments are ignored
	keyword = strtok(line," \t:=");
    }

    // read arguments
    for (argcount = 0; argcount < 16; argcount++) {
	if ((args[argcount] = strtok(NULL, " \t,")) == NULL) break;

	char del;
	if ((del = args[argcount][0]) == '"' || del == '\'') {
	    assert(args[argcount][strlen(args[argcount])-1] == del);
	    ++args[argcount];
	    args[argcount][strlen(args[argcount])-1] = '\0';
	}
    }

    // more than 16 arguments is an error
    if (argcount == 16 && strtok(NULL, " \t,") != NULL) {
	argcount = 0;
	throw ArgumentCountError("Parser::nextLine","Too many arguments",
				 lineno);
    }

    // determine token
    for (int i = 0; transtable[i].keyword != NULL; i++) {
	if (strcasecmp(keyword, transtable[i].keyword) == 0) {
	    token = transtable[i].token;
	    if (argcount < transtable[i].min_arg || argcount > transtable[i].max_arg)
		throw ArgumentCountError("Parser::nextLine", lineno);
	    break;
	}
    }

    //DEBUGOUT(DEBUG_GENERAL1,"Keyword: %s\n",keyword);

    if (token == 0) throw UnknownKeywordError("Parser::nextLine", lineno);

    restoreSeparatorsInStrings(len);
    return lineno;
}


void Parser::saveSeparatorsInStrings() {
    for (char * start = strpbrk(line,"'\""); start != NULL;
	 start = strpbrk(start,"'\"")) {
	char del = *start;
	char * end = start+1;
	while ((end = strchr(end,del)) != NULL && *(end-1) == '\\') ++end;
	if (end == NULL)
	    throw SyntaxError("Parser::nextLine",
			      "String not terminated", lineno);
	for (char * c = start; c != end; ++c) {
	    switch (*c) {
	    case ' ': *c = '\001'; break;
	    case '\t': *c = '\002'; break;
	    case ',': *c = '\003'; break;
	    }
	}
	start = end + 1;
    }
}

void Parser::restoreSeparatorsInStrings(size_t len) {
    for (char * c = line; c < line+len; ++c) {
	    switch (*c) {
	    case '\001': *c = ' '; break;
	    case '\002': *c = '\t'; break;
	    case '\003': *c = ','; break;
	    }
    }
}

//      int getLineNo() { return lineno; }
//      int getArgCount() { return argcount; }
//      int getToken()  { return token; }


void Parser::getString(int index, string * x) const {
    if (index < 0 || index >= argcount)
	throw IndexError("Parser::getString", lineno);

    assert(x != NULL);
    *x = args[index];
    if (*x == "@empty@") *x = "";
}


void Parser::getCString(int index, char * x, size_t n) const {
    if (index < 0 || index >= argcount)
	throw IndexError("Parser::getCString", lineno);

    assert(x != NULL);
	x[0] = '\0';
    if (strlen(args[index]) >= n)
	throw ConversionError("Parser::getCString","String too long",
			      lineno);
    strcpy(x, args[index]);
    if (strcmp(x,"@empty@") == 0) x[0] = '\0';
}

void Parser::getInt(int index, int * x) const {
    if (index < 0 || index >= argcount)
	throw IndexError("Parser::getInt", lineno);

    assert(x != NULL);
    *x = 0;
    if (strncmp(args[index], "0x", 2) == 0) {
	if (sscanf(args[index], "%x", x) != 1)
	    throw ConversionError("Parser::getInt", "Integer expected",
				  lineno);
    } else if (strncmp(args[index], "0", 1) == 0) {
	if (sscanf(args[index], "%o", x) != 1)
	    throw ConversionError("Parser::getInt", "Integer expected",
				  lineno);
    } else {
	if (sscanf(args[index], "%d", x) != 1)
	    throw ConversionError("Parser::getInt", "Integer expected",
				  lineno);
    }
}


void Parser::getUInt(int index, unsigned int * x) const {
    if (index < 0 || index >= argcount)
	throw IndexError("Parser::getUInt", lineno);

    assert(x != NULL);
    *x = 0;
    if (strncmp(args[index], "0x", 2) == 0) {
	if (sscanf(args[index], "%x", x) != 1)
	    throw ConversionError("Parser::getUInt", "Unsigned integer expected", lineno);
    } else if (strncmp(args[index], "0", 1) == 0) {
	if (sscanf(args[index], "%o", x) != 1)
	    throw ConversionError("Parser::getUInt", "Unsigned integer expected", lineno);
    } else {
	if (sscanf(args[index], "%u", x) != 1)
	    throw ConversionError("Parser::getUInt", "Unsigned integer expected", lineno);
    }
}


void Parser::getUShort(int index, unsigned short * x) const {
    if (index < 0 || index >= argcount)
	throw IndexError("Parser::getUShort", lineno);

    assert(x != NULL);
    *x = 0;
    if (strncmp(args[index], "0x", 2) == 0) {
	if (sscanf(args[index], "%hx", x) != 1)
	    throw ConversionError("Parser::getUShort", "Integer between 0 and 65535 expected", lineno);
    } else if (strncmp(args[index], "0", 1) == 0) {
	if (sscanf(args[index], "%ho", x) != 1)
	    throw ConversionError("Parser::getUShort", "Integer between 0 and 65535 expected", lineno);
    } else {
	if (sscanf(args[index], "%hu", x) != 1)
	    throw ConversionError("Parser::getUShort", "Integer between 0 and 65535 expected", lineno);
    }
}


void Parser::getDouble(int index, double * x) const {
    if (index < 0 || index >= argcount)
	throw IndexError("Parser::getDouble", lineno);

    assert(x != NULL);
    *x = 0.0;

    if (sscanf(args[index], "%lf", x) != 1)
	throw ConversionError("Parser::getDouble", "Double floating point value expected", lineno);
}


void Parser::getBool(int index, bool * x) const {
    if (index < 0 || index >= argcount)
	throw IndexError("Parser::getBool", lineno);

    assert( x != NULL);
    *x = false;
    if ((strcasecmp(args[index],"true") == 0) ||
	(strcasecmp(args[index],"yes") == 0) ||
	(strcasecmp(args[index],"1") == 0)) {
	*x = true;
	return;
    }
    if ((strcasecmp(args[index],"false") == 0) ||
	(strcasecmp(args[index],"no") == 0) ||
	(strcasecmp(args[index],"0") == 0)) {
	*x = false;
	return;
    }
    throw ConversionError("Parser::getBool", "Boolean (1/0, Yes/No, True/False) expected", lineno);
}

// void Parser::getIP(int index, uint32_t * x) const {
//     if (index < 0 || index >= argcount)
// 	throw IndexError("Parser::getIP", lineno);

//     in_addr addr;

//     assert(x != NULL);
//     if (inet_aton(args[index],&addr) == 0)
// 	throw ConversionError("Parser::getIP", "IP address expected",
// 			      lineno);
//     *x = addr.s_addr;
// }


// void Parser::getIPMask(int index, uint32_t * addr, uint32_t * mask, uint8_t * masklen) const {
//     if (index < 0 || index >= argcount)
// 	throw IndexError("Parser::getIPMask", lineno);

//     in_addr tmp_addr;
//     int     bits;

//     assert(addr != NULL);
//     assert(mask != NULL);

//     char * s = strchr(args[index],'/');
//     if (s == NULL)
// 	throw ConversionError("Parser::getIPMask", "IP address and mask expected", lineno);
//     *s = '\0';

//     if (inet_aton(args[index],&tmp_addr) == 0) {
// 	*s = '/';
// 	throw ConversionError("Parser::getIPMask", "IP address and mask expected", lineno);
//     }
//     *s = '/';
//     s++;
//     if (sscanf(s, "%d", &bits) != 1)
// 	throw ConversionError("Parser::getIPMask", "IP address and mask expected", lineno);
//     if (bits < 0 || bits > 32)
// 	throw ConversionError("Parser::getIPMask", "IP address and mask expected", lineno);

//     *addr = tmp_addr.s_addr;
//     *mask = htonl(len2mask(bits));

//     if (masklen != NULL) *masklen = bits;
// }

// void Parser::getIPPair(int index, uint32_t * addr1, uint32_t * addr2) const {
//     if (index < 0 || index >= argcount)
// 	throw IndexError("Parser::getIPPair", lineno);

//     in_addr tmp_addr1, tmp_addr2;

//     assert(addr1 != NULL);
//     assert(addr2 != NULL);

//     char * s = strchr(args[index],'/');
//     if (s == NULL)
// 	throw ConversionError("Parser::getIPPair", "IP address pair expected (seperated by '/')", lineno);
//     *s = '\0';

//     if (inet_aton(args[index],&tmp_addr1) == 0) {
// 	*s = '/';
// 	throw ConversionError("Parser::getIPPair", "IP address pair expected (seperated by '/')", lineno);
//     }
//     *s = '/';
//     s++;
//     if (inet_aton(s,&tmp_addr2) == 0) {
// 	throw ConversionError("Parser::getIPPair", "IP address pair expected (seperated by '/')", lineno);
//     }

//     *addr1 = tmp_addr1.s_addr;
//     *addr2 = tmp_addr2.s_addr;
// }

void Parser::close() {
    transtable = NULL;
    argcount = 0;
    token = 0;
}

//      ~Parser() {
//  	close();
//      }

//  private:
//      Parser(const Parser&) __forbidden__
//      void operator = (const Parser&) __forbidden__
//  };

// FileParser::FileParser(const char * conffile, const Translation table[]) : Parser(table) {

//     if ((fp = fopen(conffile,"r")) == NULL)
// 	throw IO_Error("FileParser","Could not open file");
//     close_on_delete = true;
// }

// FileParser::FileParser(int fdesc, const Translation table[])
//     : Parser(table) {

//     if ((fp = fdopen(fdesc,"r")) == NULL)
// 	throw IO_Error("FileParser","Could not open file");
//     close_on_delete = true;
// }

// unsigned int FileParser::readLine() {
//     errno = 0;
//     int retval = getline(&line,&line_size,fp);
//     if (retval < 0) {
// 	if (errno == 0) return 0;
// 	throw IO_Error("FileParser::readLine", lineno);
//     }
//     return retval;
// }

// void FileParser::close() {
//     if (line != NULL) {
// 	free(line);
// 	line = NULL;
// 	line_size = 0;
//     }
//     try {
// 	if (fp != NULL && close_on_delete) {
// 	    if (fclose(fp) == EOF)
// 		throw IO_Error("FileParser::close","Error while closing file");
// 	    fp = NULL;
// 	}
//     }
//     catch (...) {
// 	fp = NULL;
// 	throw;
//     }
//     Parser::close();
// }


const Parser::Translation StringParser::dummy_table[] = {
    { "dummy", 1, 1, 1 },
    { NULL, 0, 0, 0 }
};

unsigned int StringParser::readLine() {
    if (finished) return 0;

    try {
	if (line_size < data.size() + 1) {
	    delete [] line;
	    line_size = data.size() + 1;
	    line = new char[line_size];
	}
    }
    catch (const bad_alloc&) {
	line_size = 0;
	throw;
    }
    int copied = data.copy(line,line_size);
    assert(copied < (int) line_size);
    line[copied] = '\0';

    if (!infinite) finished = true;

    return copied;
}

void StringParser::close() {
    if (line != NULL) {
	delete [] line;
	line = NULL;
	line_size = 0;
    }
    Parser::close();
}
