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

/*  Cut-down version from MOMBASA SE
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

#ifndef _PARSER_H
#define _PARSER_H

// TODO rewrite with string???

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <cassert>
#include <cstdio>
#include <string>
using namespace std;
// #include <stdint.h>

/// Abstract parser class.
class Parser {
public:
    /// Translation rule for an option
    struct Translation {
	char * keyword; ///< Keyword of an option.
	int    token;   ///< Related token. 0 is reserved.
	int    min_arg; ///< Minimum number of arguments for this option.
	int    max_arg; ///< Maximum number of arguments for this option. Limitted to 16 arguments.
    };


    class Error : public exception {
    public:
	enum Code { eNO_ERROR = 0, eIO_ERROR = -1, eSYNTAX_ERROR = -2,
		    eARGUMENT_COUNT_ERROR = -3, eUNKNOWN_KEYWORD_ERROR = -4,
		    eINDEX_ERROR = -5, eCONVERSION_ERROR = -6, eMISC_ERROR = -7 };

    protected:
	string msg;
	string location;

	Code         code;
	unsigned int lineno;

    public:
	explicit Error(const string& loc_, Code code_,
		       unsigned int lineno_ = 0)
	    : msg(getCodeStr(code_)), location(loc_),
	      code(code_), lineno(lineno_) { }

	explicit Error(const string& loc_, Code code_, const string& msg_,
		       unsigned int lineno_ = 0)
	    : msg(msg_), location(loc_), code(code_), lineno(lineno_) { }


	virtual const char * what() const throw() {
	    return msg.c_str();
	}

	const char * where() const throw() {
	    return location.c_str();
	}

	void setWhat(const string& _msg) { msg = _msg; };

	Code getCode() const throw() { return code; }
	string getCodeStr() const {
	    return getCodeStr(code);
	}

	unsigned int getLineNo() const throw() { return lineno; }

	static string getCodeStr(Code _code);
	virtual ~Error() throw() {};
    };

    template <Error::Code errorType>
    class _Error : public Error {
    public:
	explicit _Error(const string& _loc, unsigned int _lineno = 0)
	    : Error(_loc, errorType, _lineno) { }
	explicit _Error(const string& _loc, const string& _msg,
			unsigned int _lineno = 0)
	    : Error(_loc, errorType, _msg, _lineno) { }
    };

    typedef _Error<Error::eIO_ERROR>              IO_Error;
    typedef _Error<Error::eSYNTAX_ERROR>          SyntaxError;
    typedef _Error<Error::eARGUMENT_COUNT_ERROR>  ArgumentCountError;
    typedef _Error<Error::eUNKNOWN_KEYWORD_ERROR> UnknownKeywordError;
    typedef _Error<Error::eINDEX_ERROR>           IndexError;
    typedef _Error<Error::eCONVERSION_ERROR>      ConversionError;
    typedef _Error<Error::eMISC_ERROR>            MiscError;

protected:
    const Translation * transtable; ///< Translation table.

    char *         line; ///< Text line read from configuration file.
    size_t         line_size; ///< Number of characters in Parser::line.
    unsigned int   lineno; ///< Current line number.

    int            argcount; ///< Number of arguments in this line.
    int            token; ///< Token of this line.
    char *         keyword; ///< Pointer to the keyword in the line buffer.
    char *         args[16]; ///< Pointers to the arguments in the line buffer.

    /** Create a parser.
     *  \param table Translation table.
     *  \throws MiscError
     */
    Parser(const Translation table[]);

public:
    /** Parse next (non-comment) text line.
     *  \return Line number.
     *  \throws Error
     */
    unsigned int nextLine();

    /** Read a text line. Must be overridden.
     *  \return Number of characters read or error code.
     */
    virtual unsigned int readLine() = 0;

    int getLineNo() const { return lineno; }     ///< Get current line number.
    int getArgCount() const { return argcount; } ///< Get current argument count.
    int getToken() const { return token; }       ///< Get current token.

    /** Convert argument to a string.
     *  \param index Argument number.
     *  \param x Address of value.
     *  \throws Error
     */
    void getString(int index, string * x) const;

    /** Convert argument to a C string.
     *  \param index Argument number.
     *  \param x Address of value.
     *  \param n Size of string buffer.
     *  \throws Error
     */
    void getCString(int index, char * x, size_t n) const;

    /** Convert argument to an int.
     *  \param index Argument number.
     *  \param x Address of value.
     *  \throws Error
     */
    void getInt(int index, int * x) const;

    /** Convert argument to an unsigned int.
     *  \param index Argument number.
     *  \param x Address of value.
     *  \throws Error
     */
    void getUInt(int index, unsigned int * x) const;

    /** Convert argument to a unsigned short.
     *  \param index Argument number.
     *  \param x Address of value.
     *  \throws Error
     */
    void getUShort(int index, unsigned short * x) const;

    /** Convert argument to a double.
     *  \param index Argument number.
     *  \param x Address of value.
     *  \throws Error
     */
    void getDouble(int index, double * x) const;

    
    /** Convert argument to a bool.
     *  \param index Argument number.
     *  \param x Address of value.
     *  \throws Error
     */
    void getBool(int index, bool * x) const;

//     /** Convert argument to an IP address in network byte order.
//      *  \param index Argument number.
//      *  \param x Address of value.
//      *  \throws Error
//      */
//     void getIP(int index, uint32_t * x) const;

//     /** Convert argument to an IP address with mask.
//      *  \param index Argument number.
//      *  \param addr Address of address value.
//      *  \param mask Address of mask value.
//      *  \param masklen Address of mask length.
//      *  \throws Error
//      */
//     void getIPMask(int index, uint32_t * addr, uint32_t * mask, uint8_t * masklen = NULL) const; // network byte order

//     /** Convert argument to a pair of IP addresses.
//      *  \param index Argument number.
//      *  \param addr1 Address of first address.
//      *  \param addr2 Address of second address.
//      *  \throws Error
//      */
//     void getIPPair(int index, uint32_t * addr1, uint32_t * addr2) const; // network byte order

    /** Finish parsing of configuration file.
     *  \throws Error
     */
    virtual void close();

    /// Destroy parser.
    virtual ~Parser() throw() {
	try {
	    close();
	}
	catch (...) { }
    }

private:
    void saveSeparatorsInStrings();
    void restoreSeparatorsInStrings(size_t len);

private:
    Parser(const Parser&);
    void operator = (const Parser&);
};


// /// Configuration file parser.
// class FileParser : public Parser {
// private:
//     FILE *         fp; ///< Configuration file pointer.
//     bool           close_on_delete; ///< The file should be closed by us.

// public:
//     /** Create a file parser by file name.
//      *  \param conffile Configuration file name.
//      *  \param table Translation table.
//      *  \throws MiscError
//      *  \throws Error
//      */
//     FileParser(const char * conffile, const Translation table[]);

//     /** Create a parser by file descriptor.
//      *  \param fdesc Open file descriptor.
//      *  \param table Translation table.
//      *  \throws MiscError
//      *  \throws Error
//      */
//     FileParser(int fdesc, const Translation table[]);

//     /** Create a parser by FILE pointer.
//      *  \param file Open FILE pointer.
//      *  \param table Translation table.
//      *  \param _close Should the file be closed on destruction?
//      *  \throws MiscError
//      */
//     FileParser(FILE * file, const Translation table[],
// 		   bool _close = true)
// 	: Parser(table), fp(file), close_on_delete(_close) {
// 	if (file == NULL) throw MiscError("FileParser","file is NULL");
//     }

//     /** Read a text line.
//      *  \return Number of characters read or error code.
//      *  \throws Error
//      */
//     virtual unsigned int readLine();

//     /** Finish parsing of configuration file.
//      *  \throws Error
//      */
//     virtual void close();

//     /// Destroy file parser.
//     virtual ~FileParser() throw() {
// 	try {
// 	    close();
// 	}
// 	catch (...) { }
//     }
// };


/**
 * Parser for strings. It may be given either a whole line or just an
 * argument. It gives the string either exactly once or infinite times.
 */
class StringParser : public Parser {
private:
    string  data;     ///< String from which line is filled.
    bool    infinite; /**< The same line should be returned on every call of
			*  readLine()
			*/
    bool    finished; ///< The line was already returned once.

    /// Translation table with a dummy keyword only.
    static const Translation dummy_table[];

public:
    /** Create string parser from a string denoting a whole line.
     *  \param _line String containing the text line.
     *  \param table Translation table.
     *  \param _infinite Should the line be returned again and again?
     *  \throws MiscError
     */
    StringParser(const string& _line, const Translation table[],
		     bool _infinite = false)
	: Parser(table), data(_line), infinite(_infinite), finished(false)
	{ }

    /** Create string parser from a string denoting only a single argument.
     *  \param _arg String containing the text argument.
     *  \param _infinite Should the line be returned again and again?
     *  \throws MiscError
     */
    StringParser(const string& _arg, bool _infinite = false)
	: Parser(dummy_table), infinite(_infinite), finished(false) {

	if (_arg.empty()) data = "dummy: @empty@";
	else data = string("dummy: ")+_arg;
    }

    /** Read a text line.
     *  \return Number of characters read or error code.
     *  \throws Error
     *  \throws bad_alloc
     */
    virtual unsigned int readLine();

    /** Finish parsing of string.
     *  \throws Error
     */
    virtual void close();

    /// Destroy string parser.
    virtual ~StringParser() throw () {
	try {
	    close();
	}
	catch (...) { }
    }
};

// /// Inline function to convert mask length to mask
// inline uint32_t len2mask(uint8_t len) {
//     assert(len <= 32);
//     return len ? (0xffffffff << (32-len)) : 0;
// }

#endif /*_PARSER_H */
