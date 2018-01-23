//
// Generated file, do not edit! Created by opp_msgc 3.3 from PingMessage.msg.
//

// Disable warnings about unused variables, empty switch stmts, etc:
#ifdef _MSC_VER
#  pragma warning(disable:4101)
#  pragma warning(disable:4065)
#endif

#include <iostream>
#include <sstream>
#include "PingMessage_m.h"

// Template rule which fires if a struct or class doesn't have operator<<
template<typename T>
std::ostream& operator<<(std::ostream& out,const T&) {return out;}

// Another default rule (prevents compiler from choosing base class' doPacking())
template<typename T>
void doPacking(cCommBuffer *, T& t) {
    throw new cException("Parsim error: no doPacking() function for type %s or its base class (check .msg and _m.cc/h files!)",opp_typename(typeid(t)));
}
template<typename T>
void doUnpacking(cCommBuffer *, T& t) {
    throw new cException("Parsim error: no doUnpacking() function for type %s or its base class (check .msg and _m.cc/h files!)",opp_typename(typeid(t)));
}

// Automatically supply array (un)packing functions
template<typename T>
void doPacking(cCommBuffer *b, T *t, int n) {
    for (int i=0; i<n; i++)
        doPacking(b,t[i]);
}
template<typename T>
void doUnpacking(cCommBuffer *b, T *t, int n) {
    for (int i=0; i<n; i++)
        doUnpacking(b,t[i]);
}
inline void doPacking(cCommBuffer *, cPolymorphic&) {}
inline void doUnpacking(cCommBuffer *, cPolymorphic&) {}

#define DOPACKING(T,R) \
    inline void doPacking(cCommBuffer *b, T R a) {b->pack(a);}  \
    inline void doPacking(cCommBuffer *b, T *a, int n) {b->pack(a,n);}  \
    inline void doUnpacking(cCommBuffer *b, T& a) {b->unpack(a);}  \
    inline void doUnpacking(cCommBuffer *b, T *a, int n) {b->unpack(a,n);}
#define _
DOPACKING(char,_)
DOPACKING(unsigned char,_)
DOPACKING(bool,_)
DOPACKING(short,_)
DOPACKING(unsigned short,_)
DOPACKING(int,_)
DOPACKING(unsigned int,_)
DOPACKING(long,_)
DOPACKING(unsigned long,_)
DOPACKING(float,_)
DOPACKING(double,_)
DOPACKING(long double,_)
DOPACKING(char *,_)
DOPACKING(const char *,_)
DOPACKING(opp_string,&)
//DOPACKING(std::string,&)
#undef _
#undef DOPACKING


Register_Class(PingMessage);

PingMessage::PingMessage(const char *name, int kind) : NetworkPacket(name,kind)
{
    this->seqNo_var = -1;
    this->reply_var = false;
}

PingMessage::PingMessage(const PingMessage& other) : NetworkPacket()
{
    unsigned int i;
    setName(other.name());
    operator=(other);
}

PingMessage::~PingMessage()
{
    unsigned int i;
}

PingMessage& PingMessage::operator=(const PingMessage& other)
{
    if (this==&other) return *this;
    unsigned int i;
    NetworkPacket::operator=(other);
    this->seqNo_var = other.seqNo_var;
    this->reply_var = other.reply_var;
    return *this;
}

void PingMessage::netPack(cCommBuffer *b)
{
    NetworkPacket::netPack(b);
    doPacking(b,this->seqNo_var);
    doPacking(b,this->reply_var);
}

void PingMessage::netUnpack(cCommBuffer *b)
{
    NetworkPacket::netUnpack(b);
    doUnpacking(b,this->seqNo_var);
    doUnpacking(b,this->reply_var);
}

long PingMessage::getSeqNo() const
{
    return seqNo_var;
}

void PingMessage::setSeqNo(long seqNo_var)
{
    this->seqNo_var = seqNo_var;
}

bool PingMessage::getReply() const
{
    return reply_var;
}

void PingMessage::setReply(bool reply_var)
{
    this->reply_var = reply_var;
}

class PingMessageDescriptor : public cStructDescriptor
{
  public:
    PingMessageDescriptor();
    virtual ~PingMessageDescriptor();
    PingMessageDescriptor& operator=(const PingMessageDescriptor& other);
    virtual cPolymorphic *dup() const {return new PingMessageDescriptor(*this);}

    virtual int getFieldCount();
    virtual const char *getFieldName(int field);
    virtual int getFieldType(int field);
    virtual const char *getFieldTypeString(int field);
    virtual const char *getFieldEnumName(int field);
    virtual int getArraySize(int field);

    virtual bool getFieldAsString(int field, int i, char *resultbuf, int bufsize);
    virtual bool setFieldAsString(int field, int i, const char *value);

    virtual const char *getFieldStructName(int field);
    virtual void *getFieldStructPointer(int field, int i);
    virtual sFieldWrapper *getFieldWrapper(int field, int i);
};

Register_Class(PingMessageDescriptor);

PingMessageDescriptor::PingMessageDescriptor() : cStructDescriptor("NetworkPacket")
{
}

PingMessageDescriptor::~PingMessageDescriptor()
{
}

int PingMessageDescriptor::getFieldCount()
{
    return baseclassdesc ? 2+baseclassdesc->getFieldCount() : 2;
}

int PingMessageDescriptor::getFieldType(int field)
{
    if (baseclassdesc) {
        if (field < baseclassdesc->getFieldCount())
            return baseclassdesc->getFieldType(field);
        field -= baseclassdesc->getFieldCount();
    }
    switch (field) {
        case 0: return FT_BASIC;
        case 1: return FT_BASIC;
        default: return FT_INVALID;
    }
}

const char *PingMessageDescriptor::getFieldName(int field)
{
    if (baseclassdesc) {
        if (field < baseclassdesc->getFieldCount())
            return baseclassdesc->getFieldName(field);
        field -= baseclassdesc->getFieldCount();
    }
    switch (field) {
        case 0: return "seqNo";
        case 1: return "reply";
        default: return NULL;
    }
}

const char *PingMessageDescriptor::getFieldTypeString(int field)
{
    if (baseclassdesc) {
        if (field < baseclassdesc->getFieldCount())
            return baseclassdesc->getFieldTypeString(field);
        field -= baseclassdesc->getFieldCount();
    }
    switch (field) {
        case 0: return "long";
        case 1: return "bool";
        default: return NULL;
    }
}

const char *PingMessageDescriptor::getFieldEnumName(int field)
{
    if (baseclassdesc) {
        if (field < baseclassdesc->getFieldCount())
            return baseclassdesc->getFieldEnumName(field);
        field -= baseclassdesc->getFieldCount();
    }
    switch (field) {
        default: return NULL;
    }
}

int PingMessageDescriptor::getArraySize(int field)
{
    if (baseclassdesc) {
        if (field < baseclassdesc->getFieldCount())
            return baseclassdesc->getArraySize(field);
        field -= baseclassdesc->getFieldCount();
    }
    PingMessage *pp = (PingMessage *)p;
    switch (field) {
        default: return 0;
    }
}

bool PingMessageDescriptor::getFieldAsString(int field, int i, char *resultbuf, int bufsize)
{
    if (baseclassdesc) {
        if (field < baseclassdesc->getFieldCount())
            return baseclassdesc->getFieldAsString(field,i,resultbuf,bufsize);
        field -= baseclassdesc->getFieldCount();
    }
    PingMessage *pp = (PingMessage *)p;
    switch (field) {
        case 0: long2string(pp->getSeqNo(),resultbuf,bufsize); return true;
        case 1: bool2string(pp->getReply(),resultbuf,bufsize); return true;
        default: return false;
    }
}

bool PingMessageDescriptor::setFieldAsString(int field, int i, const char *value)
{
    if (baseclassdesc) {
        if (field < baseclassdesc->getFieldCount())
            return baseclassdesc->setFieldAsString(field,i,value);
        field -= baseclassdesc->getFieldCount();
    }
    PingMessage *pp = (PingMessage *)p;
    switch (field) {
        case 0: pp->setSeqNo(string2long(value)); return true;
        case 1: pp->setReply(string2bool(value)); return true;
        default: return false;
    }
}

const char *PingMessageDescriptor::getFieldStructName(int field)
{
    if (baseclassdesc) {
        if (field < baseclassdesc->getFieldCount())
            return baseclassdesc->getFieldStructName(field);
        field -= baseclassdesc->getFieldCount();
    }
    switch (field) {
        default: return NULL;
    }
}

void *PingMessageDescriptor::getFieldStructPointer(int field, int i)
{
    if (baseclassdesc) {
        if (field < baseclassdesc->getFieldCount())
            return baseclassdesc->getFieldStructPointer(field, i);
        field -= baseclassdesc->getFieldCount();
    }
    PingMessage *pp = (PingMessage *)p;
    switch (field) {
        default: return NULL;
    }
}

sFieldWrapper *PingMessageDescriptor::getFieldWrapper(int field, int i)
{
    return NULL;
}

