//
// Generated file, do not edit! Created by opp_msgc 3.3 from ARQMessage.msg.
//

// Disable warnings about unused variables, empty switch stmts, etc:
#ifdef _MSC_VER
#  pragma warning(disable:4101)
#  pragma warning(disable:4065)
#endif

#include <iostream>
#include <sstream>
#include "ARQMessage_m.h"

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


Register_Class(ARQData);

ARQData::ARQData(const char *name, int kind) : NetworkPacket(name,kind)
{
    this->seqNo_var = -1;
}

ARQData::ARQData(const ARQData& other) : NetworkPacket()
{
    unsigned int i;
    setName(other.name());
    operator=(other);
}

ARQData::~ARQData()
{
    unsigned int i;
}

ARQData& ARQData::operator=(const ARQData& other)
{
    if (this==&other) return *this;
    unsigned int i;
    NetworkPacket::operator=(other);
    this->seqNo_var = other.seqNo_var;
    return *this;
}

void ARQData::netPack(cCommBuffer *b)
{
    NetworkPacket::netPack(b);
    doPacking(b,this->seqNo_var);
}

void ARQData::netUnpack(cCommBuffer *b)
{
    NetworkPacket::netUnpack(b);
    doUnpacking(b,this->seqNo_var);
}

long ARQData::getSeqNo() const
{
    return seqNo_var;
}

void ARQData::setSeqNo(long seqNo_var)
{
    this->seqNo_var = seqNo_var;
}

class ARQDataDescriptor : public cStructDescriptor
{
  public:
    ARQDataDescriptor();
    virtual ~ARQDataDescriptor();
    ARQDataDescriptor& operator=(const ARQDataDescriptor& other);
    virtual cPolymorphic *dup() const {return new ARQDataDescriptor(*this);}

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

Register_Class(ARQDataDescriptor);

ARQDataDescriptor::ARQDataDescriptor() : cStructDescriptor("NetworkPacket")
{
}

ARQDataDescriptor::~ARQDataDescriptor()
{
}

int ARQDataDescriptor::getFieldCount()
{
    return baseclassdesc ? 1+baseclassdesc->getFieldCount() : 1;
}

int ARQDataDescriptor::getFieldType(int field)
{
    if (baseclassdesc) {
        if (field < baseclassdesc->getFieldCount())
            return baseclassdesc->getFieldType(field);
        field -= baseclassdesc->getFieldCount();
    }
    switch (field) {
        case 0: return FT_BASIC;
        default: return FT_INVALID;
    }
}

const char *ARQDataDescriptor::getFieldName(int field)
{
    if (baseclassdesc) {
        if (field < baseclassdesc->getFieldCount())
            return baseclassdesc->getFieldName(field);
        field -= baseclassdesc->getFieldCount();
    }
    switch (field) {
        case 0: return "seqNo";
        default: return NULL;
    }
}

const char *ARQDataDescriptor::getFieldTypeString(int field)
{
    if (baseclassdesc) {
        if (field < baseclassdesc->getFieldCount())
            return baseclassdesc->getFieldTypeString(field);
        field -= baseclassdesc->getFieldCount();
    }
    switch (field) {
        case 0: return "long";
        default: return NULL;
    }
}

const char *ARQDataDescriptor::getFieldEnumName(int field)
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

int ARQDataDescriptor::getArraySize(int field)
{
    if (baseclassdesc) {
        if (field < baseclassdesc->getFieldCount())
            return baseclassdesc->getArraySize(field);
        field -= baseclassdesc->getFieldCount();
    }
    ARQData *pp = (ARQData *)p;
    switch (field) {
        default: return 0;
    }
}

bool ARQDataDescriptor::getFieldAsString(int field, int i, char *resultbuf, int bufsize)
{
    if (baseclassdesc) {
        if (field < baseclassdesc->getFieldCount())
            return baseclassdesc->getFieldAsString(field,i,resultbuf,bufsize);
        field -= baseclassdesc->getFieldCount();
    }
    ARQData *pp = (ARQData *)p;
    switch (field) {
        case 0: long2string(pp->getSeqNo(),resultbuf,bufsize); return true;
        default: return false;
    }
}

bool ARQDataDescriptor::setFieldAsString(int field, int i, const char *value)
{
    if (baseclassdesc) {
        if (field < baseclassdesc->getFieldCount())
            return baseclassdesc->setFieldAsString(field,i,value);
        field -= baseclassdesc->getFieldCount();
    }
    ARQData *pp = (ARQData *)p;
    switch (field) {
        case 0: pp->setSeqNo(string2long(value)); return true;
        default: return false;
    }
}

const char *ARQDataDescriptor::getFieldStructName(int field)
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

void *ARQDataDescriptor::getFieldStructPointer(int field, int i)
{
    if (baseclassdesc) {
        if (field < baseclassdesc->getFieldCount())
            return baseclassdesc->getFieldStructPointer(field, i);
        field -= baseclassdesc->getFieldCount();
    }
    ARQData *pp = (ARQData *)p;
    switch (field) {
        default: return NULL;
    }
}

sFieldWrapper *ARQDataDescriptor::getFieldWrapper(int field, int i)
{
    return NULL;
}

Register_Class(ARQAck);

ARQAck::ARQAck(const char *name, int kind) : NetworkPacket(name,kind)
{
    this->seqNoExpected_var = -1;
}

ARQAck::ARQAck(const ARQAck& other) : NetworkPacket()
{
    unsigned int i;
    setName(other.name());
    operator=(other);
}

ARQAck::~ARQAck()
{
    unsigned int i;
}

ARQAck& ARQAck::operator=(const ARQAck& other)
{
    if (this==&other) return *this;
    unsigned int i;
    NetworkPacket::operator=(other);
    this->seqNoExpected_var = other.seqNoExpected_var;
    return *this;
}

void ARQAck::netPack(cCommBuffer *b)
{
    NetworkPacket::netPack(b);
    doPacking(b,this->seqNoExpected_var);
}

void ARQAck::netUnpack(cCommBuffer *b)
{
    NetworkPacket::netUnpack(b);
    doUnpacking(b,this->seqNoExpected_var);
}

long ARQAck::getSeqNoExpected() const
{
    return seqNoExpected_var;
}

void ARQAck::setSeqNoExpected(long seqNoExpected_var)
{
    this->seqNoExpected_var = seqNoExpected_var;
}

class ARQAckDescriptor : public cStructDescriptor
{
  public:
    ARQAckDescriptor();
    virtual ~ARQAckDescriptor();
    ARQAckDescriptor& operator=(const ARQAckDescriptor& other);
    virtual cPolymorphic *dup() const {return new ARQAckDescriptor(*this);}

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

Register_Class(ARQAckDescriptor);

ARQAckDescriptor::ARQAckDescriptor() : cStructDescriptor("NetworkPacket")
{
}

ARQAckDescriptor::~ARQAckDescriptor()
{
}

int ARQAckDescriptor::getFieldCount()
{
    return baseclassdesc ? 1+baseclassdesc->getFieldCount() : 1;
}

int ARQAckDescriptor::getFieldType(int field)
{
    if (baseclassdesc) {
        if (field < baseclassdesc->getFieldCount())
            return baseclassdesc->getFieldType(field);
        field -= baseclassdesc->getFieldCount();
    }
    switch (field) {
        case 0: return FT_BASIC;
        default: return FT_INVALID;
    }
}

const char *ARQAckDescriptor::getFieldName(int field)
{
    if (baseclassdesc) {
        if (field < baseclassdesc->getFieldCount())
            return baseclassdesc->getFieldName(field);
        field -= baseclassdesc->getFieldCount();
    }
    switch (field) {
        case 0: return "seqNoExpected";
        default: return NULL;
    }
}

const char *ARQAckDescriptor::getFieldTypeString(int field)
{
    if (baseclassdesc) {
        if (field < baseclassdesc->getFieldCount())
            return baseclassdesc->getFieldTypeString(field);
        field -= baseclassdesc->getFieldCount();
    }
    switch (field) {
        case 0: return "long";
        default: return NULL;
    }
}

const char *ARQAckDescriptor::getFieldEnumName(int field)
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

int ARQAckDescriptor::getArraySize(int field)
{
    if (baseclassdesc) {
        if (field < baseclassdesc->getFieldCount())
            return baseclassdesc->getArraySize(field);
        field -= baseclassdesc->getFieldCount();
    }
    ARQAck *pp = (ARQAck *)p;
    switch (field) {
        default: return 0;
    }
}

bool ARQAckDescriptor::getFieldAsString(int field, int i, char *resultbuf, int bufsize)
{
    if (baseclassdesc) {
        if (field < baseclassdesc->getFieldCount())
            return baseclassdesc->getFieldAsString(field,i,resultbuf,bufsize);
        field -= baseclassdesc->getFieldCount();
    }
    ARQAck *pp = (ARQAck *)p;
    switch (field) {
        case 0: long2string(pp->getSeqNoExpected(),resultbuf,bufsize); return true;
        default: return false;
    }
}

bool ARQAckDescriptor::setFieldAsString(int field, int i, const char *value)
{
    if (baseclassdesc) {
        if (field < baseclassdesc->getFieldCount())
            return baseclassdesc->setFieldAsString(field,i,value);
        field -= baseclassdesc->getFieldCount();
    }
    ARQAck *pp = (ARQAck *)p;
    switch (field) {
        case 0: pp->setSeqNoExpected(string2long(value)); return true;
        default: return false;
    }
}

const char *ARQAckDescriptor::getFieldStructName(int field)
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

void *ARQAckDescriptor::getFieldStructPointer(int field, int i)
{
    if (baseclassdesc) {
        if (field < baseclassdesc->getFieldCount())
            return baseclassdesc->getFieldStructPointer(field, i);
        field -= baseclassdesc->getFieldCount();
    }
    ARQAck *pp = (ARQAck *)p;
    switch (field) {
        default: return NULL;
    }
}

sFieldWrapper *ARQAckDescriptor::getFieldWrapper(int field, int i)
{
    return NULL;
}

