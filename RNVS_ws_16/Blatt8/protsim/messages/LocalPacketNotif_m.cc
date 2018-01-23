//
// Generated file, do not edit! Created by opp_msgc 3.3 from LocalPacketNotif.msg.
//

// Disable warnings about unused variables, empty switch stmts, etc:
#ifdef _MSC_VER
#  pragma warning(disable:4101)
#  pragma warning(disable:4065)
#endif

#include <iostream>
#include <sstream>
#include "LocalPacketNotif_m.h"

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


Register_Class(LocalPacketNotif);

LocalPacketNotif::LocalPacketNotif(const char *name, int kind) : NetworkPacket(name,kind)
{
    this->noRoute_var = false;
}

LocalPacketNotif::LocalPacketNotif(const LocalPacketNotif& other) : NetworkPacket()
{
    unsigned int i;
    setName(other.name());
    operator=(other);
}

LocalPacketNotif::~LocalPacketNotif()
{
    unsigned int i;
}

LocalPacketNotif& LocalPacketNotif::operator=(const LocalPacketNotif& other)
{
    if (this==&other) return *this;
    unsigned int i;
    NetworkPacket::operator=(other);
    this->noRoute_var = other.noRoute_var;
    return *this;
}

void LocalPacketNotif::netPack(cCommBuffer *b)
{
    NetworkPacket::netPack(b);
    doPacking(b,this->noRoute_var);
}

void LocalPacketNotif::netUnpack(cCommBuffer *b)
{
    NetworkPacket::netUnpack(b);
    doUnpacking(b,this->noRoute_var);
}

bool LocalPacketNotif::getNoRoute() const
{
    return noRoute_var;
}

void LocalPacketNotif::setNoRoute(bool noRoute_var)
{
    this->noRoute_var = noRoute_var;
}

class LocalPacketNotifDescriptor : public cStructDescriptor
{
  public:
    LocalPacketNotifDescriptor();
    virtual ~LocalPacketNotifDescriptor();
    LocalPacketNotifDescriptor& operator=(const LocalPacketNotifDescriptor& other);
    virtual cPolymorphic *dup() const {return new LocalPacketNotifDescriptor(*this);}

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

Register_Class(LocalPacketNotifDescriptor);

LocalPacketNotifDescriptor::LocalPacketNotifDescriptor() : cStructDescriptor("NetworkPacket")
{
}

LocalPacketNotifDescriptor::~LocalPacketNotifDescriptor()
{
}

int LocalPacketNotifDescriptor::getFieldCount()
{
    return baseclassdesc ? 1+baseclassdesc->getFieldCount() : 1;
}

int LocalPacketNotifDescriptor::getFieldType(int field)
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

const char *LocalPacketNotifDescriptor::getFieldName(int field)
{
    if (baseclassdesc) {
        if (field < baseclassdesc->getFieldCount())
            return baseclassdesc->getFieldName(field);
        field -= baseclassdesc->getFieldCount();
    }
    switch (field) {
        case 0: return "noRoute";
        default: return NULL;
    }
}

const char *LocalPacketNotifDescriptor::getFieldTypeString(int field)
{
    if (baseclassdesc) {
        if (field < baseclassdesc->getFieldCount())
            return baseclassdesc->getFieldTypeString(field);
        field -= baseclassdesc->getFieldCount();
    }
    switch (field) {
        case 0: return "bool";
        default: return NULL;
    }
}

const char *LocalPacketNotifDescriptor::getFieldEnumName(int field)
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

int LocalPacketNotifDescriptor::getArraySize(int field)
{
    if (baseclassdesc) {
        if (field < baseclassdesc->getFieldCount())
            return baseclassdesc->getArraySize(field);
        field -= baseclassdesc->getFieldCount();
    }
    LocalPacketNotif *pp = (LocalPacketNotif *)p;
    switch (field) {
        default: return 0;
    }
}

bool LocalPacketNotifDescriptor::getFieldAsString(int field, int i, char *resultbuf, int bufsize)
{
    if (baseclassdesc) {
        if (field < baseclassdesc->getFieldCount())
            return baseclassdesc->getFieldAsString(field,i,resultbuf,bufsize);
        field -= baseclassdesc->getFieldCount();
    }
    LocalPacketNotif *pp = (LocalPacketNotif *)p;
    switch (field) {
        case 0: bool2string(pp->getNoRoute(),resultbuf,bufsize); return true;
        default: return false;
    }
}

bool LocalPacketNotifDescriptor::setFieldAsString(int field, int i, const char *value)
{
    if (baseclassdesc) {
        if (field < baseclassdesc->getFieldCount())
            return baseclassdesc->setFieldAsString(field,i,value);
        field -= baseclassdesc->getFieldCount();
    }
    LocalPacketNotif *pp = (LocalPacketNotif *)p;
    switch (field) {
        case 0: pp->setNoRoute(string2bool(value)); return true;
        default: return false;
    }
}

const char *LocalPacketNotifDescriptor::getFieldStructName(int field)
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

void *LocalPacketNotifDescriptor::getFieldStructPointer(int field, int i)
{
    if (baseclassdesc) {
        if (field < baseclassdesc->getFieldCount())
            return baseclassdesc->getFieldStructPointer(field, i);
        field -= baseclassdesc->getFieldCount();
    }
    LocalPacketNotif *pp = (LocalPacketNotif *)p;
    switch (field) {
        default: return NULL;
    }
}

sFieldWrapper *LocalPacketNotifDescriptor::getFieldWrapper(int field, int i)
{
    return NULL;
}

