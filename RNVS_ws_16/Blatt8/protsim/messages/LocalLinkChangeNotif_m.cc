//
// Generated file, do not edit! Created by opp_msgc 3.3 from LocalLinkChangeNotif.msg.
//

// Disable warnings about unused variables, empty switch stmts, etc:
#ifdef _MSC_VER
#  pragma warning(disable:4101)
#  pragma warning(disable:4065)
#endif

#include <iostream>
#include <sstream>
#include "LocalLinkChangeNotif_m.h"

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


Register_Class(LocalLinkChangeNotif);

LocalLinkChangeNotif::LocalLinkChangeNotif(const char *name, int kind) : NetworkPacket(name,kind)
{
    this->linkIndex_var = 0;
    this->nextHop_var = 0;
    this->newCost_var = 0;
    this->oldCost_var = 0;
}

LocalLinkChangeNotif::LocalLinkChangeNotif(const LocalLinkChangeNotif& other) : NetworkPacket()
{
    unsigned int i;
    setName(other.name());
    operator=(other);
}

LocalLinkChangeNotif::~LocalLinkChangeNotif()
{
    unsigned int i;
}

LocalLinkChangeNotif& LocalLinkChangeNotif::operator=(const LocalLinkChangeNotif& other)
{
    if (this==&other) return *this;
    unsigned int i;
    NetworkPacket::operator=(other);
    this->linkIndex_var = other.linkIndex_var;
    this->nextHop_var = other.nextHop_var;
    this->newCost_var = other.newCost_var;
    this->oldCost_var = other.oldCost_var;
    return *this;
}

void LocalLinkChangeNotif::netPack(cCommBuffer *b)
{
    NetworkPacket::netPack(b);
    doPacking(b,this->linkIndex_var);
    doPacking(b,this->nextHop_var);
    doPacking(b,this->newCost_var);
    doPacking(b,this->oldCost_var);
}

void LocalLinkChangeNotif::netUnpack(cCommBuffer *b)
{
    NetworkPacket::netUnpack(b);
    doUnpacking(b,this->linkIndex_var);
    doUnpacking(b,this->nextHop_var);
    doUnpacking(b,this->newCost_var);
    doUnpacking(b,this->oldCost_var);
}

unsigned int LocalLinkChangeNotif::getLinkIndex() const
{
    return linkIndex_var;
}

void LocalLinkChangeNotif::setLinkIndex(unsigned int linkIndex_var)
{
    this->linkIndex_var = linkIndex_var;
}

long LocalLinkChangeNotif::getNextHop() const
{
    return nextHop_var;
}

void LocalLinkChangeNotif::setNextHop(long nextHop_var)
{
    this->nextHop_var = nextHop_var;
}

double LocalLinkChangeNotif::getNewCost() const
{
    return newCost_var;
}

void LocalLinkChangeNotif::setNewCost(double newCost_var)
{
    this->newCost_var = newCost_var;
}

double LocalLinkChangeNotif::getOldCost() const
{
    return oldCost_var;
}

void LocalLinkChangeNotif::setOldCost(double oldCost_var)
{
    this->oldCost_var = oldCost_var;
}

class LocalLinkChangeNotifDescriptor : public cStructDescriptor
{
  public:
    LocalLinkChangeNotifDescriptor();
    virtual ~LocalLinkChangeNotifDescriptor();
    LocalLinkChangeNotifDescriptor& operator=(const LocalLinkChangeNotifDescriptor& other);
    virtual cPolymorphic *dup() const {return new LocalLinkChangeNotifDescriptor(*this);}

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

Register_Class(LocalLinkChangeNotifDescriptor);

LocalLinkChangeNotifDescriptor::LocalLinkChangeNotifDescriptor() : cStructDescriptor("NetworkPacket")
{
}

LocalLinkChangeNotifDescriptor::~LocalLinkChangeNotifDescriptor()
{
}

int LocalLinkChangeNotifDescriptor::getFieldCount()
{
    return baseclassdesc ? 4+baseclassdesc->getFieldCount() : 4;
}

int LocalLinkChangeNotifDescriptor::getFieldType(int field)
{
    if (baseclassdesc) {
        if (field < baseclassdesc->getFieldCount())
            return baseclassdesc->getFieldType(field);
        field -= baseclassdesc->getFieldCount();
    }
    switch (field) {
        case 0: return FT_BASIC;
        case 1: return FT_BASIC;
        case 2: return FT_BASIC;
        case 3: return FT_BASIC;
        default: return FT_INVALID;
    }
}

const char *LocalLinkChangeNotifDescriptor::getFieldName(int field)
{
    if (baseclassdesc) {
        if (field < baseclassdesc->getFieldCount())
            return baseclassdesc->getFieldName(field);
        field -= baseclassdesc->getFieldCount();
    }
    switch (field) {
        case 0: return "linkIndex";
        case 1: return "nextHop";
        case 2: return "newCost";
        case 3: return "oldCost";
        default: return NULL;
    }
}

const char *LocalLinkChangeNotifDescriptor::getFieldTypeString(int field)
{
    if (baseclassdesc) {
        if (field < baseclassdesc->getFieldCount())
            return baseclassdesc->getFieldTypeString(field);
        field -= baseclassdesc->getFieldCount();
    }
    switch (field) {
        case 0: return "unsigned int";
        case 1: return "long";
        case 2: return "double";
        case 3: return "double";
        default: return NULL;
    }
}

const char *LocalLinkChangeNotifDescriptor::getFieldEnumName(int field)
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

int LocalLinkChangeNotifDescriptor::getArraySize(int field)
{
    if (baseclassdesc) {
        if (field < baseclassdesc->getFieldCount())
            return baseclassdesc->getArraySize(field);
        field -= baseclassdesc->getFieldCount();
    }
    LocalLinkChangeNotif *pp = (LocalLinkChangeNotif *)p;
    switch (field) {
        default: return 0;
    }
}

bool LocalLinkChangeNotifDescriptor::getFieldAsString(int field, int i, char *resultbuf, int bufsize)
{
    if (baseclassdesc) {
        if (field < baseclassdesc->getFieldCount())
            return baseclassdesc->getFieldAsString(field,i,resultbuf,bufsize);
        field -= baseclassdesc->getFieldCount();
    }
    LocalLinkChangeNotif *pp = (LocalLinkChangeNotif *)p;
    switch (field) {
        case 0: long2string(pp->getLinkIndex(),resultbuf,bufsize); return true;
        case 1: long2string(pp->getNextHop(),resultbuf,bufsize); return true;
        case 2: double2string(pp->getNewCost(),resultbuf,bufsize); return true;
        case 3: double2string(pp->getOldCost(),resultbuf,bufsize); return true;
        default: return false;
    }
}

bool LocalLinkChangeNotifDescriptor::setFieldAsString(int field, int i, const char *value)
{
    if (baseclassdesc) {
        if (field < baseclassdesc->getFieldCount())
            return baseclassdesc->setFieldAsString(field,i,value);
        field -= baseclassdesc->getFieldCount();
    }
    LocalLinkChangeNotif *pp = (LocalLinkChangeNotif *)p;
    switch (field) {
        case 0: pp->setLinkIndex(string2long(value)); return true;
        case 1: pp->setNextHop(string2long(value)); return true;
        case 2: pp->setNewCost(string2double(value)); return true;
        case 3: pp->setOldCost(string2double(value)); return true;
        default: return false;
    }
}

const char *LocalLinkChangeNotifDescriptor::getFieldStructName(int field)
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

void *LocalLinkChangeNotifDescriptor::getFieldStructPointer(int field, int i)
{
    if (baseclassdesc) {
        if (field < baseclassdesc->getFieldCount())
            return baseclassdesc->getFieldStructPointer(field, i);
        field -= baseclassdesc->getFieldCount();
    }
    LocalLinkChangeNotif *pp = (LocalLinkChangeNotif *)p;
    switch (field) {
        default: return NULL;
    }
}

sFieldWrapper *LocalLinkChangeNotifDescriptor::getFieldWrapper(int field, int i)
{
    return NULL;
}

