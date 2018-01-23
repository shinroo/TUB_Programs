//
// Generated file, do not edit! Created by opp_msgc 3.3 from NetworkPacket.msg.
//

// Disable warnings about unused variables, empty switch stmts, etc:
#ifdef _MSC_VER
#  pragma warning(disable:4101)
#  pragma warning(disable:4065)
#endif

#include <iostream>
#include <sstream>
#include "NetworkPacket_m.h"

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


Register_Class(NetworkPacket);

NetworkPacket::NetworkPacket(const char *name, int kind) : cMessage(name,kind)
{
    this->baseName_var = "";
    this->id_var = 0;
    this->srcNode_var = PS_NODE_UNKNOWN;
    this->srcAppl_var = PS_APP_UNKNOWN;
    this->destNode_var = PS_NODE_UNKNOWN;
    this->destAppl_var = PS_APP_UNKNOWN;
    this->alert_var = false;
    this->lastNode_var = PS_NODE_UNKNOWN;
    this->TTL_var = 0;
}

NetworkPacket::NetworkPacket(const NetworkPacket& other) : cMessage()
{
    unsigned int i;
    setName(other.name());
    operator=(other);
}

NetworkPacket::~NetworkPacket()
{
    unsigned int i;
}

NetworkPacket& NetworkPacket::operator=(const NetworkPacket& other)
{
    if (this==&other) return *this;
    unsigned int i;
    cMessage::operator=(other);
    this->baseName_var = other.baseName_var;
    this->id_var = other.id_var;
    this->srcNode_var = other.srcNode_var;
    this->srcAppl_var = other.srcAppl_var;
    this->destNode_var = other.destNode_var;
    this->destAppl_var = other.destAppl_var;
    this->alert_var = other.alert_var;
    this->lastNode_var = other.lastNode_var;
    this->TTL_var = other.TTL_var;
    return *this;
}

void NetworkPacket::netPack(cCommBuffer *b)
{
    cMessage::netPack(b);
    doPacking(b,this->baseName_var);
    doPacking(b,this->id_var);
    doPacking(b,this->srcNode_var);
    doPacking(b,this->srcAppl_var);
    doPacking(b,this->destNode_var);
    doPacking(b,this->destAppl_var);
    doPacking(b,this->alert_var);
    doPacking(b,this->lastNode_var);
    doPacking(b,this->TTL_var);
}

void NetworkPacket::netUnpack(cCommBuffer *b)
{
    cMessage::netUnpack(b);
    doUnpacking(b,this->baseName_var);
    doUnpacking(b,this->id_var);
    doUnpacking(b,this->srcNode_var);
    doUnpacking(b,this->srcAppl_var);
    doUnpacking(b,this->destNode_var);
    doUnpacking(b,this->destAppl_var);
    doUnpacking(b,this->alert_var);
    doUnpacking(b,this->lastNode_var);
    doUnpacking(b,this->TTL_var);
}

const char * NetworkPacket::getBaseName() const
{
    return baseName_var.c_str();
}

void NetworkPacket::setBaseName(const char * baseName_var)
{
    this->baseName_var = baseName_var;
}

long NetworkPacket::getId() const
{
    return id_var;
}

void NetworkPacket::setId(long id_var)
{
    this->id_var = id_var;
}

long NetworkPacket::getSrcNode() const
{
    return srcNode_var;
}

void NetworkPacket::setSrcNode(long srcNode_var)
{
    this->srcNode_var = srcNode_var;
}

long NetworkPacket::getSrcAppl() const
{
    return srcAppl_var;
}

void NetworkPacket::setSrcAppl(long srcAppl_var)
{
    this->srcAppl_var = srcAppl_var;
}

long NetworkPacket::getDestNode() const
{
    return destNode_var;
}

void NetworkPacket::setDestNode(long destNode_var)
{
    this->destNode_var = destNode_var;
}

long NetworkPacket::getDestAppl() const
{
    return destAppl_var;
}

void NetworkPacket::setDestAppl(long destAppl_var)
{
    this->destAppl_var = destAppl_var;
}

bool NetworkPacket::getAlert() const
{
    return alert_var;
}

void NetworkPacket::setAlert(bool alert_var)
{
    this->alert_var = alert_var;
}

long NetworkPacket::getLastNode() const
{
    return lastNode_var;
}

void NetworkPacket::setLastNode(long lastNode_var)
{
    this->lastNode_var = lastNode_var;
}

long NetworkPacket::getTTL() const
{
    return TTL_var;
}

void NetworkPacket::setTTL(long TTL_var)
{
    this->TTL_var = TTL_var;
}

class NetworkPacketDescriptor : public cStructDescriptor
{
  public:
    NetworkPacketDescriptor();
    virtual ~NetworkPacketDescriptor();
    NetworkPacketDescriptor& operator=(const NetworkPacketDescriptor& other);
    virtual cPolymorphic *dup() const {return new NetworkPacketDescriptor(*this);}

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

Register_Class(NetworkPacketDescriptor);

NetworkPacketDescriptor::NetworkPacketDescriptor() : cStructDescriptor("cMessage")
{
}

NetworkPacketDescriptor::~NetworkPacketDescriptor()
{
}

int NetworkPacketDescriptor::getFieldCount()
{
    return baseclassdesc ? 9+baseclassdesc->getFieldCount() : 9;
}

int NetworkPacketDescriptor::getFieldType(int field)
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
        case 4: return FT_BASIC;
        case 5: return FT_BASIC;
        case 6: return FT_BASIC;
        case 7: return FT_BASIC;
        case 8: return FT_BASIC;
        default: return FT_INVALID;
    }
}

const char *NetworkPacketDescriptor::getFieldName(int field)
{
    if (baseclassdesc) {
        if (field < baseclassdesc->getFieldCount())
            return baseclassdesc->getFieldName(field);
        field -= baseclassdesc->getFieldCount();
    }
    switch (field) {
        case 0: return "baseName";
        case 1: return "id";
        case 2: return "srcNode";
        case 3: return "srcAppl";
        case 4: return "destNode";
        case 5: return "destAppl";
        case 6: return "alert";
        case 7: return "lastNode";
        case 8: return "TTL";
        default: return NULL;
    }
}

const char *NetworkPacketDescriptor::getFieldTypeString(int field)
{
    if (baseclassdesc) {
        if (field < baseclassdesc->getFieldCount())
            return baseclassdesc->getFieldTypeString(field);
        field -= baseclassdesc->getFieldCount();
    }
    switch (field) {
        case 0: return "string";
        case 1: return "long";
        case 2: return "long";
        case 3: return "long";
        case 4: return "long";
        case 5: return "long";
        case 6: return "bool";
        case 7: return "long";
        case 8: return "long";
        default: return NULL;
    }
}

const char *NetworkPacketDescriptor::getFieldEnumName(int field)
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

int NetworkPacketDescriptor::getArraySize(int field)
{
    if (baseclassdesc) {
        if (field < baseclassdesc->getFieldCount())
            return baseclassdesc->getArraySize(field);
        field -= baseclassdesc->getFieldCount();
    }
    NetworkPacket *pp = (NetworkPacket *)p;
    switch (field) {
        default: return 0;
    }
}

bool NetworkPacketDescriptor::getFieldAsString(int field, int i, char *resultbuf, int bufsize)
{
    if (baseclassdesc) {
        if (field < baseclassdesc->getFieldCount())
            return baseclassdesc->getFieldAsString(field,i,resultbuf,bufsize);
        field -= baseclassdesc->getFieldCount();
    }
    NetworkPacket *pp = (NetworkPacket *)p;
    switch (field) {
        case 0: oppstring2string(pp->getBaseName(),resultbuf,bufsize); return true;
        case 1: long2string(pp->getId(),resultbuf,bufsize); return true;
        case 2: long2string(pp->getSrcNode(),resultbuf,bufsize); return true;
        case 3: long2string(pp->getSrcAppl(),resultbuf,bufsize); return true;
        case 4: long2string(pp->getDestNode(),resultbuf,bufsize); return true;
        case 5: long2string(pp->getDestAppl(),resultbuf,bufsize); return true;
        case 6: bool2string(pp->getAlert(),resultbuf,bufsize); return true;
        case 7: long2string(pp->getLastNode(),resultbuf,bufsize); return true;
        case 8: long2string(pp->getTTL(),resultbuf,bufsize); return true;
        default: return false;
    }
}

bool NetworkPacketDescriptor::setFieldAsString(int field, int i, const char *value)
{
    if (baseclassdesc) {
        if (field < baseclassdesc->getFieldCount())
            return baseclassdesc->setFieldAsString(field,i,value);
        field -= baseclassdesc->getFieldCount();
    }
    NetworkPacket *pp = (NetworkPacket *)p;
    switch (field) {
        case 0: pp->setBaseName((value)); return true;
        case 1: pp->setId(string2long(value)); return true;
        case 2: pp->setSrcNode(string2long(value)); return true;
        case 3: pp->setSrcAppl(string2long(value)); return true;
        case 4: pp->setDestNode(string2long(value)); return true;
        case 5: pp->setDestAppl(string2long(value)); return true;
        case 6: pp->setAlert(string2bool(value)); return true;
        case 7: pp->setLastNode(string2long(value)); return true;
        case 8: pp->setTTL(string2long(value)); return true;
        default: return false;
    }
}

const char *NetworkPacketDescriptor::getFieldStructName(int field)
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

void *NetworkPacketDescriptor::getFieldStructPointer(int field, int i)
{
    if (baseclassdesc) {
        if (field < baseclassdesc->getFieldCount())
            return baseclassdesc->getFieldStructPointer(field, i);
        field -= baseclassdesc->getFieldCount();
    }
    NetworkPacket *pp = (NetworkPacket *)p;
    switch (field) {
        default: return NULL;
    }
}

sFieldWrapper *NetworkPacketDescriptor::getFieldWrapper(int field, int i)
{
    return NULL;
}

