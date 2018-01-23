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

#ifndef GOBACKN_SENDER_H
#define GOBACKN_SENDER_H

#include <map>
#include <omnetpp.h>
#include "protsim_defines.h"
#include "Application.h"
#include "../messages/ARQMessage_m.h"

class GoBackNSender : public Application {
public:
    GoBackNSender(const char *name=0, cModule *parent=0)
        : Application(name,parent,0), nextSendTimer(NULL),
          expirationTimer(NULL) { }

    virtual ~GoBackNSender() {
        cancelAndDelete(nextSendTimer);
        cancelAndDelete(expirationTimer);
    }
    
protected:
    virtual void initialize();
    virtual void handleMessage(cMessage * msg);

    virtual ARQData * getDataPacket(long seqNo);
    virtual void freeBuffers(long start, long end);
    virtual void resetTimers();
    void printBuffers();

private:
    struct PacketInfo {
	ARQData * packet;
	simtime_t timeout;

	PacketInfo() : packet(NULL), timeout(-1) {}
	virtual ~PacketInfo() { delete packet; packet = NULL; }
    };

    typedef std::map<long,PacketInfo> PacketBuffer;

    PacketBuffer packetBuffer;

    long    packetSize;
    double  timeout;
    double  interval;
    long    window;

    bool    disabled; ///< Sending is disabled if packetSize <= 0

    long    lastAckSeqNo;
    long    nextSendSeqNo;

    simtime_t nextSendTime, timerExpiration;
    cMessage * nextSendTimer;
    cMessage * expirationTimer;

    long    sentBytes;
    long    ackBytes;

    cOutVector sendSeqNumVector;
    cOutVector ackSeqNumVector;
    cOutVector nackSeqNumVector;
};

#endif /* GOBACKN_SENDER_H */
