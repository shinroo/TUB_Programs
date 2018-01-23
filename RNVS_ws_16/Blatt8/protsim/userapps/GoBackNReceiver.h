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

#ifndef GOBACKN_RECEIVER_H
#define GOBACKN_RECEIVER_H

#include <omnetpp.h>
#include "protsim_defines.h"
#include "Application.h"

class GoBackNReceiver : public Application {
public:
    Module_Class_Members(GoBackNReceiver,Application,0);

protected:
    virtual void initialize();
    virtual void handleMessage(cMessage * msg);
    virtual void finish();

    virtual void sendAck(NetworkPacket * packet, long expected);

private:
    long    ackSize;
    bool    sendNack;

    long    lastReceivedSeqNo;
    long    totalBytes;
    long    goodBytes;

    cOutVector receivedSeqNumVector;
    cOutVector corruptedSeqNumVector;
    cOutVector outOfOrderSeqNumVector;
};

#endif /* GOBACKN_RECEIVER_H */
