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

#include <cassert>
#include <cfloat>
#include "GoBackNSender.h"

Define_Module(GoBackNSender);

void GoBackNSender::printBuffers() {
    for (PacketBuffer::iterator i = packetBuffer.begin();
	 i != packetBuffer.end(); ++i) {
	printf("%ld -> %s\n",i->first,i->second.packet ? i->second.packet->name() : "NONE");
    }
}

ARQData * GoBackNSender::getDataPacket(long seqNo) {
    char n[60];

    // We never insert packets older than the oldest one in the buffer
    assert(packetBuffer.empty() || seqNo >= packetBuffer.begin()->first);
    ARQData * data = packetBuffer[seqNo].packet;
    assert(packetBuffer.size() <= (unsigned)window);

    // create new data packet if necessary
    if (data == NULL) {
	data = new ARQData("Data",eNETWORK_PACKET);
	data->setBaseName("Data");
	data->setLength(packetSize);
	data->setDestAppl(par("destAppl").longValue());
	data->setDestNode(par("destNode").longValue());
	data->setId(eDATA);
	data->setSeqNo(seqNo);
	snprintf(n,sizeof(n),"Data %ld %ld.%ld->%ld.%ld",
		 data->getSeqNo(),
		 ancestorPar("nodeAddr").longValue(),
		 par("applAddr").longValue(),
		 data->getDestNode(),
		 data->getDestAppl());
	data->setName(n);
	packetBuffer[seqNo].packet = data;
    }

    return new ARQData(*data);
}

void GoBackNSender::freeBuffers(long start, long end) {
    assert(start <= end);

    for (long i = start; i <= end; ++i) {
	int removed = packetBuffer.erase(i);
	assert(removed == 1);
    }
}

void GoBackNSender::resetTimers() {
    timerExpiration = INFINITY;      
    for (PacketBuffer::iterator i = packetBuffer.begin();
	 i != packetBuffer.end(); ++i) {
	i->second.timeout = INFINITY;
    }
    if (expirationTimer->isScheduled())
	cancelEvent(expirationTimer);
}


void GoBackNSender::initialize() {
    Application::initialize();

    lastAckSeqNo = nextSendSeqNo = 0;
    WATCH(lastAckSeqNo);
    WATCH(nextSendSeqNo);

    packetSize = par("packetSize").longValue();
    timeout = par("timeout").doubleValue();
    interval = par("interval").doubleValue();
    window = par("window").longValue();
    assert(window > 0);

    sentBytes = ackBytes = 0;
    WATCH(sentBytes);
    WATCH(ackBytes);

    nextSendTime = 0.0;
    timerExpiration = INFINITY;
    WATCH(nextSendTime);
    WATCH(timerExpiration);

    disabled = (par("packetSize").type() == 'B' && !par("packetSize").boolValue()) ||
	(par("packetSize").type() == 'L' && par("packetSize").longValue() <= 0) ||
	(par("packetSize").type() == 'D' && par("packetSize").doubleValue() <= 0.0);

    sendSeqNumVector.setName("sendSeqNumVector");
    ackSeqNumVector.setName("ackSeqNumVector");
    nackSeqNumVector.setName("nackSeqNumVector");

    if (!disabled) {
	nextSendTimer = new cMessage("nextSendTimer",eTIMER0);
	scheduleAt(simTime(),nextSendTimer);
    }
    expirationTimer = new cMessage("expirationTimer",eTIMER1);
}


void GoBackNSender::handleMessage(cMessage * msg) {
    switch (msg->kind()) {
	case eTIMER0: { // time for sending next packet
	    assert(simTime() == nextSendTime);

	    // Window must have room at this time
	    assert(nextSendSeqNo < lastAckSeqNo + window);

	    //************** Your task ******************
	    // - get data packet
	    // - send data packet
	    // - update timers and schedule next send
	    //*******************************************

	    ARQData * data = getDataPacket(nextSendSeqNo);

	    // Send data
	    sentBytes += packetSize/8;
	    ev << "Sending message " << data->name() << "\n";
	    sendSeqNumVector.record(data->getSeqNo());
	    send(data,"out");

	    // Update timers
	    simtime_t nextTimer = simTime()+timeout;
	    packetBuffer[nextSendSeqNo].timeout = nextTimer;
	    ++nextSendSeqNo;
	    if (nextTimer < timerExpiration) {
		if (expirationTimer->isScheduled())
		    cancelEvent(expirationTimer);
		timerExpiration = nextTimer;
		scheduleAt(timerExpiration,expirationTimer);
	    }
	    
	    // Schedule next send
	    if (nextSendSeqNo >= lastAckSeqNo + window) // window exhausted
		nextSendTime = INFINITY;
	    else {
		nextSendTime = simTime() + interval;
		scheduleAt(nextSendTime,nextSendTimer);
	    }
	    

	    break;
	}
	case eTIMER1: {
	    assert(simTime() == timerExpiration);
	    
	    //************** Your task ******************
	    // - if a timeout occurs resend packets
	    //*******************************************

	    ev << "Timeout, resending\n";
	    if (nextSendTime == INFINITY) {
		nextSendTime = simTime();
		scheduleAt(nextSendTime,nextSendTimer);
	    }
	    nextSendSeqNo = lastAckSeqNo;
	    resetTimers();


	    break;
	}
	case eNETWORK_PACKET: {
	    //************** Your task ******************
	    // - if a positive ack. arrives free buffers
	    //   and update timers, nextSendSeqNo etc.
	    // - optional part: if the last received
	    //   ack. arrives again resend packets
	    //*******************************************

	    ARQAck  * ack = dynamic_cast<ARQAck*>(msg);
	    assert(ack);

	    // NOTE this check
	    if (!ack->hasBitError()) {
		// This is a reject
		if (ack->getSeqNoExpected() == lastAckSeqNo) {
		    nackSeqNumVector.record(ack->getSeqNoExpected());
		    ev << "Received reject " << ack->name() << "\n";
		    if (nextSendTime == INFINITY) {
			nextSendTime = simTime();
			scheduleAt(nextSendTime,nextSendTimer);
		    }
		    nextSendSeqNo = lastAckSeqNo;
		}
		// This is a real ack
		else {
		    assert(ack->getSeqNoExpected() > lastAckSeqNo);
		    ackSeqNumVector.record(ack->getSeqNoExpected());
		    ev << "Received ack " << ack->name() << "\n";
		    freeBuffers(lastAckSeqNo,ack->getSeqNoExpected()-1);
		    ackBytes += (packetSize/8)*(ack->getSeqNoExpected()-lastAckSeqNo);
		    lastAckSeqNo = ack->getSeqNoExpected();
		    if (expirationTimer->isScheduled())
			cancelEvent(expirationTimer);
		    if (nextSendSeqNo > lastAckSeqNo) {
			timerExpiration = packetBuffer[lastAckSeqNo].timeout;
			assert(timerExpiration>0.0);
			assert(timerExpiration>simTime());
			scheduleAt(timerExpiration,expirationTimer);
		    }
		    else {
			timerExpiration = INFINITY;
			if (nextSendSeqNo < lastAckSeqNo) {
			    nextSendSeqNo = lastAckSeqNo;
			}
		    }
		    if (nextSendTime == INFINITY) {
			nextSendTime = simTime();
			scheduleAt(nextSendTime,nextSendTimer);
		    }
		}
	    }
	    delete ack;
	    ack = NULL;

	    break;
	}
	default: assert(false);
    }
}
