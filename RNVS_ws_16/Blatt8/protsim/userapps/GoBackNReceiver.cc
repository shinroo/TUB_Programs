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
#include "GoBackNReceiver.h"
#include "../messages/ARQMessage_m.h"


Define_Module(GoBackNReceiver);

void GoBackNReceiver::initialize() {
    Application::initialize();

    ackSize = par("ackSize").longValue();
    sendNack = par("sendNack").boolValue();

    lastReceivedSeqNo = -1;
    WATCH(lastReceivedSeqNo);
    totalBytes = goodBytes = 0;
    WATCH(totalBytes);
    WATCH(goodBytes);

    receivedSeqNumVector.setName("receivedSeqNumVector");
    corruptedSeqNumVector.setName("corruptedSeqNumVector");
    outOfOrderSeqNumVector.setName("outOfOrderSeqNumVector");
}

void GoBackNReceiver::sendAck(NetworkPacket * packet, long expected) {
    char n[60];

    ARQAck  * ack = new ARQAck("Ack",eNETWORK_PACKET);
    ack->setBaseName("Ack");
    ack->setLength(ackSize);
    ack->setDestAppl(packet->getSrcAppl());
    ack->setDestNode(packet->getSrcNode());
    ack->setId(eACK);
    ack->setSeqNoExpected(expected);
    //ack->setLoss(nack);
    snprintf(n,sizeof(n),"Ack %ld %ld.%ld->%ld.%ld",
	     ack->getSeqNoExpected(),
	     ancestorPar("nodeAddr").longValue(),
	     par("applAddr").longValue(),
	     ack->getDestNode(),
	     ack->getDestAppl());
    ack->setName(n);
    send(ack,"out");
}

void GoBackNReceiver::handleMessage(cMessage * msg) {
    ARQData * data = dynamic_cast<ARQData*>(msg);
    assert(data);
    assert(data->getId() == eDATA);
    receivedSeqNumVector.record(data->getSeqNo());

    /*************** Your task ****************
     *  - wait for data packets
     *  - if the packet has bit errors: discard
     *  - if the packet is not the one expected: discard
     *  - if the packet is the next one expected:
     *    send an acknowledgement
     */

    totalBytes += data->length()/8;
    if (!data->hasBitError()) {
        // Good packet: ACK
        if (data->getSeqNo() == lastReceivedSeqNo+1) {
            goodBytes += data->length()/8;
            lastReceivedSeqNo = data->getSeqNo();
            sendAck(data,data->getSeqNo()+1);
        }
        // Out of order: NACK
        else {
            // Packet too new (old ones ignored)
            if (data->getSeqNo() > lastReceivedSeqNo+1) {
                ev << "Out of order message " << msg->name() << " received in " << fullPath() << "\n";
                outOfOrderSeqNumVector.record(data->getSeqNo());

                if (sendNack && data->getSeqNo() == lastReceivedSeqNo+2) {
                    sendAck(data,lastReceivedSeqNo+1);
                }
            }
        }
    }
    // corrupted packet
    else {
        ev << "Corrupted message " << msg->name() << " received in " << fullPath() << "\n";
        corruptedSeqNumVector.record(data->getSeqNo());
    }
        
    delete data;
    data = NULL;
    msg = NULL;

    assert(msg == NULL);
}

void GoBackNReceiver::finish() {
    recordScalar("goodBytes",goodBytes);
    recordScalar("goodput",goodBytes/simTime());
    recordScalar("totalBytes",totalBytes);
    recordScalar("throughput",totalBytes/simTime());
}
