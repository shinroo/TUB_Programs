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
#include <omnetpp.h>
#include "protsim_defines.h"
#include "Application.h"
#include "../messages/PingMessage_m.h"

/**
 * Application that sends regular ping messages and replies to ping messages
 * it receives.
 * Interval may be configured by parameter \c interval.
 */
class PingPongAppl : public Application {
public:
    Module_Class_Members(PingPongAppl,Application,16384);

protected:
    virtual void activity();

private:
    cOutVector pingTTLs; ///< TTLs of received pings.
    cOutVector pongTTLs; ///< TTLs of received pongs.

    long    lastSentSeqNo; ///< Last sequence number in a sent ping.
    cPar    interval; ///< Interval of originated pings (from parameter \c interval)
    bool    disabled; ///< Sending is disabled if interval <= 0

    int     randGen;

    int     sent;       ///< Number of sent ping messages
    int     received;   ///< Number of received pong messages
};

//Define_Module_Like(PingPongAppl,Application);
Define_Module(PingPongAppl);

void PingPongAppl::activity() {
    char n[60];

    lastSentSeqNo = -1;
    interval = par("interval");
    sent = received = 0;
    WATCH(sent);
    WATCH(received);

    pingTTLs.setName("pingTTLs");
    pongTTLs.setName("pongTTLs");

    disabled = (interval.type() == 'B' && !interval.boolValue()) ||
	(interval.type() == 'L' && interval.longValue() <= 0) ||
	(interval.type() == 'D' && interval.doubleValue() <= 0.0);

    randGen = (par("applAddr").longValue() + networkStack->getNodeAddr()*4)%32;

    simtime_t next_send = DBL_MAX;
    if (!disabled)
	next_send = simTime() + uniform(0.0,interval.doubleValue(),randGen);

    while (true) {
	PingMessage * ping = NULL;
	
	while (next_send > simTime()) {
	    cMessage * msg = receive(next_send - simTime());
	    
	    if (msg != NULL) {
		ping = dynamic_cast<PingMessage*>(msg);
		assert(ping);

		if (ping->getReply()) {
		    pongTTLs.record(ping->getTTL());

		    ++received;
		    delete ping;
		    ping = NULL;
		}
		else {
		    pingTTLs.record(ping->getTTL());

		    ping->setDestNode(ping->getSrcNode());
		    ping->setDestAppl(ping->getSrcAppl());
		    ping->setReply(true);
		    snprintf(n,sizeof(n),"Pong %ld %ld.%ld->%ld.%ld",
			    ping->getSeqNo(),
			    ancestorPar("nodeAddr").longValue(),
			    par("applAddr").longValue(),
			    ping->getDestNode(),
			    ping->getDestAppl());
		    ping->setName(n);
		    ping->setBaseName("Pong");
		    send(ping,"out");
		}
	    }
	}

	assert(!disabled);
	next_send = simTime() + interval.doubleValue();
	ping = new PingMessage("Ping",eNETWORK_PACKET);
	ping->setBaseName("Ping");
	//packet->setLength(intuniform(800,8000));
	ping->setLength(8000);
	ping->setDestAppl(par("destAppl").longValue());
	ping->setDestNode(par("destNode").longValue());
	ping->setId(ePING);
	ping->setSeqNo(++lastSentSeqNo);
	ping->setReply(false);
	snprintf(n,sizeof(n),"Ping %ld %ld.%ld->%ld.%ld",
		 ping->getSeqNo(),
		 ancestorPar("nodeAddr").longValue(),
		 par("applAddr").longValue(),
		 ping->getDestNode(),
		 ping->getDestAppl());
	ping->setName(n);
	ping->setBaseName("Ping");
	++sent;
	send(ping,"out");
    }
}
