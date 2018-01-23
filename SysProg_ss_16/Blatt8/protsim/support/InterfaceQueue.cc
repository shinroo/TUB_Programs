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
#include <limits.h>
#include <omnetpp.h>
#include "protsim_defines.h"

class InterfaceQueue: public cSimpleModule {
public:
    InterfaceQueue(const char *name=0, cModule *parent=0)
        : cSimpleModule(name,parent,0)
          ,waitingTimeSumVector(NULL)
          ,timeout(NULL)
        {}

    virtual ~InterfaceQueue();

protected:
    virtual void initialize();
    virtual void handleMessage(cMessage * msg);
    virtual void finish();

    virtual void detect();
    virtual void detectTransient();
    virtual void detectBatch();

    virtual void resetStatistics();
    virtual void recordStatistics(int batch = -1);

private:
    long            maxQueueLength;

    long            transientRemoval;
    bool            transientOver;

    long            batchLength;
    int             batchNum;

    cOutVector      queueLengthVector;
    cOutVector      waitingTimeVector;
    cOutVector *    waitingTimeSumVector;

    simtime_t       lastUtilChange;
    simtime_t       lastQueueChange;

    cWeightedStdDev linkUtilizationStat;
    cWeightedStdDev queueLengthStat;
    cStdDev         waitingTimeStat;

    cLongHistogram  queueLengthHist;

    //*******************************************
    // Your task:
    // - Copy your additions from last assignment
    // - Please note the additions above
    // - Add what you need
    //*******************************************

    long            totalMessages;
    long            droppedMessages;

    cQueue          queue;
    cGate *         watchedGate;

    cMessage * timeout;
    // END YOUR TASK
}; // InterfaceQueue

Define_Module(InterfaceQueue);


void InterfaceQueue::initialize() {
    maxQueueLength = par("maxQueueLength").longValue();

    transientRemoval = par("transientRemoval").longValue();
    transientOver = (transientRemoval <= 0);

    batchLength = par("batchLength").longValue();
    batchNum = 0;

    // Initialize vectors, statistics etc.
    queueLengthVector.setName("queue length");
    if (!transientOver) queueLengthVector.disable();

    waitingTimeVector.setName("waiting time");
    if (!transientOver) waitingTimeVector.disable();

    waitingTimeSumVector = new cOutVector("waiting time sum",2);
    if (!transientOver) waitingTimeSumVector->disable();

    lastUtilChange = 0.0;
    lastQueueChange = 0.0;

    linkUtilizationStat.setName("link utilization");
    queueLengthStat.setName("queue length");
    waitingTimeStat.setName("waiting time");

    queueLengthHist.setName("queue length");
    queueLengthHist.setNumCells(maxQueueLength+1);
    queueLengthHist.setRange(0,maxQueueLength);

    //*******************************************
    // Your task:
    // - Copy your additions from last assignment
    // - Please note the additions above
    // - Add what you need
    //*******************************************

    // Detect connection with bandwidth
    watchedGate = gate("out");
    while (watchedGate != NULL && watchedGate->datarate() == NULL) {
	watchedGate = watchedGate->toGate();
    }

    // If there is none just take our output gate
    if (watchedGate == NULL) watchedGate = gate("out");

    totalMessages = droppedMessages = 0;
    WATCH(totalMessages);
    WATCH(droppedMessages);

    // Initialize timeout
    timeout = new cMessage("timeout");
    timeout->setPriority(INT_MIN);
}


void InterfaceQueue::handleMessage(cMessage * msg) {
    //*******************************************
    // Your task:
    // - Copy your additions from last assignment
    // - Modify the code as necessary
    //*******************************************

    if (msg == timeout) {
	if (!queue.empty()) {
	    queueLengthVector.record(queue.length());
	    queueLengthStat.collect2(queue.length(),simTime()-lastQueueChange);
	    lastQueueChange = simTime();
	    msg = dynamic_cast<cMessage*>(queue.pop()); // we put only message in so cast will always work
	    queueLengthVector.record(queue.length());
	    queueLengthHist.collect(queue.length());
	    waitingTimeVector.record(simTime()-msg->arrivalTime());
	    waitingTimeStat.collect(simTime()-msg->arrivalTime());
	    waitingTimeSumVector->record(waitingTimeStat.samples(),
					 waitingTimeStat.sum());
	    detect();
	    send(msg,"out");
	    scheduleAt(watchedGate->transmissionFinishes(),timeout);
	}
	else {
	    linkUtilizationStat.collect2(1,simTime()-lastUtilChange);
	    lastUtilChange = simTime();
	}
    }
    else {
	++totalMessages;
	if (queue.empty() &&
	    watchedGate->transmissionFinishes() <= simTime()) {

	    linkUtilizationStat.collect2(0,simTime()-lastUtilChange);
	    lastUtilChange = simTime();

	    waitingTimeVector.record(0.0);
	    waitingTimeStat.collect(0.0);
	    waitingTimeSumVector->record(waitingTimeStat.samples(),
					 waitingTimeStat.sum());

	    queueLengthVector.record(0.0);
	    queueLengthHist.collect(0.0);

	    detect();

	    send(msg,"out");
	    scheduleAt(watchedGate->transmissionFinishes(),timeout);
	}
	else {
	    if (queue.length() < maxQueueLength) {
		queueLengthStat.collect2(queue.length(),simTime()-lastQueueChange);
		lastQueueChange = simTime();
		queueLengthVector.record(queue.length());
		queue.insert(msg);
		queueLengthVector.record(queue.length());
		queueLengthHist.collect(queue.length());
	    }
	    else {
		ev << "Message " << msg->name() << " dropped in queue "
		   << fullPath() << "\n";
		delete msg;
		++droppedMessages;
	    }
	}
    }
}


void InterfaceQueue::detectTransient() {
    //***********************************************************
    // Your task: Insert the transient detection here
    //***********************************************************

    if (!transientOver && waitingTimeStat.samples() >= transientRemoval) {
	transientOver = true;

	queueLengthVector.enable();
	waitingTimeVector.enable();

	waitingTimeSumVector->enable();

	resetStatistics();
    }
    //END

    assert(transientOver || waitingTimeStat.samples() < transientRemoval);
}


void InterfaceQueue::detectBatch() {
    //***********************************************************
    // Your task: Insert the end of batch detection here
    //***********************************************************
    if (transientOver && batchLength > 0 &&
	waitingTimeStat.samples() >= batchLength) {

	recordStatistics(batchNum);
	resetStatistics();
	batchNum++;
    }

    assert(batchLength <= 0 || waitingTimeStat.samples() < batchLength);
}


void InterfaceQueue::resetStatistics() {
    //***********************************************************
    // Your task: Reset statistics, counters etc. here
    //***********************************************************

    lastUtilChange = simTime();
    lastQueueChange = simTime();
    
    linkUtilizationStat.clearResult();
    queueLengthStat.clearResult();
    waitingTimeStat.clearResult();

    // Note for ass.7: If student does not take the sum and/or sample count
    //                 for the waitingTimeSumVector from the waitingTimeStat
    //                 but has his/her own counter, these must be reset here
    queueLengthHist.clearResult();

    totalMessages = droppedMessages = 0;

    queueLengthHist.setNumCells(maxQueueLength+1);
    queueLengthHist.setRange(0,maxQueueLength);
}


void InterfaceQueue::recordStatistics(int batch) {
    //***********************************************************
    // Your task: Add your code from last time here
    //***********************************************************

    if (watchedGate->transmissionFinishes() < simTime())
	linkUtilizationStat.collect2(0,simTime()-lastUtilChange);
    else
	linkUtilizationStat.collect2(1,simTime()-lastUtilChange);
    queueLengthStat.collect2(queue.length(),simTime()-lastQueueChange);
    //END

    lastUtilChange = simTime();
    lastQueueChange = simTime();

    char n[80];
    snprintf(n,80,"%s.Batch%d", linkUtilizationStat.name(), batch);
#ifdef OMNET30
    linkUtilizationStat.recordScalar(n);
#else
    recordStats(n, &linkUtilizationStat);
#endif
    recordScalar(n, linkUtilizationStat.mean());
    snprintf(n,80,"%s.Batch%d", queueLengthStat.name(), batch);
#ifdef OMNET30
    queueLengthStat.recordScalar(n);
#else
    recordStats(n, &queueLengthStat);
#endif
    recordScalar(n, queueLengthStat.mean());
    snprintf(n,80,"%s.Batch%d", waitingTimeStat.name(), batch);
#ifdef OMNET30
    waitingTimeStat.recordScalar(n);
#else
    recordStats(n, &waitingTimeStat);
#endif
    recordScalar(n, waitingTimeStat.mean());

    snprintf(n,80,"%s.Batch%d", queueLengthHist.name(), batch);
#ifdef OMNET30
    queueLengthHist.recordScalar(n);
#else
    recordStats(n, &queueLengthHist);
#endif
}


void InterfaceQueue::detect() {
    detectTransient();
    detectBatch();
}


void InterfaceQueue::finish() {
}

InterfaceQueue::~InterfaceQueue() {
    cancelAndDelete(timeout);
    timeout = NULL;
    delete waitingTimeSumVector;
    waitingTimeSumVector = NULL;
}
