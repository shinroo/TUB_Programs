//=========================================================================
//  CSOCKETRTSCHEDULER.CC - part of
//
//                  OMNeT++/OMNEST
//           Discrete System Simulation in C++
//
//   Written by:  Andras Varga, 2005
//
//=========================================================================

/*--------------------------------------------------------------*
  Copyright (C) 2005 Andras Varga

  This file is distributed WITHOUT ANY WARRANTY. See the file
  `license' for details on this and other legal matters.
  *--------------------------------------------------------------*/

#include <cassert>
#include "ProtSimSocketRTScheduler.h"
#include "NetworkStack.h"
#include "ExternalApplication.h"

Register_Class(ProtSimSocketRTScheduler);


inline std::ostream& operator<<(std::ostream& ev, const timeval& tv)
{
    return ev << (unsigned long)tv.tv_sec << "s" << tv.tv_usec << "us";
}

//---

ProtSimSocketRTScheduler::ProtSimSocketRTScheduler()
    : cScheduler(), listenerSocket(INVALID_SOCKET) { }

ProtSimSocketRTScheduler::~ProtSimSocketRTScheduler()
{
    for (ApplBySocket::iterator i = applBySocket.begin(); i != applBySocket.end(); ++i) {
        delete i->second;
    }
}

void ProtSimSocketRTScheduler::startRun()
{
    if (initsocketlibonce()!=0)
        throw new cRuntimeError("ProtSimSocketRTScheduler: Cannot initialize socket library");

    gettimeofday(&baseTime, NULL);

    port = ev.config()->getAsInt("General", "socketrtscheduler-port", 4343);
    numApplPerNode = ev.config()->getAsInt("General", "socketrtscheduler-appl-per-node", 10);
    numNodes = ev.config()->getAsInt("General", "socketrtscheduler-num-nodes", 20);
    setupListener();
}

void ProtSimSocketRTScheduler::endRun()
{
    for (ApplBySocket::iterator i = applBySocket.begin(); i != applBySocket.end(); ++i) {
	shutdownApplication(i->second, true);
    }
}

void ProtSimSocketRTScheduler::executionResumed()
{
    gettimeofday(&baseTime, NULL);
    baseTime = timeval_substract(baseTime, sim->simTime());
}

cMessage *ProtSimSocketRTScheduler::getNextEvent()
{
    // calculate target time
    timeval targetTime;
    cMessage *msg = sim->msgQueue.peekFirst();
    if (!msg) {
        // if there are no events, wait until something comes from outside
        // TBD: obey simtimelimit, cpu-time-limit
        targetTime.tv_sec = LONG_MAX;
        targetTime.tv_usec = 0;
    }
    else {
        // use time of next event
        simtime_t eventSimtime = msg->arrivalTime();
        targetTime = timeval_add(baseTime, eventSimtime);
    }

    // if needed, wait until that time arrives
    timeval curTime;
    gettimeofday(&curTime, NULL);
    if (timeval_greater(targetTime, curTime)) {
        int status = receiveUntil(targetTime);
        if (status == -1)
            return NULL; // interrupted by user
        if (status == 1)
            msg = sim->msgQueue.peekFirst(); // received something
    }
    else {
        // we're behind -- customized versions of this class may
        // alert if we're too much behind, whatever that means
    }

    // ok, return the message
    return msg;
}


void ProtSimSocketRTScheduler::setupListener()
{
    listenerSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (listenerSocket==INVALID_SOCKET)
        throw new cRuntimeError("ProtSimSocketRTScheduler: cannot create socket");
    int val = 1;
    if (setsockopt(listenerSocket, SOL_SOCKET, SO_REUSEADDR, &val, sizeof(val)) < 0)
        throw new cRuntimeError("ProtSimSocketRTScheduler: cannot set SO_REUSEADDR on socket");

    sockaddr_in sinInterface;
    sinInterface.sin_family = AF_INET;
    sinInterface.sin_addr.s_addr = INADDR_ANY;
    sinInterface.sin_port = htons(port);
    if (bind(listenerSocket, (sockaddr*)&sinInterface, sizeof(sockaddr_in))==SOCKET_ERROR)
        throw new cRuntimeError("ProtSimSocketRTScheduler: socket bind() failed");

    listen(listenerSocket, SOMAXCONN);
}

void ProtSimSocketRTScheduler::acceptConnection() {
    // accept connection, and store FD in connSocket
    sockaddr_in sinRemote;
    int addrSize = sizeof(sinRemote);
    SOCKET connSocket = accept(listenerSocket, (sockaddr*)&sinRemote, (socklen_t*)&addrSize);
    if (connSocket==INVALID_SOCKET)
        throw new cRuntimeError("ProtSimSocketRTScheduler: accept() failed");
    ev << "ProtSimSocketRTScheduler: connected!\n";
    ApplicationDesc* applDesc = new ApplicationDesc;
    applDesc->socket = connSocket;
    applDesc->ipAddr = ntohl(sinRemote.sin_addr.s_addr);
    applDesc->port = ntohs(sinRemote.sin_port);
    try {
        mapApplAddr(applDesc->port, &applDesc->nodeAddr, &applDesc->applAddr);
        createApplication(applDesc);
        registerApplication(applDesc);
    }
    catch (...) {
        delete applDesc;
        throw;
    }
}

void ProtSimSocketRTScheduler::fillFdSet(fd_set* set) {
    for (ApplBySocket::const_iterator i = applBySocket.begin(); i != applBySocket.end(); ++i) {
        FD_SET(i->first, set);
    }
}

bool ProtSimSocketRTScheduler::receiveWithTimeout(long usec)
{
    // prepare sets for select()
    fd_set readFDs, writeFDs, exceptFDs;
    FD_ZERO(&readFDs);
    FD_ZERO(&writeFDs);
    FD_ZERO(&exceptFDs);

    FD_SET(listenerSocket, &readFDs);
    fillFdSet(&readFDs);

    timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = usec;

    if (select(FD_SETSIZE, &readFDs, &writeFDs, &exceptFDs, &timeout) > 0) {
        // Something happened on one of the sockets -- handle them
        if (FD_ISSET(listenerSocket, &readFDs)) {
            acceptConnection();
        }
        else {
            for (ApplBySocket::const_iterator i = applBySocket.begin(); i != applBySocket.end(); ++i) {
                SOCKET socket = i->first;

                if (FD_ISSET(socket, &readFDs)) {
                    // receive from socket
                    if (recv(socket, recvBuffer, sizeof(uint32_t), MSG_PEEK) ==SOCKET_ERROR) {
                        ev << "ProtSimSocketRTScheduler: socket error " << sock_errno() << "\n";
                        shutdownApplication(i->second);
                    }
                    
                    int nBytes = *((uint32_t*)recvBuffer);
                    nBytes = recv(socket, recvBuffer, nBytes, 0);
                    if (nBytes==SOCKET_ERROR) {
                        ev << "ProtSimSocketRTScheduler: socket error " << sock_errno() << "\n";
                        shutdownApplication(i->second);
                    }
                    else if (nBytes == 0) {
                        ev << "ProtSimSocketRTScheduler: socket closed by the client\n";
                        shutdownApplication(i->second);
                    }
                    else {
                        // schedule notificationMsg for the interface module
                        ev << "ProtSimSocketRTScheduler: received " << nBytes << " bytes\n";
                        cMessage* msg = new cMessage("External", eEXTERNAL_MSG, nBytes*8);
                        DataChunk * data = new DataChunk(nBytes);
                        memcpy(data->data(), recvBuffer, nBytes);
                        msg->setControlInfo(data);

                        timeval curTime;
                        gettimeofday(&curTime, NULL);
                        curTime = timeval_substract(curTime, baseTime);
                        simtime_t t = curTime.tv_sec + curTime.tv_usec*1e-6;
                        // TBD assert that it's somehow not smaller than previous event's time
                        msg->setArrival(i->second->module,-1,t);
                        simulation.msgQueue.insert(msg);
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

int ProtSimSocketRTScheduler::receiveUntil(const timeval& targetTime)
{
    // if there's more than 200ms to wait, wait in 100ms chunks
    // in order to keep UI responsiveness by invoking ev.idle()
    timeval curTime;
    gettimeofday(&curTime, NULL);
    while (targetTime.tv_sec-curTime.tv_sec >=2 ||
           timeval_diff_usec(targetTime, curTime) >= 200000) {
        if (receiveWithTimeout(100000)) // 100ms
            return 1;
        if (ev.idle())
            return -1;
        gettimeofday(&curTime, NULL);
    }

    // difference is now at most 100ms, do it at once
    long usec = timeval_diff_usec(targetTime, curTime);
    if (usec>0)
        if (receiveWithTimeout(usec))
            return 1;
    return 0;
}

void ProtSimSocketRTScheduler::sendBytes(cModule * appl, DataChunk* data)
{
    if (applByModule.find(appl) == applByModule.end())
        throw new cRuntimeError("ProtSimSocketRTScheduler: sendBytes(): no connection");

    assert(appl);
    send(applByModule[appl]->socket, data->data(), data->size(), 0);
    // TBD check for errors
    delete data;
}


void ProtSimSocketRTScheduler::registerNode(NodeAddressT nodeAddr, NetworkStack* module) {
    nodes[nodeAddr] = module;
}

void ProtSimSocketRTScheduler::registerApplication(ApplicationDesc * appl)
{
    assert(appl);
    assert(appl->module);

    applBySocket[appl->socket] = appl;
    applByModule[appl->module] = appl;
}

void ProtSimSocketRTScheduler::deregisterApplication(ApplicationDesc * appl)
{
    assert(appl);

    applBySocket.erase(appl->socket);
    applByModule.erase(appl->module);
}

void ProtSimSocketRTScheduler::mapApplAddr(uint16_t port, NodeAddressT* nodeAddr,
                                           ApplicationAddressT* applAddr) {
    *applAddr = port % numApplPerNode;
    *nodeAddr = (port / numApplPerNode) % numNodes;
}

void ProtSimSocketRTScheduler::createApplication(ApplicationDesc* appl) {
    if (nodes.find(appl->nodeAddr) == nodes.end()) {
        delete appl;
        throw new cRuntimeError("ProtSimSocketRTScheduler: Node not found");
    }
    nodes[appl->nodeAddr]->createApplication(appl);
}

void ProtSimSocketRTScheduler::shutdownApplication(ApplicationDesc* appl, bool endOfRun) {
    nodes[appl->nodeAddr]->deleteApplication(appl, endOfRun);
    if (shutdown(appl->socket, SHUT_WR) == SOCKET_ERROR) {
        perror("shutdown");
        //throw new cRuntimeError("ProtSimSocketRTScheduler: shutdown() failed");
    }
    closesocket(appl->socket);
    deregisterApplication(appl);
    delete appl;
}
