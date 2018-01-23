//=========================================================================
//  CSOCKETRTSCHEDULER.H - part of
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

#ifndef PROTSIM_SOCKET_RT_SCHEDULER_H_
#define PROTSIM_SOCKET_RT_SCHEDULER_H_

#include <map>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "platdep/time.h"
#include "platdep/sockets.h"
#include <omnetpp.h>
#include "protsim_defines.h"

class NetworkStack;
class ExternalApplication;


/**
 * Real-time scheduler with socket-based external communication.
 *
 * \code
 * class MyInterfaceModule : public cSimpleModule
 * {
 *    ProtSimSocketRTScheduler *rtScheduler;
 *    cMessage *extEvent;
 *    char buf[4000];
 *    int numBytes;
 *    ...
 * \endcode
 *
 * \code
 * void MyInterfaceModule::initialize()
 * {
 *     extEvent = new cMessage("extEvent");
 *     rtScheduler = check_and_cast<ProtSimSocketRTScheduler *>(simulation.scheduler());
 *     rtScheduler->setInterfaceModule(this, extEvent, buf, 4000, numBytes);
 * }
 * \endcode
 *
 * THIS CLASS IS JUST AN EXAMPLE -- IF YOU WANT TO DO HARDWARE-IN-THE-LOOP
 * SIMULATION, YOU WILL NEED TO WRITE YOUR OWN SCHEDULER WHICH FITS YOUR NEEDS.
 * For example, you'll probably want a different external interface than
 * a single TCP socket: maybe UDP socket, maybe raw socket to grab full Ethernet
 * frames, maybe pipe, maybe USB or other interface, etc.
 */
class ProtSimSocketRTScheduler : public cScheduler
{
public:
    struct ApplicationDesc {
        ExternalApplication * module;
        NodeAddressT        nodeAddr;
        ApplicationAddressT applAddr;
        SOCKET              socket;
        uint32_t            ipAddr;
        uint16_t            port;
    };

    class DataChunk : public cPolymorphic {
    public:
        DataChunk() : m_size(0), m_data(NULL) { }

        DataChunk(size_t size_) : m_size(size_), m_data(new char[size_]) { }

        void reset(size_t size_ = 0) {
            delete [] m_data;
            m_data = NULL;
            m_size = size_;
            if (m_size != 0) m_data = new char[m_size];
        }

        char * const data() const { return m_data; }
        size_t size() const { return m_size; }

        char& operator[](size_t index) { return m_data[index]; }

        virtual ~DataChunk() {
            delete [] m_data;
        }

    private:
        size_t m_size;
        char * m_data;
    };

public:
    /**
     * Constructor.
     */
    ProtSimSocketRTScheduler();

    /**
     * Destructor.
     */
    virtual ~ProtSimSocketRTScheduler();

    /**
     * Called at the beginning of a simulation run.
     */
    virtual void startRun();

    /**
     * Called at the end of a simulation run.
     */
    virtual void endRun();

    /**
     * Recalculates "base time" from current wall clock time.
     */
    virtual void executionResumed();

    /**
     * Scheduler function -- it comes from cScheduler interface.
     */
    virtual cMessage *getNextEvent();


    /**
     * To be called from the module which wishes to receive data from the
     * socket. The method must be called from the module's initialize()
     * function.
     */
    virtual void registerNode(NodeAddressT nodeAddr, NetworkStack* module);


    /**
     * Send on the currently open connection
     */
    virtual void sendBytes(cModule * appl, DataChunk* data);

protected:
    class CompareById {
    public:
        bool operator()(cModule* a, cModule* b) {if (!a || !b) return b; return a->id() < b->id(); }
    };

    //typedef std::vector<ApplicationDesc> ApplicationContainer;
    typedef std::map<SOCKET,ApplicationDesc*> ApplBySocket;
    typedef std::map<cModule*,ApplicationDesc*,CompareById> ApplByModule;

    typedef std::map<NodeAddressT,NetworkStack*> NodeMap;

protected:
    static const unsigned MAX_RECEIVE_SIZE = 65536;

    //ApplicationContainer applications;
    ApplBySocket applBySocket;
    ApplByModule applByModule;
    NodeMap      nodes;

    // config
    int port;
    int numApplPerNode;
    int numNodes;

    char recvBuffer[MAX_RECEIVE_SIZE];

    // state
    timeval baseTime;
    SOCKET listenerSocket;

    virtual void setupListener();
    virtual void acceptConnection();
    virtual void fillFdSet(fd_set* set);
    virtual bool receiveWithTimeout(long usec);
    virtual int receiveUntil(const timeval& targetTime);

    void registerApplication(ApplicationDesc * appl);
    void deregisterApplication(ApplicationDesc * appl);
    virtual void mapApplAddr(uint16_t port, NodeAddressT* nodeAddr, ApplicationAddressT* applAddr);
    virtual void createApplication(ApplicationDesc* appl);
    virtual void shutdownApplication(ApplicationDesc* appl, bool endOfRun = false);
};

#endif
