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

#ifndef NETWORK_STACK_H
#define NETWORK_STACK_H

#include <omnetpp.h>
#include "protsim_defines.h"
#include "../support/ForwardingTable.h"
#include "../support/MetricFunc.h"
#include "../messages/NetworkPacket_m.h"
#include "../emulation/ProtSimSocketRTScheduler.h"

/**
 * Represents the network stack within a NetworkNode. Maintains tables of neighboring nodes,
 * local applications and the forwarding table. The forwarding table is empty per default and
 * must be configured by the local routing daemon. The NetworkStack forwards packets according
 * to the forwarding table and the local applications table.
 */

std::istream& operator>>(std::istream& is, cPar& p);

class NetworkStack : public cSimpleModule {
public:
    /// Constructor called by OMNeT++ framework.
    NetworkStack(const char *name = NULL, cModule *parentmod = NULL)
	: cSimpleModule(name, parentmod, 0), virtualQueues(NULL), costs(NULL),
	  metricFunc(NULL), routingTableDump("routingTableDump",2) {}

    /// Destructor
    virtual ~NetworkStack();

    virtual NodeAddressT getNodeAddr() const; ///< \return Node address of NetworkNode.

    /// \name Forwarding Table
    //@{
    virtual void clearRoutes(); ///< Clear the forwarding table.

    /**
     * Add a route.
     *
     * @param dest Destination node address.
     * @param outputgate Gate to the next hop.
     */
    virtual void addRoute(NodeAddressT dest, cGate * outputgate);

    /**
     * Add a route. \overload
     *
     * @param dest Destination node address.
     * @param nextHop Next hop node address.
     */
    virtual void addRoute(NodeAddressT dest, NodeAddressT nextHop);

    /**
     * Removes a route.
     *
     * @param dest Destination node address.
     */
    virtual void removeRoute(NodeAddressT dest);

    /**
     * Lookup a route.
     *
     * @param dest Destination node address.
     * @return Gate to next hop.
     */
    virtual cGate * lookupRoute(NodeAddressT dest) const;
    //@}

    /// \name Coversion between internal and external gates
    //@{
    cGate * getExternalGate(cGate * outgate) const;
    cBasicChannel * getExternalLink(cGate * outgate) const;
    cGate * getInternalGate(cGate * outgate);
    //@}

    /// \name Neighbors and Links to Neighbors
    //@{

    /// \return if node with address dest is a neighbor
    virtual bool isNeighbor(NodeAddressT dest) const;

    /**
     * Lookup the neighbor by link.
     *
     * @param outgate Gate to external link.
     * @return Node address of neighbor on the given link.
     */
    virtual NodeAddressT lookupNeighbor(cGate * outgate) const;

    /**
     * Lookup a gate to a neighbor by neighbor address.
     *
     * @param peer Node address of neighbor.
     * @return Gate to given neighbor.
     */
    virtual cGate * lookupNeighbor(NodeAddressT peer) const;

    virtual int getNumNeighbors() const; ///< \return Number of neighbors.

    /**
     * Lookup a gate to a neighbor by link index
     *
     * @param index Index of neighbor (0..numNeighbors-1).
     * @return Gate to neighbor.
     */
    virtual const cGate * lookupNeighborAt(int index) const;

    /**
     * Lookup a gate to a neighbor by link index
     *
     * @param index Index of neighbor (0..numNeighbors-1).
     * @return Gate to neighbor.
     */
    virtual cGate * lookupNeighborAt(int index);

    /**
     * Propagation delay to neighbor.
     *
     * @param outgate Gate to external link.
     * @return Propagation delay of link in seconds.
     */
    virtual double getDelayToNeighbor(cGate * outgate) const;
    /**
     * Propagation delay to neighbor. \overload
     *
     * @param peer Neighbor node address.
     * @return Propagation delay of link in seconds.
     */
    virtual double getDelayToNeighbor(NodeAddressT peer) const;

    /**
     * Datarate to neighbor.
     *
     * @param outgate Gate to external link.
     * @return Datarate of link in bps.
     */
    virtual double getDatarateToNeighbor(cGate * outgate) const;
    /**
     * Datarate to neighbor. \overload
     *
     * @param peer Neighbor node address.
     * @return Datarate of link in bps.
     */
    virtual double getDatarateToNeighbor(NodeAddressT peer) const;

    /**
     * Dynamic delay to neighbor.
     * This is the propagation delay plus current waiting time.
     *
     * @param outgate Gate to external link.
     * @return Dynamic delay of link in seconds.
     */
    virtual double getDynamicDelayToNeighbor(cGate * outgate) const;
    /**
     * Dynamic delay to neighbor. \overload
     * This is the propagation delay plus current waiting time.
     *
     * @param peer Neighbor node address.
     * @return Dynamic delay of link in seconds.
     */
    virtual double getDynamicDelayToNeighbor(NodeAddressT peer) const;

    /**
     * Virtual queue length to neighbor of the implicit queue of OMNeT++.
     * This is the number of bits that were already issued by a send(),
     * but due to the limited bandwidth of the connection have not left the
     * network node, yet.
     *
     * @param outgate Gate to external link.
     * @return Queue length of link in bits.
     */
    virtual double getVirtualQueue(cGate * outgate) const;
    /**
     * Virtual queue length to neighbor. \overload
     *
     * @param peer Neighbor node address.
     * @return Queue length of link in bits.
     */
    virtual double getVirtualQueue(NodeAddressT peer) const;

    /**
     * Cost to neighbor.
     *
     * @param outgate Gate to external link.
     * @return Cost of link.
     */
    virtual double getMetric(cGate * outgate) const;
    /**
     * Cost to neighbor. \overload
     *
     * @param peer Neighbor node address.
     * @return Cost of link.
     */
    virtual double getMetric(NodeAddressT peer) const;
    //@}

    /// \name Application Table
    //@{
    virtual cGate * lookupApplication(ApplicationAddressT dest) const;
    //@}

    /// \name Misc. Properties
    //@{

    /// Set function set determines the metric (cost) of external links.
    virtual void setMetricFunc(const char * funcName);
    /// Get name of current metric function.
    virtual const char * getMetricFunc() const;
    //@}

    //@{
    virtual void createApplication(ProtSimSocketRTScheduler::ApplicationDesc* appl);
    virtual void deleteApplication(ProtSimSocketRTScheduler::ApplicationDesc* appl, bool endRun = false);
    //@}

protected:
    /// \name Methods from OMNeT++ framework
    //@{
    virtual int numInitStages() const;
    virtual void initialize(int stage);
    virtual void finish();
    virtual void handleMessage(cMessage * msg);
    virtual void handleParameterChange(const char * parname);
    //@}

    /**
     * Simple shell parser. Used to set parameters that can not be accessed
     * by the GUI.
     *
     * The following commands are defined:
     * \li <TT>conpar %par link_index value</TT>: Set parameter \c par
     * of connection with index \c link_index to \c value.
     *
     * @param command Command to be executed.
     * @param result Parameter that receives the result string.
     */
    virtual void shell(const char * command, cPar * result = NULL);

    /// Deliver packet to a local application.
    virtual void deliverLocally(NetworkPacket * packet);

    /// Forward packet to an external link.
    virtual void sendOut(NetworkPacket * packet);

    /**
     * Send a packet notification to the local routing daemon.
     *
     * @param packet Packet for which the notification is sent.
     *               A copy will be embedded within the notification.
     * @param noroute No route to destination in the forwarding table.
     */
    virtual void sendLocalPacketNotif(NetworkPacket * packet, bool noroute = false);

private:
    ForwardingTable   routes; ///< Forwarding table.
    NeighborTable  neighbors; ///< Table of neighbors (auto-detected).
    LocalApplTable applications; ///< Table of local applications (auto-detected).

    NodeAddressT nodeAddr; ///< Address of local node.

    double * virtualQueues; ///< Array of (virtual) queue length for external links.
    double * costs;  ///< Array of costs of external links (for watches).
    MetricFunc * metricFunc; ///< Function for determining link costs.

    bool dumpRoutes;
    cOutVector routingTableDump; ///< Vector for tracing changes in routes

    /** Maximum TTL before packet is dropped (read from system module parameter
     *  \c maxTTL).
     */
    long     maxTTL;
    std::string externalApplType;
    bool externalGenerateTransport;
    bool callFinishExternalAppl;
};

#endif /* NETWORK_STACK_H */
