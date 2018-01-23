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

#ifndef TOPOLOGY_H
#define TOPOLOGY_H

#include <set>
#include <map>
#include <omnetpp.h>
#include "../common/protsim_defines.h"

class TopologyNode;
class TopologyLink;

class CompareTopologyNode {
public:
    bool operator()(const TopologyNode * n1, const TopologyNode * n2) const;
};

class CompareTopologyLink {
public:
    bool operator()(const TopologyLink * l1, const TopologyLink * l2) const;
};

/**
 * Class representing a network topology. Unless the OMNeT++ cTopology class
 * this topology may be constructed from scratch.
 */
class Topology {
public:
    typedef std::set<TopologyNode *,CompareTopologyNode> NodeSet; ///< Type for a set of nodes.
    typedef std::set<TopologyLink *,CompareTopologyLink> LinkSet; ///< Type for a set of links.

public:
    /// Default constructor. Creates an empty topology.
    Topology() { }

    /// Create a topology from an OMNeT++ cTopology object.
    explicit Topology(cTopology * topo);

    Topology(const Topology& topo);
    const Topology& operator= (const Topology& topo);

    int getNumNodes() const; ///< \return Number of nodes.

    /// \return node number \c index
    const TopologyNode * getNode(int index) const;
    /// \return node with address \c addr
    const TopologyNode * getNodeByAddr(NodeAddressT addr) const;
    
    /// \return node number \c index
    TopologyNode * getNode(int index);
    /// \return node with address \c addr
    TopologyNode * getNodeByAddr(NodeAddressT addr);

    NodeSet getNodes(); ///< \return set of nodes

    /** 
     * Creates a node unless it already exists.
     * 
     * @param addr Node address of new node.
     * @return New or already existing node.
     */
    TopologyNode * addNodeFor(NodeAddressT addr);

    /// Clear topology.
    void clear();

    ~Topology();

private:
    void copy(const Topology& topo); ///< Copy topology (used by copy constr. and assignment

private:
    /// Type of node map.
    typedef std::map<NodeAddressT,TopologyNode*> NodeMapT;
    NodeMapT nodes; ///< Node map (indexed by node address).
};

/**
 * Node class for Topology.
 */
class TopologyNode {
public:
    /// Create node with given address
    explicit TopologyNode(NodeAddressT addr_);

    NodeAddressT getNodeAddr() const; ///< \return Node address
    int getNumLinks() const; ///< \return Number of output links

    const TopologyLink * getLink(int index) const; ///< \return Link number \c index
    /// \return Link to node \c addr
    const TopologyLink * getLinkTo(NodeAddressT addr) const;

    TopologyLink * getLink(int index); ///< \return Link number \c index
    TopologyLink * getLinkTo(NodeAddressT addr); ///< \return Link to node \c addr

    Topology::LinkSet getLinks(); ///< \return Set of links

    /** 
     * Create a link, unless it already exists. If the link already exists its
     * metric will be changed.
     *
     * @param node Peer node address
     * @param weight Link metric (cost)
     * @return Created or existing link.
     */
    TopologyLink * addLinkTo(TopologyNode * node, double weight = 1.0);

    ~TopologyNode();

private:
    /// Link map type
    typedef std::map<NodeAddressT,TopologyLink*> LinkMapT;

    NodeAddressT addr; ///< Node address
    LinkMapT  links; ///< Map of links (indexed by peer node address)

    TopologyNode(const TopologyNode&);
    void operator= (const TopologyNode&);
};

/**
 * Link class for Topology.
 */
class TopologyLink {
public:
    /** 
     * Create a new link.
     * 
     * @param weight_ Metric (cost) of link
     * @param srcNode_ Source node address
     * @param destNode_ Destination node address
     */
    explicit TopologyLink(double weight_ = 1.0,
		  TopologyNode * srcNode_ = NULL, TopologyNode * destNode_ = NULL);

    const TopologyNode * getSrcNode() const; ///< \return Source node of link
    const TopologyNode * getDestNode() const; ///< \return Destination node of link
    TopologyNode * getSrcNode(); ///< \return Source node of link 
    TopologyNode * getDestNode(); ///< \return Destination node of link
    double getWeight() const; ///< \return Metric (cost) of link
    void setWeight(double weight_); ///< Set link metric (cost)

private:
    double weight; ///< Link metric (cost)
    TopologyNode * srcNode; ///< Source node
    TopologyNode * destNode; ///< Destination node

    TopologyLink(const TopologyLink&);
    void operator= (const TopologyLink&);
};

inline bool CompareTopologyNode::operator()(const TopologyNode * n1, const TopologyNode * n2) const {
    return n1->getNodeAddr() < n2->getNodeAddr();
}

inline bool CompareTopologyLink::operator()(const TopologyLink * l1, const TopologyLink * l2) const {
    return 
        (l1->getSrcNode()->getNodeAddr() == l2->getSrcNode()->getNodeAddr()) ?
        (l1->getDestNode()->getNodeAddr() < l2->getDestNode()->getNodeAddr()) :
        (l1->getSrcNode()->getNodeAddr() < l2->getSrcNode()->getNodeAddr());
}


#endif /* TOPOLOGY_H */
