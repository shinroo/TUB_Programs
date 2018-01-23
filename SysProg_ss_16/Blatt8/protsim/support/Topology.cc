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
#include <iterator>
#include "Topology.h"
#include "../common/NetworkStack.h"

Topology::Topology(cTopology * topo) {
    for (int i = 0; i < topo->nodes(); ++i) {
	cTopology::Node * oldNode = topo->node(i);
	NetworkStack * src = dynamic_cast<NetworkStack*>(oldNode->module()->submodule("networkStack"));
	assert(src);
	NodeAddressT src_addr = src->getNodeAddr();
	TopologyNode * newNode = addNodeFor(src_addr);
	for (int j = 0; j < oldNode->outLinks(); ++j) {
	    cTopology::LinkOut * oldLink = oldNode->out(j);
	    NetworkStack * dest = dynamic_cast<NetworkStack*>(oldLink->remoteNode()->module()->submodule("networkStack"));
	    NodeAddressT dest_addr = dest->getNodeAddr();
	    newNode->addLinkTo(addNodeFor(dest_addr),oldLink->weight());
	}
    }
}

Topology::Topology(const Topology& topo) {
    copy(topo);
}

const Topology& Topology::operator= (const Topology& topo) {
    if (this == &topo) return *this;

    // cleanup old stuff
    clear();

    // and copy
    copy(topo);

    return *this;
}

void Topology::copy(const Topology& topo) {
    for (NodeMapT::const_iterator i = topo.nodes.begin();
	 i != topo.nodes.end(); ++i) {
	TopologyNode * newNode = addNodeFor(i->first);
	TopologyNode * oldNode = i->second;
	for (int j = 0; j < oldNode->getNumLinks(); ++j) {
	    TopologyLink * oldLink = oldNode->getLink(j);
	    NodeAddressT dest_addr = oldLink->getDestNode()->getNodeAddr();
	    newNode->addLinkTo(addNodeFor(dest_addr),oldLink->getWeight());
	}
    }
}

int Topology::getNumNodes() const {
    return nodes.size();
}

const TopologyNode * Topology::getNode(int index) const {
    NodeMapT::const_iterator i = nodes.begin();
    std::advance(i,index);
    return i->second;
}


const TopologyNode * Topology::getNodeByAddr(NodeAddressT addr) const {
    NodeMapT::const_iterator i = nodes.find(addr);
    if (i == nodes.end()) return NULL;
    else return i->second;
}

TopologyNode * Topology::getNode(int index) {
    NodeMapT::iterator i = nodes.begin();
    std::advance(i,index);
    return i->second;
}

TopologyNode * Topology::getNodeByAddr(NodeAddressT addr) {
    NodeMapT::iterator i = nodes.find(addr);
    if (i == nodes.end()) return NULL;
    else return i->second;
}

Topology::NodeSet Topology::getNodes() {
    NodeSet nodes_;
    for (NodeMapT::iterator i = nodes.begin(); i != nodes.end(); ++i) {
	nodes_.insert(i->second);
    }
    return nodes_;
}

// void Topology::addNode(TopologyNode * node) {
//     nodes[node->getNodeAddr()] = node;
// }

TopologyNode * Topology::addNodeFor(NodeAddressT addr) {
    TopologyNode * newNode = nodes[addr];
    if (newNode == NULL) {
	nodes[addr] = newNode = new TopologyNode(addr);
    }
    return newNode;
}

void Topology::clear() {
    for (NodeMapT::iterator i = nodes.begin(); i != nodes.end(); ++i) {
	delete i->second;
    }
    nodes.clear();
}

Topology::~Topology() {
    for (NodeMapT::iterator i = nodes.begin(); i != nodes.end(); ++i) {
	delete i->second;
    }
    // no need to set pointers to NULL, nodes is destroyed here anyway
}



TopologyNode::TopologyNode(NodeAddressT addr_): addr(addr_) { }

TopologyNode::~TopologyNode() {
    for (LinkMapT::iterator i = links.begin(); i != links.end(); ++i) {
	delete i->second;
    }
    // no need to set pointers to NULL, links is destroyed here anyway
}

NodeAddressT TopologyNode::getNodeAddr() const {
    return addr;
}

int TopologyNode::getNumLinks() const {
    return links.size();
}

const TopologyLink * TopologyNode::getLink(int index) const {
    LinkMapT::const_iterator i = links.begin();
    std::advance(i,index);
    return i->second;
}

const TopologyLink * TopologyNode::getLinkTo(NodeAddressT addr) const {
   LinkMapT::const_iterator i = links.find(addr);
   if (i == links.end()) return NULL;
   else return i->second;
}

TopologyLink * TopologyNode::getLink(int index) {
    LinkMapT::iterator i = links.begin();
    std::advance(i,index);
    return i->second;
}

TopologyLink * TopologyNode::getLinkTo(NodeAddressT addr) {
    LinkMapT::iterator i = links.find(addr);
    if (i == links.end()) return NULL;
    else return i->second;
}

Topology::LinkSet TopologyNode::getLinks() {
    Topology::LinkSet links_;
    for (LinkMapT::iterator i = links.begin(); i != links.end(); ++i) {
	links_.insert(i->second);
    }
    return links_;
}

TopologyLink * TopologyNode::addLinkTo(TopologyNode * node, double weight) {
    TopologyLink * newLink = links[node->getNodeAddr()];
    if (newLink == NULL) {
	links[node->getNodeAddr()] = newLink =
	    new TopologyLink(weight, this, node);
    }
    else {
	newLink->setWeight(weight);
    }
    return newLink;
}



TopologyLink::TopologyLink(double weight_, TopologyNode * srcNode_, TopologyNode * destNode_)
    : weight(weight_), srcNode(srcNode_), destNode(destNode_) { }

const TopologyNode * TopologyLink::getSrcNode() const {
    return srcNode;
}

const TopologyNode * TopologyLink::getDestNode() const {
    return destNode;
}

TopologyNode * TopologyLink::getSrcNode() {
    return srcNode;
}

TopologyNode * TopologyLink::getDestNode() {
    return destNode;
}

double TopologyLink::getWeight() const {
    return weight;
}

void TopologyLink::setWeight(double weight_) {
    weight = weight_;
}

