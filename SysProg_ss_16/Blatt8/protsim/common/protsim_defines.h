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

#ifndef PROTSIM_DEFINES_H
#define PROTSIM_DEFINES_H

#include <omnetpp.h>

#undef OMNET30
#undef OMNET31
#undef OMNET32

#if OMNETPP_VERSION >= 0x0300
#define OMNET30
#endif

#if OMNETPP_VERSION < 0x0203
#error "At least OMNeT++ 2.3 required"
#endif

#if OMNETPP_VERSION > 0x0300
#define OMNET31
#endif

#if OMNETPP_VERSION > 0x0301
#define OMNET32
#endif

#if OMNETPP_VERSION > 0x0303
#warning "protsim is only tested with OMNeT++ 2.3, 3.0 to 3.3"
#endif

#define PS_DEBUG

#undef WINDOWS

#ifdef WIN32
typedef int ssize_t;
#define snprintf _snprintf
#define strncasecmp _strnicmp
#define strcasecmp _stricmp
#pragma warning (disable : 4786)
#endif

#ifdef OMNET32
#define DUP_RETVAL cPolymorphic
#else
#define DUP_RETVAL cObject
#endif

/// \name Node Addresses
//@{
typedef long NodeAddressT; ///< Type for node addresses

#define PS_NODE_UNKNOWN       -1 ///< Unknown node address
#define PS_DEST_BROADCAST_ALL -2 ///< Send to all neighbors
#define PS_DEST_BROADCAST_LIMIT -3 ///< Send to all neighbors except last hop 
//@}


/// \name Type Aliases
//@{
class ForwardingTable;
typedef ForwardingTable NeighborTable; ///< Neighbor table type
typedef ForwardingTable LocalApplTable; ///< Local application table type
//@}
 

/// Message kinds
enum MessageKind {
    eNETWORK_PACKET = 1,   ///< Network packet
    eLOCAL_MESSAGE  = 2,   ///< Local message
    eSYS_CALL       = 3,   ///< System call
    eEXTERNAL_MSG   = 4,   ///< Message from external application to proxy
    eTIMER0         = 1000,///< Timer message
    eTIMER1         = 1001,///< Timer message
    eTIMER2         = 1002,///< Timer message
    eTIMER3         = 1003 ///< Timer message
};


/// Message IDs
enum MessageID {
    eLOCAL_LINK_CHANGE_NOTIFICATION = 1,///< Local message when link cost changes
    eLINK_STATE_ADVERTISEMENT       = 2,///< Link state advertisement for LinkStateRoutingDaemon
    eDISTANCE_VECTOR_MESSAGE        = 3,///< Distance vector message for DistanceVectorRoutingDaemon
    eLOCAL_PACKET_NOTIFICATION      = 4,///< Local message notifying a HotPotatoRoutingDaemon of data packets

    ePING                           = 5,///< Used by PingPongAppl
    ePONG                           = 6,///< Used by PingPongAppl
    eECHO_REQUEST                   = ePING,///< Used by PingPongAppl
    eECHO_REPLY                     = ePONG,///< Used by PingPongAppl

    eDATA                           = 7, ///< Data packet
    eACK                            = 8, ///< Acknowledgement
    ePERM                           = 9, ///< Permit packet
    eSYN                           = 10, ///< connection establishment
    eFIN                           = 11, ///< connection release
    eRST                           = 12, ///< connection reset
    eTIMEOUT                       = 13 ///< timeout
};
extern const char ** packetIdString;


/// System Calls
enum SystemCall {
    eSYS_LISTEN                   = 14,
    eSYS_INCOMING_IND             = 15,
    eSYS_OPEN                     = 16,
    eSYS_ESTABLISHED_IND          = 17,
    eSYS_CLOSE                    = 18,
    eSYS_CLOSING_IND              = 19,
    eSYS_SEND                     = 20,
    eSYS_SEND_READY_IND           = 21,
    eSYS_RECEIVE                  = 22,
    eSYS_DATA_IND                 = 23,
    eSYS_ABORT_IND                = 24,
    eSYS_CLOSED_IND               = 25
};
extern const char * sysCallString[eSYS_CLOSED_IND+1];


/// System Call Results
enum SystemCallResult {
    eOKAY                         = 0,
    eINVALID_STATE                = 1,
    eTRY_AGAIN                    = 2,
    eREJECT                       = 3,
    eABORTED                      = 4
};
extern const char * sysCallResultString[eABORTED+1];


/// \name Application Codes
//@{
typedef long ApplicationAddressT; ///< Application address type

#define PS_DUMMY_APP             -1 ///< Dummy application
#define PS_APP_UNKNOWN           PS_DUMMY_APP ///< Unknown application
/** Routing daemon. Since only one routing daemon may run on a node, all
 *  daemons share the same application address. */
#define PS_ROUTING_DAEMON        -2
#define PS_USER_APP_MIN           0 ///< Minimum value for user applications.
//@}


#endif /* PROTSIM_DEFINES_H */
