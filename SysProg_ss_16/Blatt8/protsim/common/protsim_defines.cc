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

#include "protsim_defines.h"

const char * sysCallString[] = {
    "",
    "eLOCAL_LINK_CHANGE_NOTIFICATION",
    "eLINK_STATE_ADVERTISEMENT",
    "eDISTANCE_VECTOR_MESSAGE",
    "eLOCAL_PACKET_NOTIFICATION",
    "ePING",
    "ePONG",
    "eDATA",
    "eACK",
    "ePERM",
    "eSYN",
    "eFIN",
    "eRST",
    "eTIMEOUT",
    "eSYS_LISTEN",
    "eSYS_INCOMING_IND",
    "eSYS_OPEN",
    "eSYS_ESTABLISHED_IND",
    "eSYS_CLOSE",
    "eSYS_CLOSING_IND",
    "eSYS_SEND",
    "eSYS_SEND_READY_IND",
    "eSYS_RECEIVE",
    "eSYS_DATA_IND",
    "eSYS_ABORT_IND",
    "eSYS_CLOSED_IND"
};

const char ** packetIdString = sysCallString;

const char * sysCallResultString[] = {
    "eOKAY", "eINVALID_STATE", "eTRY_AGAIN", "eREJECT", "eABORTED"
};

