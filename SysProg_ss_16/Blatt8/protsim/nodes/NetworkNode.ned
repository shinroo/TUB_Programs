// *************************************************************************
//
// This file is part of the ProtSim framework developed by TKN for a
// practical course on basics of simulation and Internet protocol functions
//
// Copyright:   (C)2004-2007 Telecommunication Networks Group (TKN) at
//              Technische Universitaet Berlin, Germany.
//
// Authors:     Lars Westerhoff, Guenter Schaefer
//
// *************************************************************************

import "NetworkStack.ned";
import "Application.ned";
import "../support/InterfaceQueue.ned";

module NetworkNode
    parameters:
	nodeAddr: numeric,

	applicationType0: string,
	applicationType1: string,
	applicationType2: string,
	applicationType3: string,
	applicationType4: string;

    gates:
	in: netIn[];
	out: netOut[];

    submodules:
	networkStack: NetworkStack
	    parameters:
	    gatesizes:
	        netIn[sizeof(netIn)],
		netOut[sizeof(netOut)],
		fromAppl[5],
		toAppl[5];
	interfaceQueue: InterfaceQueue[sizeof(netOut)];

	application0: applicationType0 like Application;
	application1: applicationType1 like Application;
	application2: applicationType2 like Application;
	application3: applicationType3 like Application;
	application4: applicationType4 like Application;

    connections:
	// Connect network interfaces
	for i=0..sizeof(netIn)-1 do
	    netIn[i] --> networkStack.netIn[i];
	endfor;
	for i=0..sizeof(netOut)-1 do
	    netOut[i] <-- interfaceQueue[i].out;
	    interfaceQueue[i].in <-- networkStack.netOut[i];
	endfor;

	// Connect applications
	networkStack.toAppl[0] --> application0.in;
	networkStack.fromAppl[0] <-- application0.out;

	networkStack.toAppl[1] --> application1.in;
	networkStack.fromAppl[1] <-- application1.out;

	networkStack.toAppl[2] --> application2.in;
	networkStack.fromAppl[2] <-- application2.out;

	networkStack.toAppl[3] --> application3.in;
	networkStack.fromAppl[3] <-- application3.out;

	networkStack.toAppl[4] --> application4.in;
	networkStack.fromAppl[4] <-- application4.out;
endmodule NetworkNode;
