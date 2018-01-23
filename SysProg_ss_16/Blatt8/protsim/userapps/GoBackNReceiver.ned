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

simple GoBackNReceiver
    parameters:
	applAddr: numeric,
	ackSize: numeric,
	processing: numeric,
	receiveQueue: numeric,
	sendNack: numeric;

    gates:
	in: in;
	out: out;
endsimple;
