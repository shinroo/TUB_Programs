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

simple Application
    parameters:
	applAddr: numeric;

    gates:
	in: in;
	out: out;
endsimple;

simple DummyApplication
    parameters:
	applAddr: numeric;

    gates:
	in: in;
	out: out;
endsimple;

simple ExternalApplication
    parameters:
	applAddr: numeric,
	ipAddr: numeric,
	port: numeric;

    gates:
	in: in;
	out: out;
endsimple;
