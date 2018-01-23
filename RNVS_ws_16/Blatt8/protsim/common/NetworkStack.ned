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

simple NetworkStack
    parameters:
	metric: string,
        dumpRoutes: numeric,
	externalApplType: string,
        externalGenerateTransport: numeric,
	callFinishExternalAppl: numeric,
	shell: string,
	shell_result: string;

    gates:
	in: netIn[], fromAppl[];
	out: netOut[], toAppl[];
endsimple
