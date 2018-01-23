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

import  
    "../nodes/NetworkNode.ned"; 
                                
 
module ARQNetworkModule 
    parameters: 
        routing : string, 
        maxTTL : numeric, 
        N : numeric, 
        localDelay : numeric, 
        localDatarate : numeric, 
        localError : numeric, 
        wideAreaDelay : numeric, 
        wideAreaDatarate : numeric, 
        wideAreaError : numeric,
        leftNodeAddrOffset : numeric,
        rightNodeAddrOffset : numeric;

    submodules: 
        LeftNode: NetworkNode[N]; 
            parameters: 
                nodeAddr = leftNodeAddrOffset+index, 
                                       
                applicationType0 = routing, 
                applicationType1 = "DummyApplication",
                applicationType2 = "DummyApplication",
                applicationType3 = "DummyApplication", 
                applicationType4 = "DummyApplication"; 
            gatesizes: 
                netIn[1], 
                netOut[1]; 
                           
            display: "p=60,60,column;b=40,24";
        LeftRouter: NetworkNode; 
            parameters: 
                nodeAddr = 1000, 
                                 
                applicationType0 = routing, 
                applicationType1 = "DummyApplication", 
                applicationType2 = "DummyApplication", 
                applicationType3 = "DummyApplication", 
                applicationType4 = "DummyApplication"; 
                                                       
            gatesizes: 
                netIn[N+1], 
                netOut[N+1]; 
                             
            display: "p=200,260;b=40,24";
        RightRouter: NetworkNode; 
            parameters: 
                nodeAddr = 2000, 
                                 
                applicationType0 = routing, 
                applicationType1 = "DummyApplication", 
                applicationType2 = "DummyApplication", 
                applicationType3 = "DummyApplication", 
                applicationType4 = "DummyApplication"; 
                                                       
            gatesizes: 
                netIn[N+1], 
                netOut[N+1]; 
                             
            display: "p=360,260;b=40,24";
        RightNode: NetworkNode[N]; 
            parameters: 
                nodeAddr = rightNodeAddrOffset+index, 
                                       
                applicationType0 = routing, 
                applicationType1 = "DummyApplication",
                applicationType2 = "DummyApplication",
                applicationType3 = "DummyApplication", 
                applicationType4 = "DummyApplication"; 
                                                       
            gatesizes: 
                netIn[1], 
                netOut[1]; 
                           
            display: "p=540,60,column;b=40,24";
    connections: 
        for i=1..N do 
            LeftNode[i-1].netOut[0] --> delay localDelay datarate localDatarate error localError --> LeftRouter.netIn[i]; 
            LeftNode[i-1].netIn[0] <-- delay localDelay datarate localDatarate error localError <-- LeftRouter.netOut[i]; 
                                                                                                                          
            RightNode[i-1].netOut[0] --> delay localDelay datarate localDatarate error localError --> RightRouter.netIn[i]; 
            RightNode[i-1].netIn[0] <-- delay localDelay datarate localDatarate error localError <-- RightRouter.netOut[i]; 
        endfor; 
        LeftRouter.netOut[0] --> delay wideAreaDelay datarate wideAreaDatarate error wideAreaError --> RightRouter.netIn[0]; 
        LeftRouter.netIn[0] <-- delay wideAreaDelay datarate wideAreaDatarate error wideAreaError <-- RightRouter.netOut[0]; 
    display: "p=-126,2;b=706,530";
endmodule 


network ARQNetwork : ARQNetworkModule 
    parameters:
        maxTTL = 16; 
endnetwork
