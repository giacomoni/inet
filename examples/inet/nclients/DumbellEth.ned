//
// Copyright (C) 2004 OpenSim Ltd.
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//





package inet.examples.inet.nclients;

import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.inet.Router;
import inet.node.inet.StandardHost;
import ned.DatarateChannel;
import inet.node.ethernet.EthernetSwitch;


network DumbellEth
{
    parameters:
        int n;

    types:
        channel NormalP extends DatarateChannel
        {
            parameters:
                per = 0;
                ber = 0;
        }

    submodules:
        configurator: Ipv4NetworkConfigurator {
            parameters:
                @display("p=100,100;is=s");
        }
        r1: EthernetSwitch {
            @display("p=400,250");
        }

        cli[n]: StandardHost {
            parameters:
                @display("i=device/laptop;p=250,100,col,100");
        }
        srv: StandardHost {
            parameters:
                @display("i=device/server;p=850,250");
        }
    connections:
        for i=0..n-1 {
            cli[i].ethg++ <--> NormalP <--> r1.ethg++;
        }
       
         srv.ethg++ <--> NormalP <--> r1.ethg++;
        
}

