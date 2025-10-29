package src;

channel Link extends ned.DatarateChannel
{
    datarate = 100Mbps;
    delay = 1ms;
}

simple RouteController
{
    parameters:
        int numNodes = default(10);
        @display("i=block/control");
    gates:
        inout ppp;
}

simple SDNNode
{
    parameters:
        int address;
        @display("i=device/router");
    gates:
        inout port[];
}

simple TrafficGenerator
{
    parameters:
        int address;
        int destAddr;
        int ttl = default(20);
        string trafficType = default("video");
        double bandwidth @unit(Mbps) = default(10Mbps);
        double startTime @unit(s) = default(1s);
        double sendInterval @unit(s) = default(0.5s);
        int numPackets = default(5);
        @display("i=device/pc");
    gates:
        inout ppp;
}

network SDNNetwork
{
    parameters:
        @display("bgb=1200,800");

    submodules:
        controller: RouteController {
            @display("p=600,50");
        }

        videoSrc: TrafficGenerator {
            address = 0;
            destAddr = 9;
            trafficType = "video";
            startTime = 1s;
            sendInterval = 0.3s;
            numPackets = 10;
            @display("p=100,150");
        }

        datacenterSrc: TrafficGenerator {
            address = 1;
            destAddr = 8;
            trafficType = "datacenter";
            startTime = 1.2s;
            sendInterval = 0.5s;
            numPackets = 8;
            @display("p=100,300");
        }

        gamingSrc: TrafficGenerator {
            address = 3;
            destAddr = 7;
            trafficType = "gaming";
            startTime = 1.5s;
            sendInterval = 0.4s;
            numPackets = 12;
            @display("p=100,450");
        }

        iotSrc: TrafficGenerator {
            address = 6;
            destAddr = 5;
            trafficType = "iot";
            startTime = 2s;
            sendInterval = 1s;
            numPackets = 5;
            @display("p=100,600");
        }

        node[10]: SDNNode {
            @display("p=300,200,m,5,150,100");
        }

    connections:
        // Control plane - controller connected to central router
        controller.ppp <--> Link <--> node[0].port++;

        // Traffic generators connected to their respective nodes
        videoSrc.ppp <--> Link <--> node[0].port++;
        datacenterSrc.ppp <--> Link <--> node[1].port++;
        gamingSrc.ppp <--> Link <--> node[3].port++;
        iotSrc.ppp <--> Link <--> node[6].port++;

        // Data plane topology (matching buildTopology() in RouteController.cc)
        node[0].port++ <--> Link <--> node[1].port++;  // 0-1
        node[0].port++ <--> Link <--> node[2].port++;  // 0-2
        node[1].port++ <--> Link <--> node[3].port++;  // 1-3
        node[1].port++ <--> Link <--> node[4].port++;  // 1-4
        node[2].port++ <--> Link <--> node[4].port++;  // 2-4
        node[3].port++ <--> Link <--> node[5].port++;  // 3-5
        node[4].port++ <--> Link <--> node[5].port++;  // 4-5
        node[4].port++ <--> Link <--> node[6].port++;  // 4-6
        node[5].port++ <--> Link <--> node[7].port++;  // 5-7
        node[6].port++ <--> Link <--> node[7].port++;  // 6-7
        node[6].port++ <--> Link <--> node[8].port++;  // 6-8
        node[7].port++ <--> Link <--> node[9].port++;  // 7-9
        node[8].port++ <--> Link <--> node[9].port++;  // 8-9
}