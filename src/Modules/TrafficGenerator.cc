#include <omnetpp.h>
#include "helpers.h"
using namespace omnetpp;
using namespace std;

class TrafficGenerator : public cSimpleModule {
private:
    int addr;
    int dstAddr;
    TrafficType trafficType;
    int priority;
    cMessage *sendEvent;
    int packetsSent;

protected:
    void initialize() override {
        addr = par("address");
        dstAddr = par("destAddr");
        
        string type = par("trafficType").stdstringValue();
        if (type == "video") { trafficType = VIDEO; priority = 1; }
        else if (type == "datacenter") { trafficType = DATACENTER; priority = 1; }
        else if (type == "gaming") { trafficType = GAMING; priority = 2; }
        else { trafficType = IOT; priority = 3; }
        
        packetsSent = 0;
        sendEvent = new cMessage("sendTimer");
        scheduleAt(simTime() + par("startTime").doubleValue(), sendEvent);
    }

    void handleMessage(cMessage *msg) override {
        if (msg->isSelfMessage()) {
            // Send packet to controller
            QoSParams qos;
            qos.type = trafficType;
            qos.priority = priority;
                qos.bandwidth = par("bandwidth").doubleValue();
                qos.ttl = par("ttl").intValue();
            qos.delay = 0;
            
            auto *pkt = mkData("DataPacket", addr, dstAddr, qos);
            pkt->addPar("seqno").setLongValue(++packetsSent);
            
            EV_INFO << "Node " << addr << " sending " 
                    << (trafficType==VIDEO?"video":trafficType==DATACENTER?"datacenter":
                        trafficType==GAMING?"gaming":"iot")
                    << " packet #" << packetsSent << " to " << dstAddr 
                    << " (priority=" << priority << ")\n";
            
            // send via the declared 'ppp' gate in Network.ned
            send(pkt, "ppp$o");
            
            // Schedule next packet
            if (packetsSent < par("numPackets").intValue()) {
                scheduleAt(simTime() + par("sendInterval").doubleValue(), sendEvent);
            }
        } else {
            delete msg;
        }
    }

    void finish() override {
        cancelAndDelete(sendEvent);
    }
};

Define_Module(TrafficGenerator);