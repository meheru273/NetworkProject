#include <omnetpp.h>
#include "helpers.h"
#include <sstream>
#include <unordered_set>
using namespace omnetpp;
using namespace std;

class SDNNode : public cSimpleModule {
private:
    int addr;
    map<int, int> routes; // dst -> gate
    // seen packets (src, seqno) to avoid flooding duplicates
    std::unordered_set<uint64_t> seen;

protected:
    void initialize() override {
        addr = par("address");
    }

    void handleMessage(cMessage *msg) override {
        if (msg->getKind() == DATA_PKT) {
            // TTL handling: drop when ttl<=0, otherwise decrement
            if (msg->hasPar("ttl")) {
                long ttl = msg->par("ttl").longValue();
                if (ttl <= 0) {
                    EV_INFO << "Node " << addr << " dropping packet from " << SRC(msg) << " due to ttl=0\n";
                    delete msg;
                    return;
                }
                msg->par("ttl").setLongValue(ttl - 1);
            }
            int dst = DST(msg);
            // duplicate suppression: if packet has seqno, drop if already seen
            long seq = -1;
            if (msg->hasPar("seqno")) seq = msg->par("seqno").longValue();
            uint64_t key = 0;
            if (seq >= 0) {
                uint64_t s = (uint64_t)(uint32_t)seq;
                uint64_t src = (uint64_t)(uint32_t)SRC(msg);
                key = (src << 32) | s;
                if (seen.find(key) != seen.end()) {
                    EV_INFO << "Node " << addr << " dropping duplicate packet from " << SRC(msg) << " seq=" << seq << "\n";
                    delete msg;
                    return;
                }
                seen.insert(key);
            }
            
            // If this is destination
            if (dst == addr) {
                EV_INFO << "Node " << addr << " received packet from " 
                        << SRC(msg) << " Type=" << TYPE(msg) 
                        << " Pri=" << PRIORITY(msg) << "\n";
                delete msg;
                return;
            }
            
            // Check if route is embedded in message
            if (msg->hasPar("route")) {
                string routeStr = msg->par("route").stringValue();
                vector<int> path;
                stringstream ss(routeStr);
                string item;
                while (getline(ss, item, ',')) {
                    path.push_back(atoi(item.c_str()));
                }
                
                // Find next hop
                int nextHop = -1;
                for (size_t i = 0; i < path.size()-1; i++) {
                    if (path[i] == addr) {
                        nextHop = path[i+1];
                        break;
                    }
                }
                
                if (nextHop >= 0) {
                    // Find gate to next hop
                    for (int g = 0; g < gateSize("port"); g++) {
                        cGate *outGate = gate("port$o", g);
                        if (outGate->isConnected()) {
                            cModule *peer = outGate->getPathEndGate()->getOwnerModule();
                            int peerAddr = -1;
                            if (peer->hasPar("address"))
                                peerAddr = peer->par("address").intValue();
                            if (peerAddr == nextHop) {
                                send(msg, "port$o", g);
                                return;
                            }
                        }
                    }
                }
            }
            
            // Fallback: flood (skip incoming)
            EV_INFO << "Node " << addr << " flooding packet to " << dst << "\n";
            int inIdx = msg->getArrivalGate()->getIndex();
            for (int i = 0; i < gateSize("port"); i++) {
                if (i != inIdx) send(msg->dup(), "port$o", i);
            }
            delete msg;
        } else {
            delete msg;
        }
    }
};

Define_Module(SDNNode);