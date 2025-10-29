#ifndef MODULES_HELPERS_H_
#define MODULES_HELPERS_H_

#include <omnetpp.h>
#include <map>
#include <sstream>
using namespace omnetpp;
using namespace std;

// Traffic types
enum TrafficType { VIDEO=1, DATACENTER=2, GAMING=3, IOT=4 };

// Message kinds
enum { DATA_PKT=10, ROUTE_REQUEST=20, ROUTE_REPLY=21, LINK_FAILURE=30,
       DNS_QUERY=40, DNS_RESPONSE=41, HTTP_GET=50, HTTP_RESPONSE=51 };

// Routing algorithms
enum RoutingAlgo { ASTAR=1, DIJKSTRA=2, BELLMAN_FORD=3, LOAD_BALANCE=4 };

// QoS parameters structure
struct QoSParams {
    TrafficType type;
    int priority;
    double bandwidth;
    double delay;
    int ttl; // hop limit for packets
};

// Create message - same pattern as your mk() function
static cMessage* mkData(const char* name, long src, long dst, const QoSParams& qos) {
    auto *m = new cMessage(name, DATA_PKT);
    m->addPar("src").setLongValue(src);
    m->addPar("dst").setLongValue(dst);
    m->addPar("type").setLongValue(qos.type);
    m->addPar("priority").setLongValue(qos.priority);
    m->addPar("bandwidth").setDoubleValue(qos.bandwidth);
    m->addPar("ttl").setLongValue(qos.ttl);
    return m;
}

// Generic message maker used by modules
static inline cMessage* mk(const char* name, int kind, long src, long dst) {
    auto *m = new cMessage(name, kind);
    m->addPar("src").setLongValue(src);
    m->addPar("dst").setLongValue(dst);
    return m;
}

static inline long SRC(cMessage* m){ return m->par("src").longValue(); }
static inline long DST(cMessage* m){ return m->par("dst").longValue(); }
static inline int PRIORITY(cMessage* m){ return m->par("priority").longValue(); }
static inline TrafficType TYPE(cMessage* m){ return (TrafficType)m->par("type").longValue(); }

#endif /* MODULES_HELPERS_H_ */
