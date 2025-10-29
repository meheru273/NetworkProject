#ifndef ROUTECONTROLLER_H
#define ROUTECONTROLLER_H

#include <omnetpp.h>
#include <map>
#include <vector>
#include <queue>
#include "inet/networklayer/common/L3Address.h"
#include "messages/messages_m.h"

using namespace omnetpp;
using namespace inet;

class RouteController : public cSimpleModule {
private:
    struct Node {
        int id;
        std::vector<std::pair<int, double>> neighbors;  // neighbor id, link cost
        double utilization;
    };
    
    std::map<int, Node> network;
    std::map<std::pair<int, int>, std::vector<int>> backupPaths;

    // Routing algorithms
    std::vector<int> dijkstra(int src, int dest);
    std::vector<int> aStar(int src, int dest);
    std::vector<int> bellmanFord(int src, int dest);
    std::vector<int> loadBalancing(int src, int dest);
    
    double getHeuristic(int current, int dest);
    void updateNetworkState();
    std::vector<int> getBackupPath(int src, int dest);
    bool isLinkCongested(int node1, int node2);

protected:
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
};

#endif