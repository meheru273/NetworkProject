#include <omnetpp.h>
#include "helpers.h"
#include <map>
#include <vector>
#include <queue>
#include <set>
#include <cmath>
#include <algorithm>
#include <limits>
#include <sstream>
#include <string>
using namespace omnetpp;
using namespace std;

class RouteController : public cSimpleModule {
private:
    struct Node { int id; double cost; bool operator>(const Node& o) const { return cost > o.cost; } };
    
    map<int, map<int, double>> topology; // adj[u][v] = cost
    map<int, double> utilization; // link utilization
    map<int, vector<int>> backupPaths; // backup routes
    int numNodes;

protected:
    void initialize() override {
        numNodes = par("numNodes");
        buildTopology();
        computeBackupPaths();
    }

    void buildTopology() {
        // Build topology: nodes 0-9, edges with costs
        // Organization network with known paths (for A* heuristic)
        int edges[][3] = {
            {0,1,10}, {0,2,15}, {1,3,12}, {1,4,15}, {2,4,10},
            {3,5,15}, {4,5,10}, {4,6,12}, {5,7,10}, {6,7,10},
            {6,8,15}, {7,9,10}, {8,9,12}
        };
        for (auto& e : edges) {
            topology[e[0]][e[1]] = e[2];
            topology[e[1]][e[0]] = e[2];
            utilization[e[0]*100+e[1]] = 0.0;
            utilization[e[1]*100+e[0]] = 0.0;
        }
    }

    void computeBackupPaths() {
        // Compute backup paths for all pairs using Dijkstra
        for (int src = 0; src < numNodes; src++) {
            for (int dst = 0; dst < numNodes; dst++) {
                if (src != dst) {
                    auto path = dijkstra(src, dst, -1);
                    backupPaths[src*100+dst] = path;
                }
            }
        }
    }

    vector<int> dijkstra(int src, int dst, int excludeNode) {
    vector<double> dist(numNodes, std::numeric_limits<double>::infinity());
        vector<int> prev(numNodes, -1);
        priority_queue<Node, vector<Node>, greater<Node>> pq;
        
        dist[src] = 0;
        pq.push({src, 0});
        
        while (!pq.empty()) {
            Node u = pq.top(); pq.pop();
            if (u.id == dst) break;
            if (u.cost > dist[u.id]) continue;
            if (u.id == excludeNode) continue;
            
            for (auto& [v, cost] : topology[u.id]) {
                if (v == excludeNode) continue;
                double newDist = dist[u.id] + cost;
                if (newDist < dist[v]) {
                    dist[v] = newDist;
                    prev[v] = u.id;
                    pq.push({v, newDist});
                }
            }
        }
        
        vector<int> path;
        if (prev[dst] == -1) return path;
        for (int at = dst; at != -1; at = prev[at]) path.push_back(at);
        std::reverse(path.begin(), path.end());
        return path;
    }

    vector<int> astar(int src, int dst) {
        auto heuristic = [](int a, int b) { return abs(a-b) * 8.0; };
        
    vector<double> gScore(numNodes, std::numeric_limits<double>::infinity());
    vector<double> fScore(numNodes, std::numeric_limits<double>::infinity());
        vector<int> prev(numNodes, -1);
        priority_queue<Node, vector<Node>, greater<Node>> pq;
        
        gScore[src] = 0;
        fScore[src] = heuristic(src, dst);
        pq.push({src, fScore[src]});
        
        while (!pq.empty()) {
            Node u = pq.top(); pq.pop();
            if (u.id == dst) break;
            
            for (auto& [v, cost] : topology[u.id]) {
                double tentG = gScore[u.id] + cost;
                if (tentG < gScore[v]) {
                    prev[v] = u.id;
                    gScore[v] = tentG;
                    fScore[v] = gScore[v] + heuristic(v, dst);
                    pq.push({v, fScore[v]});
                }
            }
        }
        
        vector<int> path;
        if (prev[dst] == -1) return path;
        for (int at = dst; at != -1; at = prev[at]) path.push_back(at);
        std::reverse(path.begin(), path.end());
        return path;
    }

    vector<int> bellmanFord(int src, int dst) {
    vector<double> dist(numNodes, std::numeric_limits<double>::infinity());
        vector<int> prev(numNodes, -1);
        dist[src] = 0;
        
        for (int i = 0; i < numNodes-1; i++) {
            for (auto& [u, neighbors] : topology) {
                for (auto& [v, cost] : neighbors) {
                    if (dist[u] + cost < dist[v]) {
                        dist[v] = dist[u] + cost;
                        prev[v] = u;
                    }
                }
            }
        }
        
        vector<int> path;
        if (prev[dst] == -1) return path;
        for (int at = dst; at != -1; at = prev[at]) path.push_back(at);
        std::reverse(path.begin(), path.end());
        return path;
    }

    vector<int> loadBalance(int src, int dst) {
        // Find path avoiding high-utilization links
        vector<double> dist(numNodes, INFINITY);
        vector<int> prev(numNodes, -1);
        priority_queue<Node, vector<Node>, greater<Node>> pq;
        
        dist[src] = 0;
        pq.push({src, 0});
        
        while (!pq.empty()) {
            Node u = pq.top(); pq.pop();
            if (u.id == dst) break;
            if (u.cost > dist[u.id]) continue;
            
            for (auto& [v, baseCost] : topology[u.id]) {
                int linkId = u.id*100 + v;
                double util = utilization[linkId];
                double cost = baseCost * (1 + util); // Penalize high utilization
                double newDist = dist[u.id] + cost;
                if (newDist < dist[v]) {
                    dist[v] = newDist;
                    prev[v] = u.id;
                    pq.push({v, newDist});
                }
            }
        }
        
        vector<int> path;
        if (prev[dst] == -1) return path;
        for (int at = dst; at != -1; at = prev[at]) path.push_back(at);
        reverse(path.begin(), path.end());
        return path;
    }

    RoutingAlgo selectAlgorithm(int priority, bool congested) {
        if (priority == 1) return congested ? LOAD_BALANCE : ASTAR;
        if (priority == 2) return DIJKSTRA;
        return BELLMAN_FORD;
    }

    vector<int> computeRoute(int src, int dst, RoutingAlgo algo) {
        switch (algo) {
            case ASTAR: return astar(src, dst);
            case DIJKSTRA: return dijkstra(src, dst, -1);
            case BELLMAN_FORD: return bellmanFord(src, dst);
            case LOAD_BALANCE: return loadBalance(src, dst);
            default: return dijkstra(src, dst, -1);
        }
    }

    void handleMessage(cMessage *msg) override {
        if (msg->getKind() == DATA_PKT) {
            int src = SRC(msg);
            int dst = DST(msg);
            int priority = PRIORITY(msg);
            
            // Check congestion
            double avgUtil = 0;
            int count = 0;
            for (auto& [k, v] : utilization) { avgUtil += v; count++; }
            avgUtil = count > 0 ? avgUtil/count : 0;
            bool congested = avgUtil > 0.7;
            
            // Select algorithm
            RoutingAlgo algo = selectAlgorithm(priority, congested);
            vector<int> path = computeRoute(src, dst, algo);
            
            if (path.empty()) {
                EV_WARN << "No route from " << src << " to " << dst << "\n";
                delete msg;
                return;
            }
            
            // Update utilization
            for (size_t i = 0; i < path.size()-1; i++) {
                int linkId = path[i]*100 + path[i+1];
                utilization[linkId] += 0.1;
                if (utilization[linkId] > 1.0) utilization[linkId] = 1.0;
            }
            
            // Store path in message
            stringstream ss;
            for (size_t i = 0; i < path.size(); i++) {
                if (i > 0) ss << ",";
                ss << path[i];
            }
            msg->addPar("route").setStringValue(ss.str().c_str());
            msg->addPar("algo").setLongValue(algo);
            
            EV_INFO << "Route " << src << "->" << dst 
                    << " Pri=" << priority << " Algo=" << algo 
                    << " Path=" << ss.str() << "\n";
            
            send(msg, "ppp$o");
        } else if (msg->getKind() == LINK_FAILURE) {
            int failNode = msg->par("failedNode").longValue();
            EV_WARN << "Link failure at node " << failNode << ", using backup\n";
            // In real scenario, would recompute avoiding failed node
            delete msg;
        } else {
            delete msg;
        }
    }
};

Define_Module(RouteController);