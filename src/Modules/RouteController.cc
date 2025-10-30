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
    map<int, bool> linkStatus; // linkId -> active/failed
    int numNodes;
    
    //  Link Failure Simulation
    cMessage *failureEvent;
    cMessage *recoveryEvent;
    int failedLink;
    
    // Performance Metrics
    vector<PacketMetrics> completedPackets;
    int totalPacketsSent;
    int totalPacketsDelivered;
    int totalPacketsDropped;
    map<RoutingAlgo, int> algoUsageCount;
    map<RoutingAlgo, double> algoTotalDelay;
    map<TrafficType, int> trafficDelivered;
    map<TrafficType, double> trafficTotalDelay;
    
    // K-Shortest Paths (k=3)
    vector<vector<int>> kShortestPaths;

protected:
    void initialize() override {
        numNodes = par("numNodes");
        buildTopology();
        computeBackupPaths();
        
        // Initialize metrics
        totalPacketsSent = 0;
        totalPacketsDelivered = 0;
        totalPacketsDropped = 0;
        
        // Schedule first link failure
        failureEvent = new cMessage("linkFailure", FAILURE_EVENT);
        scheduleAt(simTime() + par("failureStartTime").doubleValue(), failureEvent);
        
        recoveryEvent = nullptr;
        failedLink = -1;
        
        EV_INFO << "RouteController initialized with " << numNodes << " nodes\n";
    }

    void buildTopology() {
        // Build topology: nodes 0-9, edges with costs
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
            linkStatus[e[0]*100+e[1]] = true;
            linkStatus[e[1]*100+e[0]] = true;
        }
    }

    void computeBackupPaths() {
        // Compute backup paths for all pairs using Dijkstra
        for (int src = 0; src < numNodes; src++) {
            for (int dst = 0; dst < numNodes; dst++) {
                if (src != dst) {
                    auto path = dijkstra(src, dst, -1, -1);
                    backupPaths[src*100+dst] = path;
                }
            }
        }
    }

    vector<int> dijkstra(int src, int dst, int excludeNode, int excludeLink) {
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
                
                // Check if link is active
                int linkId = u.id*100 + v;
                if (linkId == excludeLink || !linkStatus[linkId]) continue;
                
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
                // Check if link is active
                int linkId = u.id*100 + v;
                if (!linkStatus[linkId]) continue;
                
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
                    // Check if link is active
                    int linkId = u*100 + v;
                    if (!linkStatus[linkId]) continue;
                    
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
                
                // Check if link is active
                if (!linkStatus[linkId]) continue;
                
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

    // K-Shortest Paths (Yen's Algorithm)
    vector<int> kShortestPath(int src, int dst, int k) {
        vector<vector<int>> A; // Store k shortest paths
        set<pair<double, vector<int>>> B; // Potential k shortest paths
        
        // Find first shortest path
        vector<int> firstPath = dijkstra(src, dst, -1, -1);
        if (firstPath.empty()) return firstPath;
        
        A.push_back(firstPath);
        
        for (int kk = 1; kk < k; kk++) {
            vector<int> prevPath = A[kk-1];
            
            for (size_t i = 0; i < prevPath.size() - 1; i++) {
                int spurNode = prevPath[i];
                vector<int> rootPath(prevPath.begin(), prevPath.begin() + i + 1);
                
                // Find spur path by excluding used nodes/links
                set<int> excludeNodes;
                for (auto& path : A) {
                    if (path.size() > i && 
                        vector<int>(path.begin(), path.begin() + i + 1) == rootPath) {
                        if (path.size() > i + 1) {
                            excludeNodes.insert(path[i+1]);
                        }
                    }
                }
                
                // Exclude nodes in root path except spur node
                for (size_t j = 0; j < i; j++) {
                    excludeNodes.insert(prevPath[j]);
                }
                
                // Try to find spur path
                vector<int> spurPath;
                for (int exclude : excludeNodes) {
                    spurPath = dijkstra(spurNode, dst, exclude, -1);
                    if (!spurPath.empty()) break;
                }
                
                if (!spurPath.empty() && spurPath[0] == spurNode) {
                    vector<int> totalPath = rootPath;
                    totalPath.insert(totalPath.end(), spurPath.begin() + 1, spurPath.end());
                    
                    double pathCost = calculatePathCost(totalPath);
                    B.insert({pathCost, totalPath});
                }
            }
            
            if (B.empty()) break;
            
            auto best = B.begin();
            A.push_back(best->second);
            B.erase(best);
        }
        
        // Return best alternative path based on current utilization
        if (A.size() > 1) {
            double minLoad = INFINITY;
            int bestIdx = 0;
            for (size_t i = 0; i < A.size(); i++) {
                double load = calculatePathLoad(A[i]);
                if (load < minLoad) {
                    minLoad = load;
                    bestIdx = i;
                }
            }
            return A[bestIdx];
        }
        
        return firstPath;
    }

    double calculatePathCost(const vector<int>& path) {
        double cost = 0;
        for (size_t i = 0; i < path.size() - 1; i++) {
            if (topology[path[i]].count(path[i+1])) {
                cost += topology[path[i]][path[i+1]];
            }
        }
        return cost;
    }

    double calculatePathLoad(const vector<int>& path) {
        double load = 0;
        for (size_t i = 0; i < path.size() - 1; i++) {
            int linkId = path[i]*100 + path[i+1];
            load += utilization[linkId];
        }
        return load;
    }

    RoutingAlgo selectAlgorithm(int priority, bool congested) {
        if (priority == 1) return congested ? LOAD_BALANCE : ASTAR;
        if (priority == 2) return DIJKSTRA;
        if (priority == 3 && congested) return KSHORT; // Use K-shortest for low priority when congested
        return BELLMAN_FORD;
    }

    vector<int> computeRoute(int src, int dst, RoutingAlgo algo) {
        switch (algo) {
            case ASTAR: return astar(src, dst);
            case DIJKSTRA: return dijkstra(src, dst, -1, -1);
            case BELLMAN_FORD: return bellmanFord(src, dst);
            case LOAD_BALANCE: return loadBalance(src, dst);
            case KSHORT: return kShortestPath(src, dst, 3);
            default: return dijkstra(src, dst, -1, -1);
        }
    }

    // Link Failure Handling
    void handleLinkFailure() {
        // Randomly select a link to fail
        vector<int> activeLinks;
        for (auto& [linkId, status] : linkStatus) {
            if (status) activeLinks.push_back(linkId);
        }
        
        if (activeLinks.empty()) return;
        
        int idx = intuniform(0, activeLinks.size() - 1);
        failedLink = activeLinks[idx];
        linkStatus[failedLink] = false;
        
        int u = failedLink / 100;
        int v = failedLink % 100;
        
        EV_WARN << "LINK FAILURE: Link " << u << " <-> " << v << " has FAILED at time " 
                << simTime() << "\n";
        
        // Schedule recovery
        if (recoveryEvent == nullptr) {
            recoveryEvent = new cMessage("linkRecovery", RECOVERY_EVENT);
        }
        scheduleAt(simTime() + par("recoveryTime").doubleValue(), recoveryEvent);
    }

    void handleLinkRecovery() {
        if (failedLink == -1) return;
        
        linkStatus[failedLink] = true;
        int u = failedLink / 100;
        int v = failedLink % 100;
        
        EV_INFO << "LINK RECOVERY: Link " << u << " <-> " << v << " has RECOVERED at time " 
                << simTime() << "\n";
        
        failedLink = -1;
        
        // Schedule next failure
        scheduleAt(simTime() + par("failureInterval").doubleValue(), failureEvent);
    }

    void handleMessage(cMessage *msg) override {
        // FEATURE Handle failure/recovery events
        if (msg->getKind() == FAILURE_EVENT) {
            handleLinkFailure();
            return;
        }
        
        if (msg->getKind() == RECOVERY_EVENT) {
            handleLinkRecovery();
            return;
        }
        
        if (msg->getKind() == DATA_PKT) {
            totalPacketsSent++;
            
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
                EV_WARN << "No route from " << src << " to " << dst << " - PACKET DROPPED\n";
                totalPacketsDropped++;
                delete msg;
                return;
            }
            
            // Update utilization
            for (size_t i = 0; i < path.size()-1; i++) {
                int linkId = path[i]*100 + path[i+1];
                utilization[linkId] += 0.1;
                if (utilization[linkId] > 1.0) utilization[linkId] = 1.0;
            }
            
            // Track metrics
            algoUsageCount[algo]++;
            
            // Store path in message
            stringstream ss;
            for (size_t i = 0; i < path.size(); i++) {
                if (i > 0) ss << ",";
                ss << path[i];
            }
            msg->addPar("route").setStringValue(ss.str().c_str());
            msg->addPar("algo").setLongValue(algo);
            
            const char* algoName = (algo==ASTAR?"A*":algo==DIJKSTRA?"Dijkstra":
                                   algo==BELLMAN_FORD?"Bellman-Ford":
                                   algo==LOAD_BALANCE?"LoadBalance":"K-Shortest");
            
            EV_INFO << "Route " << src << "->" << dst 
                    << " Pri=" << priority << " Algo=" << algoName
                    << " Path=" << ss.str() << " Hops=" << (path.size()-1) << "\n";
            
            send(msg, "ppp$o");
            
        } else if (msg->getKind() == LINK_FAILURE) {
            int failNode = msg->par("failedNode").longValue();
            EV_WARN << "Link failure at node " << failNode << ", using backup\n";
            delete msg;
        } else {
            delete msg;
        }
    }

    // Metrics collection and reporting
    void finish() override {
        cancelAndDelete(failureEvent);
        if (recoveryEvent != nullptr) {
            cancelAndDelete(recoveryEvent);
        }
        
        EV_INFO << "\n========== PERFORMANCE METRICS REPORT ==========\n";
        EV_INFO << "Total Packets Sent: " << totalPacketsSent << "\n";
        EV_INFO << "Total Packets Delivered: " << totalPacketsDelivered << "\n";
        EV_INFO << "Total Packets Dropped: " << totalPacketsDropped << "\n";
        
        if (totalPacketsSent > 0) {
            double deliveryRatio = (double)totalPacketsDelivered / totalPacketsSent * 100;
            EV_INFO << "Packet Delivery Ratio: " << deliveryRatio << "%\n";
        }
        
        EV_INFO << "\n--- Routing Algorithm Usage ---\n";
        for (auto& [algo, count] : algoUsageCount) {
            const char* algoName = (algo==ASTAR?"A*":algo==DIJKSTRA?"Dijkstra":
                                   algo==BELLMAN_FORD?"Bellman-Ford":
                                   algo==LOAD_BALANCE?"LoadBalance":"K-Shortest");
            EV_INFO << algoName << ": " << count << " times\n";
        }
        
        EV_INFO << "\n--- Network State ---\n";
        double avgUtil = 0;
        int linkCount = 0;
        for (auto& [linkId, util] : utilization) {
            avgUtil += util;
            linkCount++;
        }
        if (linkCount > 0) {
            avgUtil /= linkCount;
            EV_INFO << "Average Link Utilization: " << (avgUtil * 100) << "%\n";
        }
        
        EV_INFO << "================================================\n";
        
        // Record scalar statistics for OMNeT++
        recordScalar("packetsSent", totalPacketsSent);
        recordScalar("packetsDelivered", totalPacketsDelivered);
        recordScalar("packetsDropped", totalPacketsDropped);
        if (totalPacketsSent > 0) {
            recordScalar("deliveryRatio", (double)totalPacketsDelivered / totalPacketsSent);
        }
        recordScalar("avgLinkUtilization", avgUtil);
    }
};

Define_Module(RouteController);
