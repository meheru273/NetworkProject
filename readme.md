# 🧠 SDN Network Simulation (OMNeT++ Project)

This project implements a **Software-Defined Networking (SDN)** simulation using **OMNeT++ 6.2**.  
It models data transmission between nodes, controlled routing, and differentiated traffic priorities.  
The simulation showcases packet generation, routing control, and QoS (Quality of Service) management.

---

## 📂 Project Directory Structure

NetworkProject/
└── src/
├── Modules/
│ ├── helpers.h # Common structures, enums, and helper functions
│ ├── RouteController.cc # Central controller handling routing decisions
│ ├── RouteController.h # Header for RouteController
│ ├── SDNNode.cc # Node logic for forwarding, TTL, packet handling
│ ├── TrafficGenerator.cc # Generates and sends packets based on traffic type
│
├── Network.ned # Network topology definition
├── omnetpp.ini # Simulation configuration
├── package.ned # OMNeT++ package declaration
├── results/ # Simulation output files (scalars, vectors, logs)
└── .qtenvrc # QtEnv runtime environment settings

markdown
Copy code

---

## ⚙️ Components Overview

### 🧩 1. `TrafficGenerator`
Located at: `Modules/TrafficGenerator.cc`

- Generates packets for various traffic types:
  - `video`, `datacenter`, `gaming`, `iot`
- Each type has an assigned **priority**:
  | Traffic Type | Priority | Description |
  |---------------|-----------|--------------|
  | Video         | 1 | High bandwidth, time-sensitive |
  | Datacenter    | 1 | Bulk data transfer |
  | Gaming        | 2 | Low-latency, interactive |
  | IoT           | 3 | Low-priority sensor data |
- Uses `QoSParams` (from `helpers.h`) including:
  - `bandwidth`, `ttl`, `priority`, and `type`
- Schedules packets at configurable intervals defined in `omnetpp.ini`.

---

### 🧠 2. `SDNNode`
Located at: `Modules/SDNNode.cc`

- Receives and forwards packets between nodes.
- Handles:
  - **TTL expiration** – drops packets exceeding lifetime.
  - **Duplicate suppression** – avoids reprocessing already received packets.
  - **Dynamic routing** – forwards using route info from controller.
  - **Flooding** – when no route is available.
- Records statistics:
  - `packetsReceived`, `packetsForwarded`, `packetsDropped`
  - `endToEndDelay`, `hopCount`

---

### 🧭 3. `RouteController`
Located at: `Modules/RouteController.cc` and `RouteController.h`

- Acts as a **centralized SDN controller**.
- Maintains **routing tables**, updates **network state**, and computes optimal paths.
- Interacts with `SDNNode` modules for route discovery and updates.
- Can dynamically enable or disable links and calculate new routes when topology changes.

---

### ⚒️ 4. `helpers.h`
- Contains shared data structures and functions, including:
  ```cpp
  enum TrafficType { VIDEO, DATACENTER, GAMING, IOT };

  struct QoSParams {
      TrafficType type;
      int priority;
      double bandwidth;
      int ttl;
      double delay;
  };
Also defines helper macros and mkData() for creating packets with parameters.

🕸️ 5. Network.ned
Defines the overall topology and module connections.

Example (simplified):

ned
Copy code
network Network
{
    submodules:
        controller: RouteController;
        node1: SDNNode { parameters: address = 1; }
        node2: SDNNode { parameters: address = 2; }
        gen1: TrafficGenerator {
            parameters:
                address = 1;
                destAddr = 2;
                trafficType = "video";
                numPackets = 10;
                startTime = 0s;
                sendInterval = 1s;
                ttl = 8;
                bandwidth = 10;
        }

    connections:
        gen1.ppp++ <--> node1.ppp++;
        node1.port++ <--> node2.port++;
        node2.port++ <--> controller.port++;
}
⚡ 6. omnetpp.ini
Defines simulation settings and runtime parameters.

Example snippet:

ini
Copy code
[General]
network = Network
sim-time-limit = 100s

**.numPackets = 5
**.sendInterval = 1s
**.startTime = 0s
**.bandwidth = 10
**.ttl = 8
**.trafficType = "video"

**.controller.address = 0
**.node*.address = index + 1
🧩 Simulation Flow
TrafficGenerator creates packets with QoS metadata.

Packets travel through SDNNode modules.

Each node checks TTL, duplicates, and routing table entries.

RouteController updates paths dynamically when necessary.

Metrics such as delay and packet drop are recorded to /results/.

📊 Output
After running the simulation, check:

Scalars → packet counts, delays

Vectors → time-series metrics

Results Folder: /src/results/

Example output (from OMNeT++ log):

csharp
Copy code
[INFO] Node 1 sending video packet #3 to 2 (priority=1)
[INFO] Node 2 received packet from 1 (delay=0.002s)
[INFO] Controller updated route from 1 → 2
🧪 Running the Simulation
▶️ From OMNeT++ IDE:
Open the project (NetworkProject).

Build using Run → Build Project.

Launch the simulation via Run → Run Configurations → omnetpp.ini.

🧰 From Command Line:
bash
Copy code
cd omnetpp-6.2.0/samples/NetworkProject/src
opp_run -u Cmdenv -f omnetpp.ini
🧱 Dependencies
OMNeT++ 6.2.0

C++17 or later

INET Framework (optional, if using standard network components)

📜 License
This project is for academic and research purposes only.
Feel free to modify or extend for learning.

Author: Meheru Zannat
Date: October 2025
Project: SDN-Based Network Simulation in OMNeT++