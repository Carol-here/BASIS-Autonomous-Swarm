# BASIS

## Blackboard-based Autonomous Swarm Intelligence System

**BASIS** is a multi-drone autonomous swarm intelligence system designed around a **Blackboard Architecture**.

The project explores how multiple autonomous drones can operate as a coordinated swarm by maintaining a shared representation of the environment and swarm state rather than relying on tightly coupled agent-to-agent communication.

The system is being developed as a simulation-first platform using **PX4, Gazebo, and ROS 2**, with the eventual goal of enabling autonomous task allocation, perception, replanning, and high-level intelligent decision making across a swarm of drones.

---

## Overview

Traditional multi-robot systems often rely on predefined behaviors or direct communication between individual robots.

BASIS takes a different approach.

Instead of designing the swarm as:

```text
Drone 1 <----> Drone 2
   |              |
   +----> Drone 3 <----+
```

BASIS uses a shared **Blackboard**:

```text
                    BLACKBOARD
                        |
        +---------------+---------------+
        |               |               |
     Drone State      Tasks        Detections
        |               |               |
        +---------------+---------------+
                        |
                     Agents
                        |
                  Decision Making
```

The Blackboard acts as the swarm's shared knowledge layer.

Individual components can read the current state of the swarm and contribute new information without requiring every component to know the implementation details of every other component.

---

# System Architecture

The current system is organized into several major layers.

```text
                         BASIS
                           |
              +------------+------------+
              |                         |
           Gazebo                      PX4
        Simulation                Flight Control
              |                         |
              +------------+------------+
                           |
                          ROS 2
                           |
                Swarm State Collection
                           |
                           v
                    +-------------+
                    | BLACKBOARD  |
                    +-------------+
                    |             |
              +-----+------+------+------+
              |     |      |      |      |
            Drones Tasks Detections Events
              |     |      |      |      |
              +-----+------+------+------+
                           |
                         Agents
                           |
                    Decision Making
```

### Layer 1 — Simulation

**Gazebo** provides the simulated environment in which the drone swarm operates.

The current environment includes a custom five-drone simulation world.

### Layer 2 — Flight Control

**PX4 SITL** provides the flight-control stack for each simulated drone.

PX4 is responsible for low-level vehicle control and flight-state management.

### Layer 3 — Communication

**ROS 2 Humble** provides the middleware connecting the different components of the system.

ROS 2 transports vehicle state and control information between PX4 and the BASIS software components.

### Layer 4 — Swarm State Collection

The swarm state collector subscribes to the state information of the simulated drones and converts it into a normalized representation suitable for higher-level reasoning.

The collected information includes:

* Position
* Velocity
* Armed state
* Flight state
* Failsafe state
* State freshness

The collector monitors all five simulated drones simultaneously.

### Layer 5 — Blackboard

The Blackboard provides a shared world model for the swarm.

It currently maintains separate knowledge categories for:

```text
Drones
Tasks
Detections
Events
Mission State
```

The Blackboard is implemented independently from ROS 2 so that higher-level agents do not need to directly depend on PX4 or ROS 2 interfaces.

---

# Blackboard Architecture

The Blackboard is the central knowledge repository of BASIS.

Conceptually:

```text
                     +----------------+
                     |   BLACKBOARD   |
                     +----------------+
                       /      |      \
                      /       |       \
                     v        v        v
                 Drone     Task      Detection
                 State     State       Data
                     \       |       /
                      \      |      /
                       v     v     v
                     +------------+
                     |   Agents   |
                     +------------+
```

The Blackboard currently exposes operations for:

### Drone State

```text
update_drone()
get_drone()
get_all_drones()
```

### Tasks

```text
add_task()
update_task()
get_tasks()
```

### Detections

```text
add_detection()
get_detections()
```

### Events

```text
add_event()
get_events()
```

### Mission State

```text
update_mission()
get_mission()
```

### World Snapshot

```text
get_snapshot()
```

`get_snapshot()` provides a consistent representation of the current swarm world model so that future autonomous agents can reason over the state of the system.

The Blackboard also uses thread-safe access to protect shared state when multiple components interact with it.

---

# Why a Blackboard Architecture?

A swarm can contain many independently operating components:

```text
Drone Agents
Task Allocation
Perception
Mapping
Battery Management
Safety
Mission Planning
```

Without a shared knowledge layer, these components can become tightly coupled.

For example:

```text
Perception -> Drone Agent
Drone Agent -> Task Manager
Task Manager -> Battery Manager
Battery Manager -> Drone Agent
```

As the number of agents increases, the number of dependencies can grow rapidly.

BASIS instead aims for:

```text
                  BLACKBOARD
                 /     |     \
                /      |      \
         Perception   Tasks   Resources
              |        |        |
              +--------+--------+
                       |
                     Agents
```

This allows agents to remain more independent while sharing a common world representation.

---

# Current System

The system currently supports a **five-drone PX4 SITL swarm**.

The five simulated vehicles communicate through ROS 2 and their state is collected into a unified swarm representation.

The state-monitoring architecture tracks:

```text
Drone 1
Drone 2
Drone 3
Drone 4
Drone 5
```

Each drone's state is represented independently before being incorporated into the shared swarm model.

The Blackboard can then maintain the combined world state.

---

# Development Phases

## Phase 1 — Single Drone

The initial phase established the fundamental simulation and communication pipeline.

Completed:

* PX4 SITL setup
* Gazebo simulation
* ROS 2 communication
* PX4 vehicle state access
* Basic offboard-control pipeline

---

## Phase 2 — Multi-Drone Control

The system was extended from a single vehicle to a multi-drone swarm.

Completed:

* Five PX4 simulated drones
* Multi-drone Gazebo simulation
* ROS 2 namespace separation
* Multi-drone offboard control
* Dedicated PX4 setpoint streaming
* PX4 command acknowledgement handling
* Swarm launch and control infrastructure

---

## Phase 3 — Swarm State Collection

The next stage focused on observing the swarm rather than controlling it.

The state collector was developed to monitor all five drones simultaneously.

Completed:

* Five-drone state collection
* Vehicle status subscriptions
* Vehicle odometry subscriptions
* State normalization
* State freshness tracking
* Swarm health monitoring
* Multi-drone state representation

The swarm monitoring system was successfully validated with all five simulated drones connected and reporting state.

---

## Phase 4 — Blackboard

Phase 4 introduced the shared knowledge layer.

### Phase 4A — Blackboard Core

Completed:

* Blackboard data structure
* Drone state storage
* Task storage
* Detection storage
* Event storage
* Mission state
* Thread-safe access
* Consistent world-state snapshots
* Standalone Blackboard validation

The Blackboard has been tested independently before being coupled with the ROS 2 state collection layer.

---

# Repository Structure

```text
BASIS-Autonomous-Swarm/
│
├── README.md
├── .gitignore
│
│
└── src/
    │
    ├── basis_control/
    │   │
    │   ├── package.xml
    │   ├── setup.py
    │   ├── setup.cfg
    │   │
    │   └── basis_control/
    │       │
    │       ├── phase2/
    │       │   ├── swarm_controller.py
    │       │   └── swarm_launcher.py
    │       │
    │       ├── phase3/
    │       │   ├── swarm_state.py
    │       │   └── swarm_state_collector.py
    │       │
    │       └── phase4/
    │           ├── blackboard.py
    │           └── blackboard_node.py
    │
    └── px4_msgs/
        ├── msg/
        ├── srv/
        ├── package.xml
        └── CMakeLists.txt
```

---

# Technology Stack

| Layer                | Technology              |
| -------------------- | ----------------------- |
| Operating System     | Ubuntu 22.04            |
| Environment          | WSL2                    |
| Middleware           | ROS 2 Humble            |
| Flight Controller    | PX4 SITL                |
| Simulator            | Gazebo Sim              |
| Programming Language | Python                  |
| Swarm Size           | 5 simulated drones      |
| Architecture         | Blackboard Architecture |

---

# Design Principles

### Separation of Concerns

Each layer has a specific responsibility.

```text
Gazebo
  → Simulation

PX4
  → Flight Control

ROS 2
  → Communication

State Collector
  → State Observation / Normalization

Blackboard
  → Shared Knowledge

Agents
  → Autonomous Decision Making

# Future Updates

The current implementation establishes the foundation for the autonomous swarm.

Planned future development includes:

* Blackboard integration with live swarm state
* Blackboard ROS 2 interface
* Autonomous drone agents
* Distributed task allocation
* Dynamic task reassignment
* Perception and target detection
* Shared detection information
* Battery/resource-aware planning
* Swarm mapping and belief fusion
* Dynamic event handling
* Autonomous replanning
* Mission-level coordination
* High-level LLM-assisted planning
* Deterministic safety and command validation
* Large-scale swarm experiments
* Quantitative swarm performance evaluation

The eventual system will investigate whether a Blackboard-based architecture can improve swarm coordination, adaptability, and replanning compared with more rigid centralized or predefined approaches.

---

# Long-Term Vision

The final system is intended to evolve toward:

```text
                    MISSION
                       |
                 High-Level Planner
                       |
                  +---------+
                  | BLACK-  |
                  | BOARD   |
                  +---------+
                       |
        +--------------+--------------+
        |              |              |
     Agent 1        Agent 2        Agent N
        |              |              |
      Drone 1        Drone 2        Drone N
        |              |              |
        +--------------+--------------+
                       |
                     PX4
                       |
                    Gazebo
```

A swarm should be able to:

1. Observe its environment.
2. Share information.
3. Maintain a common world model.
4. Determine which tasks need to be performed.
5. Allocate tasks to suitable drones.
6. Execute tasks autonomously.
7. Respond to failures and environmental changes.
8. Reassign tasks when necessary.
9. Replan missions dynamically.
10. Coordinate multiple autonomous vehicles toward a common objective.

---

# Project Status

**Active Development**

Current validated foundation:

```text
PX4 SITL
     ↓
Gazebo
     ↓
ROS 2
     ↓
5-Drone Swarm
     ↓
State Collection
     ↓
Blackboard Core
```

