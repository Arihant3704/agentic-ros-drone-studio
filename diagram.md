# Agentic ROS Drone: Mermaid Workflow Diagrams

This document contains high-fidelity Mermaid diagrams illustrating the runtime loops, state transitions, and node communication protocols of the drone simulation.

---

## 1. Drone State Machine & Decision Tree

This state machine traces the lifecycle of a mission, detailing the criteria for state transitions and safety loops.

```mermaid
stateDiagram-v2
    [*] --> Disarmed : System Initialized
    
    Disarmed --> Armed : Operator Send Command & Battery > 20%
    Disarmed --> Failsafe_Hold : Battery <= 20%
    
    Armed --> Takeoff : Command: TAKEOFF(alt)
    Armed --> Disarmed : Timeout / Disarm Command
    
    Takeoff --> Hover : Altitude Target Reached (Z >= Target - 0.2m)
    Takeoff --> Auto_Land : Obstacle Detected / Critical Battery
    
    Hover --> Navigate : Command: NAVIGATE_TO(x, y, z)
    Hover --> Auto_Land : Low Battery / Disarm
    
    Navigate --> Hover : Waypoint Reached (Distance < 1.0m)
    Navigate --> Obstacle_Avoidance : LIDAR Warning (Distance < 10m)
    Navigate --> Auto_Land : Critical Battery / Emergency Land
    
    Obstacle_Avoidance --> Hover : Path Cleared / Steering Vector Completed
    
    Hover --> Auto_Land : Command: LAND() / Mission Complete
    Auto_Land --> Disarmed : Touchdown (Altitude <= 0.1m)
    Disarmed --> [*]
```

---

## 2. Multi-Agent Task Delegation Pipeline

This sequence diagram illustrates how a complex inspection request (e.g., *"Inspect the forest boundary for hiker signs"*) is distributed across the coordinator, navigation worker, perception worker, and safety guardrails.

```mermaid
sequenceDiagram
    autonumber
    actor Operator as Operator
    participant Coord as Coordinator Agent
    participant Nav as Navigation Agent
    participant Perc as Perception Agent
    participant Guard as Safety Guardrail
    participant PX4 as PX4 Flight Controller

    Operator->>Coord: Command: "Inspect boundary for hiker signs"
    Note over Coord: Decomposes mission:<br/>1. Patrol boundary coordinates<br/>2. Enable visual scan
    
    Coord->>Nav: Delegate: NAVIGATE_TO(80, 20, 5)
    Coord->>Perc: Delegate: ACTIVATE_CAMERA(target="hiker_sign")
    
    par Patrol Flight
        Nav->>Guard: Submit setpoint target (80, 20, 5)
        Note over Guard: Check against Geofence boundaries<br/>& LIDAR returns
        Guard->>PX4: Pass verified setpoints
        PX4-->>Nav: Broadcast updated position (x, y, z)
    and Visual Perception
        Perc->>PX4: Read Camera frame stream
        Note over Perc: Perform vision scanning<br/>(YOLO/VLM inference)
        Perc-->>Coord: Report: "Hiker sign spotted at (81, 19)"
    end

    Coord->>Nav: Direct: NAVIGATE_TO(81, 19, 3) (Lower for confirmation)
    Nav->>Guard: Submit confirmatory setpoint
    Guard->>PX4: Pass setpoint
    
    Coord->>Operator: Status: "Hiker found and verified. Landing."
    Coord->>Nav: Command: LAND()
    Nav->>PX4: Engage Offboard landing sequence
```

---

## 3. ROS 2 Node Topology & Communication Graph

This layout shows how nodes, topics, and service lines correspond to the physical deployment configuration in the ROS 2 workspace.

```mermaid
graph TD
    %% Node Definitions
    Sub1["/drone/user_command<br/>(std_msgs/String)"]
    NodeAgent["/agent_cognitive_node<br/>(AI Reasoning Node)"]
    NodeOffboard["/px4_offboard_controller<br/>(Coord Bridge)"]
    NodePercept["/perception_yolo_node<br/>(Vision Classifier)"]
    
    PX4_FMU["/fmu<br/>(PX4 Firmware / DDS Broker)"]
    
    %% Topic Links
    Sub1 -->|User instructions| NodeAgent
    
    NodeAgent -->|"/drone/arm (std_srvs/Trigger)"| NodeOffboard
    NodeAgent -->|"/drone/takeoff (std_srvs/Trigger)"| NodeOffboard
    NodeAgent -->|"/drone/navigate (geometry_msgs/PoseStamped)"| NodeOffboard
    
    NodePercept -->|"/perception/detect (std_msgs/String)"| NodeAgent
    
    NodeOffboard -->|"/fmu/in/trajectory_setpoint"| PX4_FMU
    NodeOffboard -->|"/fmu/in/vehicle_command"| PX4_FMU
    
    PX4_FMU -->|"/fmu/out/vehicle_odometry"| NodeOffboard
    PX4_FMU -->|"/fmu/out/sensor_combined"| NodeOffboard
    
    classDef node fill:#1e1e2e,stroke:#89b4fa,stroke-width:2px,color:#cdd6f4;
    classDef topic fill:#313244,stroke:#f5c2e7,stroke-width:1px,color:#cdd6f4;
    
    class NodeAgent,NodeOffboard,NodePercept,PX4_FMU node;
    class Sub1 topic;
```
