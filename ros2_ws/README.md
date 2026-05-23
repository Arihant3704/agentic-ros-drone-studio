# ROS 2 Agentic Drone Control Workspace

This workspace houses the actual ROS 2 package `agentic_drone_control` which bridges a local cognitive Ollama AI Agent to a Gazebo-simulated drone running PX4 Autopilot.

---

## Workspace Layout

```text
ros2_ws/
├── README.md
└── src/
    └── agentic_drone_control/
        ├── agentic_drone_control/
        │   ├── __init__.py
        │   ├── agent_node.py           # Cognitive Agent (Ollama LLM)
        │   └── px4_offboard_controller.py # Low-level PX4 Bridge
        ├── launch/
        │   └── agentic_drone.launch.py # Launches both nodes
        ├── package.xml                 # Package description
        ├── setup.cfg                   # Executables config
        └── setup.py                    # Build instructions
```

---

## Interfacing Node Architecture

### 1. Low-level Controller (`px4_offboard_controller`)
*   **Subscribes to `/drone/cmd_pose`** (`geometry_msgs/PoseStamped` in ENU) -> Converts values to NED frame and publishes setpoints to PX4's `/fmu/in/trajectory_setpoint`.
*   **Heartbeat Publish (10Hz)** -> Continuously streams `OffboardControlMode` messages to PX4 on `/fmu/in/offboard_control_mode` to maintain Offboard flight control activation.
*   **Exposes Services:**
    *   `/drone/arm` (`std_srvs/SetBool`): Triggers MAVLink Component Arm/Disarm commands to `/fmu/in/vehicle_command`.
    *   `/drone/land` (`std_srvs/Trigger`): Commands PX4 to change mode to Auto-Land.

### 2. Cognitive Agent Node (`agent_node`)
*   **Subscribes to `/drone/user_command`** (`std_msgs/String`) -> Reads natural language instructions.
*   **Subscribes to `/drone/state`** (`geometry_msgs/PoseStamped`) -> Monitors vehicle telemetry in ENU coordinates.
*   **Logic (ReAct Loop):** 
    1. Triggers Ollama call (`qwen3.5:0.8b` prompt with tools list).
    2. Parses thought string and targets a tool.
    3. Triggers service requests to `/drone/arm`, `/drone/land`, or publishes target poses to `/drone/cmd_pose`.
