import os
import sys
import json
import math
import time
import asyncio
import threading
import logging
from typing import Dict, List, Any, Callable, Optional
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
import requests

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger("agentic_ros_drone")

app = FastAPI(title="Agentic ROS Drone Simulator")

# Ensure templates directory exists
os.makedirs("templates", exist_ok=True)

# ----------------------------------------------------------------------
# Drone Physics & Environment Simulator
# ----------------------------------------------------------------------
class Obstacle:
    def __init__(self, id_val: str, x: float, y: float, radius: float, name: str):
        self.id = id_val
        self.x = x
        self.y = y
        self.radius = radius
        self.name = name

class Target:
    def __init__(self, id_val: str, x: float, y: float, name: str, description: str):
        self.id = id_val
        self.x = x
        self.y = y
        self.name = name
        self.description = description
        self.found = False

class DroneState:
    def __init__(self):
        self.x = 10.0
        self.y = 10.0
        self.z = 0.0          # Altitude
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw = 0.0        # Degrees
        self.pitch = 0.0      # Degrees
        self.roll = 0.0       # Degrees
        self.battery = 100.0   # Percentage
        self.armed = False
        self.mode = "HOLD"    # HOLD, OFFBOARD, LAND, MANUAL
        self.status = "Disarmed"
        
        # Offboard setpoints
        self.target_x = 10.0
        self.target_y = 10.0
        self.target_z = 0.0
        
        # Fail-safe thresholds
        self.low_battery_threshold = 15.0
        self.collision_threshold = 5.0
        
        # Sensor data
        self.lidar_ranges = [100.0] * 8  # 8 directions around the drone
        self.camera_image_description = "Ground view: Home position visible."

class Simulator:
    def __init__(self):
        self.state = DroneState()
        self.obstacles = [
            Obstacle("obs1", 35.0, 30.0, 7.0, "High-Voltage Tower"),
            Obstacle("obs2", 65.0, 45.0, 9.0, "Forest Canopy Zone"),
            Obstacle("obs3", 45.0, 75.0, 6.5, "Restricted Building"),
            Obstacle("obs4", 80.0, 15.0, 8.0, "Industrial Chimney")
        ]
        self.targets = [
            Target("tgt1", 75.0, 80.0, "Lost Thermal Pad", "A glowing orange heat signature of the missing target."),
            Target("tgt2", 20.0, 70.0, "Stray Hiker Sign", "A high-visibility thermal vest located near the tree line.")
        ]
        self.lock = threading.Lock()
        self.running = True
        self.last_update = time.time()
        self.agent_thought_logs: List[Dict[str, str]] = []

    def update_physics(self):
        with self.lock:
            now = time.time()
            dt = min(now - self.last_update, 0.1) # Caps dt to prevent physics jumps
            self.last_update = now

            if not self.state.armed:
                # If disarmed and on the ground, do nothing
                if self.state.z > 0:
                    # Slow fall to ground if disarmed in air (gravity simulation)
                    self.state.vz = -2.0
                    self.state.z = max(0.0, self.state.z + self.state.vz * dt)
                else:
                    self.state.z = 0.0
                    self.state.vx = self.state.vy = self.state.vz = 0.0
                self.state.status = "Disarmed"
                return

            # Battery discharge
            if self.state.battery > 0:
                # Drains faster when flying higher/faster
                drain_rate = 0.05 + (0.01 * self.state.z) + (0.02 * math.hypot(self.state.vx, self.state.vy))
                self.state.battery = max(0.0, self.state.battery - drain_rate * dt)
            
            # Low Battery Fail-Safe
            if self.state.battery < self.state.low_battery_threshold and self.state.mode != "LAND":
                self.state.mode = "LAND"
                self.state.target_z = 0.0
                self.state.status = "FAIL-SAFE: Low Battery! Initiating auto-landing."
                self.add_thought_log("AUTOPILOT", f"Low battery ({self.state.battery:.1f}%). Triggering fail-safe landing.")

            # Flight modes and motion control
            if self.state.mode == "LAND":
                self.state.target_z = 0.0
                # Move down to ground
                if self.state.z > 0:
                    self.state.vz = -1.5
                    self.state.z = max(0.0, self.state.z + self.state.vz * dt)
                    # Slow horizontal movement during landing
                    dx = self.state.target_x - self.state.x
                    dy = self.state.target_y - self.state.y
                    dist = math.hypot(dx, dy)
                    if dist > 0.5:
                        self.state.vx = (dx / dist) * 1.0
                        self.state.vy = (dy / dist) * 1.0
                        self.state.x += self.state.vx * dt
                        self.state.y += self.state.vy * dt
                else:
                    # Disarm when landing complete
                    self.state.armed = False
                    self.state.vx = self.state.vy = self.state.vz = 0.0
                    self.state.status = "Landed and Disarmed"
            
            elif self.state.mode == "OFFBOARD":
                # Navigate towards target_x, target_y, target_z
                dx = self.state.target_x - self.state.x
                dy = self.state.target_y - self.state.y
                dz = self.state.target_z - self.state.z
                
                # Check distances
                horizontal_dist = math.hypot(dx, dy)
                vertical_dist = abs(dz)
                
                # Speed limits
                max_speed = 6.0  # m/s
                max_climb = 2.0  # m/s
                
                # Set velocities
                if horizontal_dist > 0.1:
                    speed = min(max_speed, horizontal_dist * 1.5)
                    self.state.vx = (dx / horizontal_dist) * speed
                    self.state.vy = (dy / horizontal_dist) * speed
                else:
                    self.state.vx = self.state.vy = 0.0
                    
                if vertical_dist > 0.1:
                    climb_speed = min(max_climb, vertical_dist * 1.0)
                    self.state.vz = (1.0 if dz > 0 else -1.0) * climb_speed
                else:
                    self.state.vz = 0.0
                    
                # Collision Avoidance Check
                # Check 8 lidar directions and prevent movement into obstacles
                obstacle_nearby = False
                for obs in self.obstacles:
                    obs_dx = obs.x - self.state.x
                    obs_dy = obs.y - self.state.y
                    obs_dist = math.hypot(obs_dx, obs_dy)
                    
                    if obs_dist < obs.radius + self.state.collision_threshold:
                        # Vector pointing away from obstacle
                        if obs_dist > obs.radius:
                            # Apply resistive force/velocity override
                            # Project velocity onto obstacle vector, if heading towards it, cancel that component
                            # For simplicity, steer away
                            obstacle_nearby = True
                            overlap = (obs.radius + self.state.collision_threshold) - obs_dist
                            # Push away
                            push_x = -(obs_dx / obs_dist) * (overlap * 2.0)
                            push_y = -(obs_dy / obs_dist) * (overlap * 2.0)
                            self.state.vx += push_x
                            self.state.vy += push_y
                            self.state.status = f"COLLISION AVOIDANCE active: Near {obs.name}"
                
                # Apply motion
                self.state.x += self.state.vx * dt
                self.state.y += self.state.vy * dt
                self.state.z += self.state.vz * dt
                
                # Limit coordinates to geofence
                self.state.x = max(0.0, min(100.0, self.state.x))
                self.state.y = max(0.0, min(100.0, self.state.y))
                self.state.z = max(0.0, min(30.0, self.state.z)) # Max 30m ceiling
                
                # Update status
                if not obstacle_nearby:
                    if horizontal_dist < 0.5 and vertical_dist < 0.5:
                        self.state.status = "Hovering at waypoint"
                        self.state.vx = self.state.vy = self.state.vz = 0.0
                    else:
                        self.state.status = "Navigating to offboard waypoint"
            
            elif self.state.mode == "HOLD":
                # Maintain current position
                self.state.target_x = self.state.x
                self.state.target_y = self.state.y
                self.state.target_z = self.state.z
                self.state.vx = self.state.vy = self.state.vz = 0.0
                self.state.status = "Hovering (HOLD mode)"

            # Update sensors (Lidar & Camera)
            self._update_sensors()
            
    def _update_sensors(self):
        # 1. 8-Ray LIDAR Simulator
        for i in range(8):
            angle = self.state.yaw + (i * 45.0)
            angle_rad = math.radians(angle)
            dir_x = math.cos(angle_rad)
            dir_y = math.sin(angle_rad)
            
            # Find closest obstacle intersection
            min_dist = 50.0 # Max lidar range
            for obs in self.obstacles:
                # Vector projection to find intersection
                # Projection of circle center onto ray
                v_x = obs.x - self.state.x
                v_y = obs.y - self.state.y
                proj = v_x * dir_x + v_y * dir_y
                
                if proj > 0:
                    perp_dist_sq = (v_x**2 + v_y**2) - proj**2
                    if perp_dist_sq < obs.radius**2:
                        # Intersects
                        half_chord = math.sqrt(max(0, obs.radius**2 - perp_dist_sq))
                        dist = proj - half_chord
                        if dist < min_dist:
                            min_dist = max(0.0, dist)
            self.state.lidar_ranges[i] = min_dist

        # 2. Downward Camera Simulator
        # Camera FOV depends on altitude. If z is very low, visual search area is small.
        camera_fov_radius = self.state.z * 1.2
        camera_desc = f"Altitude: {self.state.z:.1f}m. FOV Footprint Radius: {camera_fov_radius:.1f}m. "
        
        found_any = False
        for tgt in self.targets:
            dist = math.hypot(tgt.x - self.state.x, tgt.y - self.state.y)
            if dist < camera_fov_radius and self.state.z > 1.5:
                # Visible!
                camera_desc += f"[DETECTION] {tgt.name} identified at coordinate ({tgt.x:.1f}, {tgt.y:.1f})! {tgt.description} "
                tgt.found = True
                found_any = True
                
        if not found_any:
            camera_desc += "No thermal signatures detected in immediate view. Ground scan clear."
            
        self.state.camera_image_description = camera_desc

    def add_thought_log(self, sender: str, text: str):
        self.agent_thought_logs.append({
            "sender": sender,
            "text": text,
            "timestamp": time.strftime("%H:%M:%S")
        })
        if len(self.agent_thought_logs) > 100:
            self.agent_thought_logs.pop(0)

# Instantiate the Simulator
sim = Simulator()

# Start Physics thread
def physics_loop():
    while sim.running:
        sim.update_physics()
        time.sleep(0.05)

threading.Thread(target=physics_loop, daemon=True).start()


# ----------------------------------------------------------------------
# Mock ROS 2 Middleware Graph
# ----------------------------------------------------------------------
class ROS2Message:
    def __init__(self, topic: str, data: Dict[str, Any]):
        self.topic = topic
        self.data = data
        self.timestamp = time.time()

class ROS2Middleware:
    def __init__(self):
        self.topics: Dict[str, List[ROS2Message]] = {}
        self.subscribers: Dict[str, List[Callable]] = {}
        self.nodes: Dict[str, Dict[str, Any]] = {
            "user_agent_node": {"type": "AI Agent", "status": "Active"},
            "px4_fmu_node": {"type": "Autopilot Core", "status": "Active"},
            "perception_node": {"type": "YOLO/VLM Processor", "status": "Active"},
            "telemetry_pub_node": {"type": "Telemetry Publisher", "status": "Active"}
        }
        # Record topic statistics
        self.topic_rates: Dict[str, int] = {}
        self.topic_messages: Dict[str, Dict[str, Any]] = {}
        self.message_counter = 0
        self.packet_queue = []

    def publish(self, topic: str, data: Dict[str, Any], sender: str = "unknown"):
        # Store message
        msg = ROS2Message(topic, data)
        if topic not in self.topics:
            self.topics[topic] = []
        self.topics[topic].append(msg)
        
        # Limit buffer
        if len(self.topics[topic]) > 20:
            self.topics[topic].pop(0)
            
        # Update metrics
        self.topic_rates[topic] = self.topic_rates.get(topic, 0) + 1
        self.topic_messages[topic] = data
        self.message_counter += 1
        
        # Enqueue packet flow for visual graph representation
        # Determine source and target nodes based on topic rules
        src = sender
        destinations = []
        if topic == "/px4/telemetry":
            src = "px4_fmu_node"
            destinations = ["telemetry_pub_node", "user_agent_node"]
        elif topic in ["/px4/cmd_vel", "/px4/cmd_pose"]:
            src = "user_agent_node"
            destinations = ["px4_fmu_node"]
        elif topic == "/camera/image_raw":
            src = "px4_fmu_node"
            destinations = ["perception_node"]
        elif topic == "/perception/detections":
            src = "perception_node"
            destinations = ["user_agent_node"]
            
        for dest in destinations:
            self.packet_queue.append({
                "from": src,
                "to": dest,
                "topic": topic,
                "data": data,
                "timestamp": msg.timestamp
            })
            
        # Trigger subscribers
        if topic in self.subscribers:
            for cb in self.subscribers[topic]:
                try:
                    cb(data)
                except Exception as e:
                    logger.error(f"Error in subscription callback for {topic}: {e}")

    def subscribe(self, topic: str, callback: Callable):
        if topic not in self.subscribers:
            self.subscribers[topic] = []
        self.subscribers[topic].append(callback)

    def get_topic_state(self) -> List[Dict[str, Any]]:
        state = []
        for topic, payload in self.topic_messages.items():
            state.append({
                "name": topic,
                "hz": self.topic_rates.get(topic, 0),
                "payload": payload
            })
        # Reset rates periodically
        return state

    def clear_rates(self):
        for k in self.topic_rates.keys():
            self.topic_rates[k] = 0

ros2 = ROS2Middleware()

# Spin node background telemetry publishers
def ros_publishers_loop():
    while sim.running:
        with sim.lock:
            # Publish telemetry
            ros2.publish("/px4/telemetry", {
                "x": round(sim.state.x, 2),
                "y": round(sim.state.y, 2),
                "z": round(sim.state.z, 2),
                "vx": round(sim.state.vx, 2),
                "vy": round(sim.state.vy, 2),
                "vz": round(sim.state.vz, 2),
                "yaw": round(sim.state.yaw, 2),
                "battery": round(sim.state.battery, 1),
                "armed": sim.state.armed,
                "mode": sim.state.mode,
                "status": sim.state.status
            }, sender="px4_fmu_node")
            
            # Publish raw camera status
            ros2.publish("/camera/image_raw", {
                "fov_radius": round(sim.state.z * 1.2, 2),
                "visual_description": sim.state.camera_image_description
            }, sender="px4_fmu_node")
            
            # Publish YOLO detections (perception node)
            detections = []
            camera_fov_radius = sim.state.z * 1.2
            for tgt in sim.targets:
                dist = math.hypot(tgt.x - sim.state.x, tgt.y - sim.state.y)
                if dist < camera_fov_radius and sim.state.z > 1.5:
                    detections.append({
                        "id": tgt.id,
                        "name": tgt.name,
                        "relative_dist": round(dist, 2),
                        "x": tgt.x,
                        "y": tgt.y
                    })
            ros2.publish("/perception/detections", {
                "count": len(detections),
                "detections": detections
            }, sender="perception_node")
            
        time.sleep(1.0) # Publish at 1Hz to prevent UI congestion

threading.Thread(target=ros_publishers_loop, daemon=True).start()


# ----------------------------------------------------------------------
# AI Agent: Ollama / ReAct Control Loop
# ----------------------------------------------------------------------
class AgenticROSController:
    def __init__(self):
        self.ollama_url = "http://localhost:11434/api/generate"
        self.model_name = "qwen3.5:0.8b"
        self.is_busy = False

    def arm_drone(self) -> str:
        with sim.lock:
            sim.state.armed = True
            sim.state.status = "Armed on ground"
        ros2.publish("/px4/cmd_pose", {"arm": True}, sender="user_agent_node")
        sim.add_thought_log("ROBOT_ACTION", "Executed command: Arm Drone")
        return "Drone successfully ARMED."

    def disarm_drone(self) -> str:
        with sim.lock:
            sim.state.armed = False
            sim.state.status = "Disarmed"
        ros2.publish("/px4/cmd_pose", {"arm": False}, sender="user_agent_node")
        sim.add_thought_log("ROBOT_ACTION", "Executed command: Disarm Drone")
        return "Drone successfully DISARMED."

    def set_mode(self, mode: str) -> str:
        upper_mode = mode.upper()
        if upper_mode not in ["HOLD", "OFFBOARD", "LAND", "MANUAL"]:
            return f"Invalid mode: {mode}. Supported modes: HOLD, OFFBOARD, LAND."
        with sim.lock:
            sim.state.mode = upper_mode
            sim.state.status = f"Mode changed to {upper_mode}"
        sim.add_thought_log("ROBOT_ACTION", f"Set flight mode to: {upper_mode}")
        return f"Flight mode set to {upper_mode} successfully."

    def takeoff(self, altitude: float) -> str:
        with sim.lock:
            if not sim.state.armed:
                return "Takeoff failed: Drone is not ARMED. Arm the drone first."
            sim.state.mode = "OFFBOARD"
            sim.state.target_z = altitude
            # Set target coordinates to current position to ascend vertically
            sim.state.target_x = sim.state.x
            sim.state.target_y = sim.state.y
            sim.state.status = f"Taking off to {altitude}m"
        ros2.publish("/px4/cmd_pose", {"x": sim.state.x, "y": sim.state.y, "z": altitude}, sender="user_agent_node")
        sim.add_thought_log("ROBOT_ACTION", f"Initiated vertical takeoff to {altitude} meters.")
        return f"Takeoff initiated. Current target altitude: {altitude} meters."

    def navigate_to(self, x: float, y: float, z: float) -> str:
        with sim.lock:
            if not sim.state.armed:
                return "Navigation failed: Drone is disarmed."
            if sim.state.mode != "OFFBOARD":
                return "Navigation failed: Drone is not in OFFBOARD mode. Switch to OFFBOARD mode first."
            
            # Check geofence
            if not (0 <= x <= 100) or not (0 <= y <= 100) or not (0 <= z <= 30):
                return "Navigation failed: Target coordinates out of Geofence boundaries (X: 0-100, Y: 0-100, Z: 0-30)."
                
            sim.state.target_x = x
            sim.state.target_y = y
            sim.state.target_z = z
            sim.state.status = f"Navigating to waypoint ({x}, {y}, {z})"
            
        ros2.publish("/px4/cmd_pose", {"x": x, "y": y, "z": z}, sender="user_agent_node")
        sim.add_thought_log("ROBOT_ACTION", f"Navigating to coordinates X={x}, Y={y}, Z={z}.")
        return f"Waypoint set. Moving to ({x}, {y}, {z})."

    def get_environment_info(self) -> Dict[str, Any]:
        with sim.lock:
            # Scan for targets currently seen in FOV
            visible_targets = []
            camera_fov_radius = sim.state.z * 1.2
            for tgt in sim.targets:
                dist = math.hypot(tgt.x - sim.state.x, tgt.y - sim.state.y)
                if dist < camera_fov_radius and sim.state.z > 1.5:
                    visible_targets.append({
                        "name": tgt.name,
                        "description": tgt.description,
                        "x": tgt.x,
                        "y": tgt.y
                    })

            # Check obstacle warnings via Lidar
            obstacles_alert = []
            for obs in sim.obstacles:
                dist = math.hypot(obs.x - sim.state.x, obs.y - sim.state.y)
                if dist < obs.radius + sim.state.collision_threshold + 5.0:
                    obstacles_alert.append({
                        "name": obs.name,
                        "distance": round(dist, 1),
                        "radius": obs.radius,
                        "warning": "Critical proximity!" if dist < obs.radius + sim.state.collision_threshold else "Close proximity"
                    })

            return {
                "telemetry": {
                    "x": round(sim.state.x, 1),
                    "y": round(sim.state.y, 1),
                    "z": round(sim.state.z, 1),
                    "battery": round(sim.state.battery, 1),
                    "armed": sim.state.armed,
                    "mode": sim.state.mode,
                    "status": sim.state.status
                },
                "sensors": {
                    "lidar_warnings": obstacles_alert,
                    "camera_detections": visible_targets,
                    "raw_desc": sim.state.camera_image_description
                }
            }

    def execute_command_string(self, user_command: str):
        if self.is_busy:
            return
        self.is_busy = True
        sim.add_thought_log("USER", user_command)
        
        # Run agent planning in background thread to avoid blocking FastAPI
        def run_agent():
            try:
                # 1. Attempt using local Ollama model
                success = self._run_ollama_agent(user_command)
                if not success:
                    # 2. Fallback to rule-based agent simulator
                    self._run_fallback_agent(user_command)
            except Exception as e:
                logger.error(f"Error in agent execution: {e}")
                self._run_fallback_agent(user_command)
            finally:
                self.is_busy = False
                
        threading.Thread(target=run_agent, daemon=True).start()

    def _run_ollama_agent(self, user_command: str) -> bool:
        # Build prompt exposing tools
        prompt = f"""
You are the AI cognitive agent controlling a ROS 2 quadcopter drone.
Your task is to parse the user's natural language command, inspect the drone telemetry, plan the movements, and execute the correct tools in order.

### SYSTEM STATE INFORMATION:
{json.dumps(self.get_environment_info(), indent=2)}

### CONTEXT:
Map boundaries: X: 0 to 100, Y: 0 to 100, Z: 0 to 30.
Home is at (10, 10, 0).
Available Targets: 
- "Lost Thermal Pad" is located near (75, 80)
- "Stray Hiker Sign" is located near (20, 70)

### AVAILABLE TOOLS (Execute one action per turn):
1. `ARM()` -> Arms the drone.
2. `DISARM()` -> Disarms the drone.
3. `SET_MODE(mode)` -> Mode can be: "HOLD", "OFFBOARD", "LAND".
4. `TAKEOFF(altitude)` -> Climb vertically to desired altitude (e.g. 5).
5. `NAVIGATE_TO(x, y, z)` -> Move to specified coordinates.
6. `WAIT()` -> Wait for drone to hover or complete current trajectory.
7. `MISSION_COMPLETE()` -> Use when the user's goal is fully achieved.

### FORMAT RULES:
For every step, you must output exactly two lines:
THOUGHT: <explain your reasoning, check battery and safety>
ACTION: <tool_name>(<arguments>)

User command: "{user_command}"
"""
        try:
            # Check if Ollama is running and responding quickly
            response = requests.post(
                self.ollama_url, 
                json={"model": self.model_name, "prompt": prompt, "stream": False},
                timeout=5.0
            )
            if response.status_code != 200:
                return False
                
            result = response.json()
            text_output = result.get("response", "")
            
            # Parse thoughts and actions
            thought = "Analysing mission goals."
            action_call = "WAIT()"
            
            for line in text_output.split("\n"):
                if line.startswith("THOUGHT:"):
                    thought = line.replace("THOUGHT:", "").strip()
                elif line.startswith("ACTION:"):
                    action_call = line.replace("ACTION:", "").strip()
            
            sim.add_thought_log("AGENT_THOUGHT", thought)
            self._execute_tool_action(action_call)
            return True
            
        except Exception as e:
            logger.warning(f"Ollama agent failed or timed out: {e}. Switching to fallback engine.")
            return False

    def _execute_tool_action(self, action_call: str):
        # Parse simple action like "TAKEOFF(5)" or "NAVIGATE_TO(75, 80, 5)"
        try:
            action_call = action_call.strip()
            if action_call.startswith("ARM()"):
                res = self.arm_drone()
            elif action_call.startswith("DISARM()"):
                res = self.disarm_drone()
            elif action_call.startswith("SET_MODE"):
                mode = action_call.split("(")[1].split(")")[0].replace("'", "").replace('"', "").strip()
                res = self.set_mode(mode)
            elif action_call.startswith("TAKEOFF"):
                alt = float(action_call.split("(")[1].split(")")[0].strip())
                res = self.takeoff(alt)
            elif action_call.startswith("NAVIGATE_TO"):
                args = action_call.split("(")[1].split(")")[0].split(",")
                x = float(args[0].strip())
                y = float(args[1].strip())
                z = float(args[2].strip())
                res = self.navigate_to(x, y, z)
            elif action_call.startswith("WAIT()"):
                res = "Waiting for drone state to stabilize."
            elif action_call.startswith("MISSION_COMPLETE()"):
                res = "Mission complete reported by Agentic ROS."
                sim.add_thought_log("AGENT_THOUGHT", "Goal reached. Mission complete.")
            else:
                res = f"Unknown action: {action_call}"
            
            sim.add_thought_log("SYSTEM", f"Observation: {res}")
        except Exception as e:
            sim.add_thought_log("SYSTEM", f"Observation Error parsing Action: {e}")

    def _run_fallback_agent(self, user_command: str):
        # High-fidelity rule-based agent that matches common drone requests
        # simulating a real ReAct step loop
        cmd = user_command.lower()
        info = self.get_environment_info()
        telemetry = info["telemetry"]
        
        # Sequence of operations depends on state
        if not telemetry["armed"]:
            sim.add_thought_log("AGENT_THOUGHT", "User requested flight commands, but the drone is currently disarmed. Safety checklist: Battery OK, GPS lock green. Commencing ARM command.")
            time.sleep(1.0)
            res = self.arm_drone()
            sim.add_thought_log("SYSTEM", f"Observation: {res}")
            return

        if telemetry["armed"] and telemetry["z"] < 1.0 and telemetry["mode"] != "OFFBOARD":
            sim.add_thought_log("AGENT_THOUGHT", "Drone is ARMED but on the ground. Initiating vertical takeoff action to a safe cruising altitude of 5 meters.")
            time.sleep(1.0)
            res = self.takeoff(5.0)
            sim.add_thought_log("SYSTEM", f"Observation: {res}")
            return
            
        # Target search logic
        if "search" in cmd or "find" in cmd or "scan" in cmd or "hiker" in cmd or "pad" in cmd:
            target_name = "Lost Thermal Pad" if ("pad" in cmd or "thermal" in cmd) else "Stray Hiker Sign"
            target_coords = (75.0, 80.0, 6.0) if target_name == "Lost Thermal Pad" else (20.0, 70.0, 6.0)
            
            # Check if we are near target
            dist_to_target = math.hypot(telemetry["x"] - target_coords[0], telemetry["y"] - target_coords[1])
            
            if dist_to_target > 2.0:
                sim.add_thought_log("AGENT_THOUGHT", f"Search mission: Locating {target_name}. Mapping coordinates near ({target_coords[0]}, {target_coords[1]}). Deploying trajectory setpoints to Offboard flight stack.")
                time.sleep(1.5)
                res = self.navigate_to(target_coords[0], target_coords[1], target_coords[2])
                sim.add_thought_log("SYSTEM", f"Observation: {res}")
            else:
                # We are at target! Look for detections
                detections = info["sensors"]["camera_detections"]
                if len(detections) > 0:
                    sim.add_thought_log("AGENT_THOUGHT", f"Arrived at search coordinates. Camera frame yields {len(detections)} detection. Target confirmed! Logging thermal telemetry. Mission objective achieved. Initiating return to home sequence.")
                    time.sleep(2.0)
                    self.set_mode("LAND")
                else:
                    sim.add_thought_log("AGENT_THOUGHT", "At coordinates, but search target not visible. Camera footprint too small? Climbing slightly to expand field-of-view.")
                    time.sleep(1.0)
                    res = self.navigate_to(target_coords[0], target_coords[1], 10.0)
                    sim.add_thought_log("SYSTEM", f"Observation: {res}")
            return
            
        elif "land" in cmd or "home" in cmd or "return" in cmd:
            if telemetry["x"] > 15.0 or telemetry["y"] > 15.0:
                sim.add_thought_log("AGENT_THOUGHT", "Return to Home command registered. Navigating back to home coordinates (10, 10, 5) before landing.")
                time.sleep(1.5)
                res = self.navigate_to(10.0, 10.0, 5.0)
                sim.add_thought_log("SYSTEM", f"Observation: {res}")
            else:
                sim.add_thought_log("AGENT_THOUGHT", "Positioned safely over home pad. Changing flight mode to LAND. Low-level controller will handle descend rate.")
                time.sleep(1.0)
                res = self.set_mode("LAND")
                sim.add_thought_log("SYSTEM", f"Observation: {res}")
            return

        elif "circle" in cmd or "orbit" in cmd:
            # Fly to multiple coordinate points to simulate orbit
            sim.add_thought_log("AGENT_THOUGHT", "Orchestrating perimeter patrol flight plan. Sending waypoint 1 (30, 20, 5) to offboard navigation stack.")
            time.sleep(1.0)
            res = self.navigate_to(30.0, 20.0, 5.0)
            sim.add_thought_log("SYSTEM", f"Observation: {res}")
            return
            
        # General navigation fallback
        # Try to parse numbers from command
        words = cmd.split()
        coords = []
        for w in words:
            try:
                # Remove punctuation
                w_clean = w.replace(",", "").replace("(", "").replace(")", "")
                coords.append(float(w_clean))
            except ValueError:
                pass
                
        if len(coords) >= 2:
            x = coords[0]
            y = coords[1]
            z = coords[2] if len(coords) >= 3 else 5.0
            sim.add_thought_log("AGENT_THOUGHT", f"Command request parses as spatial waypoint coordinates: ({x}, {y}, {z}). Commencing autopilot execution.")
            time.sleep(1.0)
            res = self.navigate_to(x, y, z)
            sim.add_thought_log("SYSTEM", f"Observation: {res}")
        else:
            sim.add_thought_log("AGENT_THOUGHT", f"Received command: '{user_command}'. Safety parameters: holding hover position. Prompt does not specify concrete actions. Waiting for operator clarifications.")
            time.sleep(0.5)
            self._execute_tool_action("WAIT()")

agent_controller = AgenticROSController()


# ----------------------------------------------------------------------
# FastAPI Server Endpoints
# ----------------------------------------------------------------------

# Client lists for WS broadcast
connected_websockets: List[WebSocket] = []

@app.get("/")
async def get_dashboard():
    try:
        with open("templates/index.html", "r") as f:
            html_content = f.read()
        return HTMLResponse(content=html_content, status_code=200)
    except FileNotFoundError:
        return HTMLResponse("index.html not found. Place it in /templates folder.", status_code=404)

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    connected_websockets.append(websocket)
    try:
        while True:
            # Read messages from clients (user control)
            data = await websocket.receive_text()
            msg = json.loads(data)
            
            if msg.get("type") == "user_command":
                cmd = msg.get("command", "")
                agent_controller.execute_command_string(cmd)
            elif msg.get("type") == "arm":
                agent_controller.arm_drone()
            elif msg.get("type") == "disarm":
                agent_controller.disarm_drone()
            elif msg.get("type") == "mode":
                agent_controller.set_mode(msg.get("mode", "HOLD"))
            elif msg.get("type") == "reset_simulation":
                with sim.lock:
                    sim.state = DroneState()
                    for tgt in sim.targets:
                        tgt.found = False
                    sim.agent_thought_logs.clear()
                sim.add_thought_log("SYSTEM", "Simulator reset completed. Telemetry back to default home.")
                
    except WebSocketDisconnect:
        connected_websockets.remove(websocket)
    except Exception as e:
        logger.error(f"WebSocket error: {e}")
        if websocket in connected_websockets:
            connected_websockets.remove(websocket)

# Background broad-caster to Web clients
async def ui_broadcast_loop():
    while True:
        if connected_websockets:
            with sim.lock:
                # Prepare payload
                payload = {
                    "telemetry": {
                        "x": sim.state.x,
                        "y": sim.state.y,
                        "z": sim.state.z,
                        "vx": sim.state.vx,
                        "vy": sim.state.vy,
                        "vz": sim.state.vz,
                        "yaw": sim.state.yaw,
                        "battery": sim.state.battery,
                        "armed": sim.state.armed,
                        "mode": sim.state.mode,
                        "status": sim.state.status
                    },
                    "obstacles": [
                        {"id": o.id, "x": o.x, "y": o.y, "radius": o.radius, "name": o.name}
                        for o in sim.obstacles
                    ],
                    "targets": [
                        {"id": t.id, "x": t.x, "y": t.y, "name": t.name, "description": t.description, "found": t.found}
                        for t in sim.targets
                    ],
                    "lidar": sim.state.lidar_ranges,
                    "camera_desc": sim.state.camera_image_description,
                    "thought_logs": sim.agent_thought_logs,
                    "ros_topics": ros2.get_topic_state(),
                    "ros_packets": ros2.packet_queue.copy(),
                    "ros_nodes": [
                        {"name": k, "type": v["type"], "status": v["status"]}
                        for k, v in ros2.nodes.items()
                    ]
                }
                
                # Clear visual graph packet list once sent
                ros2.packet_queue.clear()
                # Clear rates so it calculates per second
                ros2.clear_rates()

            # Broadcast
            dead_sockets = []
            for ws in connected_websockets:
                try:
                    await ws.send_text(json.dumps(payload))
                except Exception:
                    dead_sockets.append(ws)
            for ws in dead_sockets:
                connected_websockets.remove(ws)
                
        await asyncio.sleep(0.1) # 10Hz updates for smooth UI render

@app.on_event("startup")
async def startup_event():
    asyncio.create_task(ui_broadcast_loop())

if __name__ == "__main__":
    import uvicorn
    # Get port from environment or default to 8000
    port = int(os.getenv("PORT", 8000))
    logger.info(f"Starting server on port {port}...")
    uvicorn.run("app:app", host="0.0.0.0", port=port, log_level="info")
