#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

import time
import json
import math
import requests
import threading
from typing import Dict, Any

class AgentNode(Node):
    def __init__(self):
        super().__init__('agent_node')

        # ----------------- ROS 2 Publishers & Clients -----------------
        self.cmd_pose_pub = self.create_publisher(PoseStamped, '/drone/cmd_pose', 10)
        self.log_pub = self.create_publisher(String, '/drone/agent_logs', 10)

        self.arm_client = self.create_client(SetBool, '/drone/arm')
        self.land_client = self.create_client(Trigger, '/drone/land')

        # ----------------- ROS 2 Subscribers -----------------
        self.state_sub = self.create_subscription(
            PoseStamped, '/drone/state', self.state_callback, 10)
        self.cmd_sub = self.create_subscription(
            String, '/drone/user_command', self.command_callback, 10)
        self.perception_sub = self.create_subscription(
            String, '/perception/detections', self.perception_callback, 10)

        # ----------------- State variables -----------------
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_z = 0.0
        self.curr_yaw = 0.0
        self.detections_summary = "No objects detected."

        self.ollama_url = "http://localhost:11434/api/generate"
        self.model_name = "qwen3.5:0.8b"
        self.is_busy = False

        self.get_logger().info('ROS 2 Agent Node Initialized. Listening on /drone/user_command...')

    def state_callback(self, msg):
        self.curr_x = msg.pose.position.x
        self.curr_y = msg.pose.position.y
        self.curr_z = msg.pose.position.z
        
        # Quaternion to Yaw
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.curr_yaw = math.atan2(siny_cosp, cosy_cosp)

    def perception_callback(self, msg):
        self.detections_summary = msg.data

    def command_callback(self, msg):
        user_cmd = msg.data
        if self.is_busy:
            self.get_logger().warn('Agent is currently processing a command. Ignoring request.')
            return
        
        self.is_busy = True
        self.get_logger().info(f'Received user command: "{user_cmd}"')
        self.publish_log(f"USER: {user_cmd}")

        # Run reasoning cycle in a separate thread so we don't block the ROS 2 executor
        threading.Thread(target=self.reasoning_loop, args=(user_cmd,), daemon=True).start()

    def publish_log(self, text: str):
        msg = String()
        msg.data = f"[{time.strftime('%H:%M:%S')}] {text}"
        self.log_pub.publish(msg)
        self.get_logger().info(text)

    def call_arm_service(self, arm_state: bool) -> str:
        if not self.arm_client.wait_for_service(timeout_sec=2.0):
            return "Failed to call ARM service: Service not available."
        
        req = SetBool.Request()
        req.data = arm_state
        future = self.arm_client.call_async(req)
        # Block until callback is done (safe inside thread)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        return res.message if res else "No response from service."

    def call_land_service(self) -> str:
        if not self.land_client.wait_for_service(timeout_sec=2.0):
            return "Failed to call LAND service: Service not available."
        
        req = Trigger.Request()
        future = self.land_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        return res.message if res else "No response from service."

    def navigate_to(self, x: float, y: float, z: float):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        
        # Orient facing target or zero rotation
        msg.pose.orientation.w = 1.0
        self.cmd_pose_pub.publish(msg)

    def get_system_telemetry(self) -> Dict[str, Any]:
        return {
            "telemetry": {
                "x": round(self.curr_x, 2),
                "y": round(self.curr_y, 2),
                "z": round(self.curr_z, 2),
                "yaw": round(self.curr_yaw, 2),
            },
            "perception": {
                "detections": self.detections_summary
            }
        }

    def reasoning_loop(self, user_cmd: str):
        # ReAct prompt detailing the tools and interface
        prompt = f"""
You are the cognitive node of an Agentic ROS 2 drone system in Gazebo.
You control a PX4-based quadcopter. Coordinate frames are standard ENU:
Map limits: X: 0 to 100m, Y: 0 to 100m, Z: 0 to 30m ceiling.
Home is at (0, 0, 0).

### SYSTEM TELEMETRY & PERCEPTION:
{json.dumps(self.get_system_telemetry(), indent=2)}

### AVAILABLE ROS 2 INTERFACES (Choose ONE tool call per turn):
1. `ARM()` -> Arm the quadcopter.
2. `DISARM()` -> Disarm the quadcopter.
3. `TAKEOFF(altitude)` -> Takeoff vertically to specified height (e.g. 5.0).
4. `NAVIGATE_TO(x, y, z)` -> Fly to specified coordinates (e.g., NAVIGATE_TO(45.0, 50.0, 5.0)).
5. `LAND()` -> Initiate auto-landing.
6. `WAIT()` -> Hover and wait for stabilizing state.
7. `MISSION_COMPLETE()` -> Conclude mission loop.

### FORMAT RULES:
For every step, you must output exactly two lines:
THOUGHT: <explain reasoning, inspect geofence limits, describe next steps>
ACTION: <tool_name>(<arguments>)

User mission command: "{user_cmd}"
"""
        try:
            # 1. Try local Ollama model
            self.publish_log("Ollama Agent thinking...")
            response = requests.post(
                self.ollama_url,
                json={"model": self.model_name, "prompt": prompt, "stream": False},
                timeout=8.0
            )
            if response.status_code == 200:
                result = response.json()
                text_output = result.get("response", "")
                
                thought = "Analysing flight path."
                action_call = "WAIT()"
                
                for line in text_output.split("\n"):
                    if line.startswith("THOUGHT:"):
                        thought = line.replace("THOUGHT:", "").strip()
                    elif line.startswith("ACTION:"):
                        action_call = line.replace("ACTION:", "").strip()

                self.publish_log(f"AGENT_THOUGHT: {thought}")
                self.execute_agent_action(action_call)
            else:
                self.run_fallback_agent(user_cmd)
        except Exception as e:
            self.get_logger().warn(f"Ollama call failed ({e}). Running fallback rules.")
            self.run_fallback_agent(user_cmd)
        finally:
            self.is_busy = False

    def execute_agent_action(self, action_call: str):
        try:
            action_call = action_call.strip()
            if action_call.startswith("ARM()"):
                res = self.call_arm_service(True)
                self.publish_log(f"ACTION RESULT (ARM): {res}")
            elif action_call.startswith("DISARM()"):
                res = self.call_arm_service(False)
                self.publish_log(f"ACTION RESULT (DISARM): {res}")
            elif action_call.startswith("TAKEOFF"):
                alt = float(action_call.split("(")[1].split(")")[0].strip())
                self.navigate_to(self.curr_x, self.curr_y, alt)
                self.publish_log(f"ACTION RESULT: Ascending to height {alt}m")
            elif action_call.startswith("NAVIGATE_TO"):
                args = action_call.split("(")[1].split(")")[0].split(",")
                x = float(args[0].strip())
                y = float(args[1].strip())
                z = float(args[2].strip())
                self.navigate_to(x, y, z)
                self.publish_log(f"ACTION RESULT: Directing autopilot to X={x}, Y={y}, Z={z}")
            elif action_call.startswith("LAND()"):
                res = self.call_land_service()
                self.publish_log(f"ACTION RESULT (LAND): {res}")
            elif action_call.startswith("WAIT()"):
                self.publish_log("ACTION RESULT: Hovering to stabilize state.")
            elif action_call.startswith("MISSION_COMPLETE()"):
                self.publish_log("ACTION RESULT: Agent declared mission successfully completed.")
            else:
                self.publish_log(f"ACTION RESULT: Command '{action_call}' not recognized.")
        except Exception as e:
            self.publish_log(f"ERROR: Action execution syntax parsing error: {e}")

    def run_fallback_agent(self, user_cmd: str):
        # Standard rule-based fallback simulating ReAct decisions
        cmd = user_cmd.lower()
        self.publish_log("FALLBACK: Processing command using deterministic rule engine...")

        if "arm" in cmd:
            self.publish_log("AGENT_THOUGHT: Requesting arming sequence to start motor ESCs.")
            res = self.call_arm_service(True)
            self.publish_log(f"ACTION RESULT: {res}")
        elif "takeoff" in cmd:
            self.publish_log("AGENT_THOUGHT: Safe takeoff request. Ascending vertically to 5.0m.")
            self.navigate_to(self.curr_x, self.curr_y, 5.0)
        elif "land" in cmd:
            self.publish_log("AGENT_THOUGHT: Land request. Triggering PX4 landing service.")
            res = self.call_land_service()
            self.publish_log(f"ACTION RESULT: {res}")
        elif "search" in cmd or "navigate" in cmd or "fly" in cmd:
            # Parse simple coordinates or fly to a pre-defined target location
            # Locate Lost Thermal Pad at (75, 80)
            if "pad" in cmd or "thermal" in cmd:
                self.publish_log("AGENT_THOUGHT: Directing drone to coordinates of the Lost Thermal Pad (75, 80, 5.0).")
                self.navigate_to(75.0, 80.0, 5.0)
            else:
                self.publish_log("AGENT_THOUGHT: Directing drone to default coordinate (30, 30, 5.0).")
                self.navigate_to(30.0, 30.0, 5.0)
        else:
            self.publish_log("AGENT_THOUGHT: Query does not match active flight actions. Keeping hover position.")
            self.navigate_to(self.curr_x, self.curr_y, self.curr_z)

def main(args=None):
    rclpy.init(args=args)
    node = AgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
