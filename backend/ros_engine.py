import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rcl_interfaces.msg import Log
import threading
import time
import psutil
import random # Fallback for CAN simulation if hardware missing

class IndustrialRobotNode(Node):
    def __init__(self):
        super().__init__('crane_scada_bridge')
        self._lock = threading.Lock()
        
        # --- System State (The Process Image) ---
        self._state = {
            "timestamp": 0,
            "joints": {
                "slewing": {"pos": 0.0, "vel": 0.0, "cur": 0.0},
                "trolley": {"pos": 0.0, "vel": 0.0, "cur": 0.0},
                "hook":    {"pos": 0.0, "vel": 0.0, "cur": 0.0},
            },
            "can_frames": [], # Ring buffer
            "logs": [],       # Ring buffer
            "system_stats": {"cpu": 0, "voltage": 48.0},
            "network_topology": { "services": [], "topics": [], "actions": [] }
        }

        # --- Subscribers ---
        self.create_subscription(JointState, '/joint_states', self._joint_callback, 10)
        self.create_subscription(Log, '/rosout', self._log_callback, 10)

        # --- Timers ---
        self.create_timer(1.0, self._introspection_loop) # 1Hz Topology Scan
        self.create_timer(0.05, self._can_bus_loop)      # 20Hz CAN Reading

        self.get_logger().info("CraneSCADA Bridge Initialized")

    def _joint_callback(self, msg):
        # Map ROS joint array to named dictionary
        with self._lock:
            # Simplified mapping logic for demo
            if len(msg.position) >= 3:
                self._state["joints"]["slewing"] = {"pos": msg.position[0], "vel": msg.velocity[0], "cur": msg.effort[0]}
                self._state["joints"]["trolley"] = {"pos": msg.position[1], "vel": msg.velocity[1], "cur": msg.effort[1]}
                self._state["joints"]["hook"]    = {"pos": msg.position[2], "vel": msg.velocity[2], "cur": msg.effort[2]}

    def _log_callback(self, msg):
        entry = {
            "time": time.strftime("%H:%M:%S"),
            "level": {10:'DEBUG', 20:'INFO', 30:'WARN', 40:'ERROR', 50:'FATAL'}.get(msg.level, 'INFO'),
            "node": msg.name,
            "msg": msg.msg
        }
        with self._lock:
            self._state["logs"].append(entry)
            if len(self._state["logs"]) > 50: self._state["logs"].pop(0)

    def _can_bus_loop(self):
        # In a real system, read from SocketCAN here.
        # Simulating CAN traffic for the dashboard if no hardware present.
        if random.random() > 0.7:
            frame = {
                "id": random.choice([0x601, 0x181, 0x281, 0x081]),
                "timestamp": time.time() * 1000,
                "dlc": 8,
                "data": [random.randint(0, 255) for _ in range(8)],
                "isError": random.random() > 0.99
            }
            with self._lock:
                self._state["can_frames"].insert(0, frame)
                if len(self._state["can_frames"]) > 50: self._state["can_frames"].pop()

    def _introspection_loop(self):
        # Semantic Analysis of ROS Graph
        services = []
        for name, types in self.get_service_names_and_types():
            ui_hint = "form"
            if 'SetBool' in types[0]: ui_hint = "toggle"
            elif 'Trigger' in types[0]: ui_hint = "button"
            services.append({"name": name, "type": types[0], "ui_hint": ui_hint})

        with self._lock:
            self._state["network_topology"]["services"] = services
            self._state["system_stats"]["cpu"] = psutil.cpu_percent()
            self._state["timestamp"] = time.time() * 1000

    def get_snapshot(self):
        with self._lock:
            return self._state.copy()
