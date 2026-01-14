import rclpy
from rclpy.node import Node
from rclpy.action.graph import get_action_names_and_types
from std_msgs.msg import Float64
from rcl_interfaces.msg import Log
import threading
import time
import psutil
import random # Fallback for CAN simulation if hardware missing
import os
import copy

# Optional CAN library import
try:
    import can
    CAN_AVAILABLE = True
except ImportError:
    CAN_AVAILABLE = False
    can = None

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

        # --- CAN Bus Interface ---
        self._can_bus = None
        self._can_interface = os.getenv('CAN_INTERFACE', 'can0')  # Default to can0
        self._can_bitrate = int(os.getenv('CAN_BITRATE', '500000'))  # Default 500kbps
        self._can_simulation_mode = False
        
        # Try to initialize real CAN bus (only if library is available)
        if CAN_AVAILABLE:
            try:
                self._can_bus = can.interface.Bus(
                    channel=self._can_interface,
                    bustype='socketcan',
                    bitrate=self._can_bitrate
                )
                self.get_logger().info(f"CAN bus initialized on {self._can_interface} at {self._can_bitrate} bps")
            except (OSError, can.CanError, ValueError) as e:
                self.get_logger().warn(f"CAN hardware {self._can_interface} not available ({e}), trying vcan0...")
                # Try virtual CAN interface as fallback
                try:
                    self._can_bus = can.interface.Bus(
                        channel='vcan0',
                        bustype='socketcan'
                    )
                    self._can_interface = 'vcan0'
                    self.get_logger().info(f"CAN bus initialized on vcan0 (virtual interface)")
                except (OSError, can.CanError, ValueError) as e2:
                    self.get_logger().warn(f"vcan0 also not available ({e2}), falling back to data simulation mode")
                    self._can_simulation_mode = True
                    self._can_bus = None
        else:
            self.get_logger().warn("python-can library not installed, using simulation mode")
            self._can_simulation_mode = True
            self._can_bus = None

        # --- Subscribers ---
        # Position topics
        self.get_logger().info("Subscribing to position topics:")
        self.create_subscription(Float64, '/hoist/crane_position', self._hoist_position_callback, 10)
        self.get_logger().info("  - /hoist/crane_position")
        self.create_subscription(Float64, '/trolley/crane_position', self._trolley_position_callback, 10)
        self.get_logger().info("  - /trolley/crane_position")
        self.create_subscription(Float64, '/slewing/crane_position', self._slewing_position_callback, 10)
        self.get_logger().info("  - /slewing/crane_position")
        
        # Velocity topics
        self.get_logger().info("Subscribing to velocity topics:")
        self.create_subscription(Float64, '/hoist/crane_velocity', self._hoist_velocity_callback, 10)
        self.get_logger().info("  - /hoist/crane_velocity")
        self.create_subscription(Float64, '/trolley/crane_velocity', self._trolley_velocity_callback, 10)
        self.get_logger().info("  - /trolley/crane_velocity")
        self.create_subscription(Float64, '/slewing/crane_velocity', self._slewing_velocity_callback, 10)
        self.get_logger().info("  - /slewing/crane_velocity")
        
        self.create_subscription(Log, '/rosout', self._log_callback, 10)

        # --- Timers ---
        self.create_timer(1.0, self._introspection_loop) # 1Hz Topology Scan
        self.create_timer(0.05, self._can_bus_loop)      # 20Hz CAN Reading

        self.get_logger().info("CraneSCADA Bridge Initialized")

    def _hoist_position_callback(self, msg):
        """Callback for /hoist/crane_position topic"""
        with self._lock:
            self._state["joints"]["hook"]["pos"] = msg.data
    
    def _trolley_position_callback(self, msg):
        """Callback for /trolley/crane_position topic"""
        with self._lock:
            self._state["joints"]["trolley"]["pos"] = msg.data
    
    def _slewing_position_callback(self, msg):
        """Callback for /slewing/crane_position topic"""
        with self._lock:
            self._state["joints"]["slewing"]["pos"] = msg.data
    
    def _hoist_velocity_callback(self, msg):
        """Callback for /hoist/crane_velocity topic"""
        with self._lock:
            self._state["joints"]["hook"]["vel"] = msg.data
    
    def _trolley_velocity_callback(self, msg):
        """Callback for /trolley/crane_velocity topic"""
        with self._lock:
            self._state["joints"]["trolley"]["vel"] = msg.data
    
    def _slewing_velocity_callback(self, msg):
        """Callback for /slewing/crane_velocity topic"""
        with self._lock:
            self._state["joints"]["slewing"]["vel"] = msg.data

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
        """Read CAN frames from SocketCAN or simulate if hardware unavailable"""
        if self._can_simulation_mode:
            # Fallback: Simulate CAN traffic for the dashboard if no hardware present
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
        else:
            # Real CAN bus: Read all available messages (non-blocking)
            if self._can_bus is not None:
                try:
                    # Read multiple messages if available (non-blocking)
                    while True:
                        msg = self._can_bus.recv(timeout=0.0)  # Non-blocking
                        if msg is None:
                            break
                        
                        frame = {
                            "id": msg.arbitration_id,
                            "timestamp": msg.timestamp * 1000 if hasattr(msg, 'timestamp') else time.time() * 1000,
                            "dlc": msg.dlc,
                            "data": list(msg.data),
                            "isError": msg.is_error_frame,
                            "isExtended": msg.is_extended_id,
                            "isRemote": msg.is_remote_frame
                        }
                        
                        with self._lock:
                            self._state["can_frames"].insert(0, frame)
                            if len(self._state["can_frames"]) > 50: self._state["can_frames"].pop()
                            
                except can.CanError as e:
                    self.get_logger().error(f"CAN bus error: {e}")
                    # On error, switch to simulation mode
                    try:
                        if self._can_bus:
                            self._can_bus.shutdown()
                    except:
                        pass
                    self._can_simulation_mode = True
                    self._can_bus = None

    def _introspection_loop(self):
        # Semantic Analysis of ROS Graph
        try:
            # Discover Services
            services = []
            service_list = self.get_service_names_and_types()
            self.get_logger().debug(f"Found {len(service_list)} services in ROS2 network")
            for name, types in service_list:
                ui_hint = "form"
                if 'SetBool' in types[0]: ui_hint = "toggle"
                elif 'Trigger' in types[0]: ui_hint = "button"
                services.append({"name": name, "type": types[0], "ui_hint": ui_hint})

            # Discover Topics
            topics = []
            try:
                topic_list = self.get_topic_names_and_types()
                self.get_logger().debug(f"Found {len(topic_list)} topics in ROS2 network")
                for name, types in topic_list:
                    topics.append({"name": name, "types": types})
            except Exception as e:
                self.get_logger().warn(f"Failed to get topics: {e}")

            # Discover Actions
            # Use ROS2's action graph API to directly get all actions
            actions = []
            try:
                action_list = get_action_names_and_types(self)
                self.get_logger().info(f"Found {len(action_list)} actions in ROS2 network")
                
                for name, types in action_list:
                    actions.append({
                        "name": name,
                        "types": types  # types is already a list
                    })
                
                if len(actions) > 0:
                    self.get_logger().debug(f"Action names: {[a['name'] for a in actions]}")
            except Exception as e:
                self.get_logger().error(f"Failed to discover actions: {e}", exc_info=True)

            with self._lock:
                self._state["network_topology"]["services"] = services
                self._state["network_topology"]["topics"] = topics
                self._state["network_topology"]["actions"] = actions
                self._state["system_stats"]["cpu"] = psutil.cpu_percent()
                self._state["timestamp"] = time.time() * 1000
                
            if len(services) > 0 or len(topics) > 0:
                self.get_logger().info(f"ROS2 Discovery: {len(services)} services, {len(topics)} topics, {len(actions)} actions")
        except Exception as e:
            self.get_logger().error(f"Error in introspection loop: {e}")

    def get_snapshot(self):
        with self._lock:
            return copy.deepcopy(self._state)
    
    def destroy_node(self):
        """Cleanup CAN bus before destroying node"""
        if self._can_bus is not None:
            try:
                self._can_bus.shutdown()
                self.get_logger().info("CAN bus closed")
            except Exception as e:
                self.get_logger().warn(f"Error closing CAN bus: {e}")
        super().destroy_node()
