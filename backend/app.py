import uvicorn
import threading
import asyncio
import os
import shlex
import subprocess
import rclpy
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field
from ros_engine import IndustrialRobotNode

app = FastAPI()

app.add_middleware(
    CORSMiddleware, allow_origins=["*"], allow_credentials=True, allow_methods=["*"], allow_headers=["*"],
)

ros_node = None

def ros_spin_wrapper(node):
    """Custom ROS spin loop with error handling"""
    import time
    while rclpy.ok():
        try:
            rclpy.spin_once(node, timeout_sec=0.1)
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"ROS spin error (continuing): {e}")
            time.sleep(0.1)

@app.on_event("startup")
async def startup():
    global ros_node
    # Set ROS_DOMAIN_ID from environment or use default
    ros_domain_id = os.getenv('ROS_DOMAIN_ID', '0')
    os.environ['ROS_DOMAIN_ID'] = ros_domain_id
    print(f"Initializing ROS2 with ROS_DOMAIN_ID={ros_domain_id}")
    
    try:
        if not rclpy.ok():
            rclpy.init()
    except RuntimeError:
        # Already initialized
        pass
    ros_node = IndustrialRobotNode()
    # Run ROS spin in background thread with error handling
    t = threading.Thread(target=ros_spin_wrapper, args=(ros_node,), daemon=True)
    t.start()

@app.get("/api/ros/interfaces")
async def get_interfaces():
    if not ros_node: 
        return {"services": [], "topics": [], "actions": []}
    snapshot = ros_node.get_snapshot()
    network_topology = snapshot.get("network_topology", {"services": [], "topics": [], "actions": []})
    # Ensure all keys exist
    return {
        "services": network_topology.get("services", []),
        "topics": network_topology.get("topics", []),
        "actions": network_topology.get("actions", [])
    }

class ControlCmd(BaseModel):
    joint_id: str
    command: str
    value: float = 0.0


class ActionGoalCmd(BaseModel):
    action_name: str
    action_type: str
    joint_names: list[str] = Field(default_factory=list)
    positions: list[float] = Field(default_factory=list)
    time_from_start_sec: float = 2.0
    goal_yaml: str = ""
    feedback: bool = True


class ServiceCallCmd(BaseModel):
    service_name: str
    service_type: str
    request_yaml: str = ""
    bool_value: bool = True


class TopicPublishCmd(BaseModel):
    topic_name: str
    topic_type: str
    message_yaml: str = ""
    float_value: float = 0.0
    once: bool = True

@app.post("/api/control/motor")
async def control(cmd: ControlCmd):
    # In production: Use ros_node.create_client to call actual services
    print(f"Executing {cmd.command} on {cmd.joint_id}")
    return {"status": "ACK"}


def _format_duration_yaml(seconds_float: float):
    total_seconds = max(0.0, float(seconds_float))
    sec = int(total_seconds)
    nanosec = int(round((total_seconds - sec) * 1_000_000_000))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec = 0
    return sec, nanosec


def _build_follow_joint_trajectory_goal(cmd: ActionGoalCmd):
    if not cmd.joint_names:
        raise HTTPException(status_code=400, detail="At least one joint name is required")
    if not cmd.positions:
        raise HTTPException(status_code=400, detail="At least one position value is required")
    if len(cmd.joint_names) != len(cmd.positions):
        raise HTTPException(
            status_code=400,
            detail="Joint names and positions must have the same number of entries",
        )

    sec, nanosec = _format_duration_yaml(cmd.time_from_start_sec)
    joint_names = ", ".join(cmd.joint_names)
    positions = ", ".join(str(value) for value in cmd.positions)

    return (
        "trajectory:\n"
        f"  joint_names: [{joint_names}]\n"
        "  points:\n"
        f"  - positions: [{positions}]\n"
        "    time_from_start:\n"
        f"      sec: {sec}\n"
        f"      nanosec: {nanosec}\n"
    )


@app.post("/api/control/action-goal")
async def send_action_goal(cmd: ActionGoalCmd):
    action_name = cmd.action_name.strip()
    action_type = cmd.action_type.strip()
    if not action_name or not action_type:
        raise HTTPException(status_code=400, detail="Action name and type are required")

    if "FollowJointTrajectory" in action_type:
        goal_yaml = _build_follow_joint_trajectory_goal(cmd)
    else:
        goal_yaml = cmd.goal_yaml.strip()
        if not goal_yaml:
            raise HTTPException(
                status_code=400,
                detail="Goal YAML is required for this action type",
            )

    command = ["ros2", "action", "send_goal", action_name, action_type, goal_yaml]
    if cmd.feedback:
        command.append("--feedback")

    try:
        process = subprocess.Popen(
            command,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
    except FileNotFoundError as exc:
        raise HTTPException(
            status_code=500,
            detail="ros2 CLI was not found in the backend environment",
        ) from exc
    except Exception as exc:
        raise HTTPException(
            status_code=500,
            detail=f"Failed to launch ROS2 action goal: {exc}",
        ) from exc

    printable_command = " ".join(shlex.quote(part) for part in command)
    print(f"Launching ROS2 action goal: {printable_command}")
    return {
        "status": "LAUNCHED",
        "pid": process.pid,
        "command": printable_command,
    }


@app.post("/api/control/service-call")
async def call_service(cmd: ServiceCallCmd):
    service_name = cmd.service_name.strip()
    service_type = cmd.service_type.strip()
    if not service_name or not service_type:
        raise HTTPException(status_code=400, detail="Service name and type are required")

    if "SetBool" in service_type:
        request_yaml = f"data: {'true' if cmd.bool_value else 'false'}"
    else:
        request_yaml = cmd.request_yaml.strip()
        if not request_yaml:
            raise HTTPException(status_code=400, detail="Request YAML is required for this service type")

    command = ["ros2", "service", "call", service_name, service_type, request_yaml]

    try:
        process = subprocess.Popen(
            command,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
    except FileNotFoundError as exc:
        raise HTTPException(
            status_code=500,
            detail="ros2 CLI was not found in the backend environment",
        ) from exc
    except Exception as exc:
        raise HTTPException(
            status_code=500,
            detail=f"Failed to launch ROS2 service call: {exc}",
        ) from exc

    printable_command = " ".join(shlex.quote(part) for part in command)
    print(f"Launching ROS2 service call: {printable_command}")
    return {
        "status": "LAUNCHED",
        "pid": process.pid,
        "command": printable_command,
    }


@app.post("/api/control/topic-publish")
async def publish_topic(cmd: TopicPublishCmd):
    topic_name = cmd.topic_name.strip()
    topic_type = cmd.topic_type.strip()
    if not topic_name or not topic_type:
        raise HTTPException(status_code=400, detail="Topic name and type are required")

    if "std_msgs/msg/Float32" in topic_type:
        message_yaml = f"data: {cmd.float_value}"
    else:
        message_yaml = cmd.message_yaml.strip()
        if not message_yaml:
            raise HTTPException(status_code=400, detail="Message YAML is required for this topic type")

    command = ["ros2", "topic", "pub", topic_name, topic_type, message_yaml]
    if cmd.once:
        command.append("--once")

    try:
        process = subprocess.Popen(
            command,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
    except FileNotFoundError as exc:
        raise HTTPException(
            status_code=500,
            detail="ros2 CLI was not found in the backend environment",
        ) from exc
    except Exception as exc:
        raise HTTPException(
            status_code=500,
            detail=f"Failed to launch ROS2 topic publish: {exc}",
        ) from exc

    printable_command = " ".join(shlex.quote(part) for part in command)
    print(f"Launching ROS2 topic publish: {printable_command}")
    return {
        "status": "LAUNCHED",
        "pid": process.pid,
        "command": printable_command,
    }

@app.post("/api/control/estop")
async def estop():
    print("!!! E-STOP TRIGGERED !!!")
    return {"status": "HALTED"}

def clean_nan(data):
    """Recursively replace NaN and Inf values with None for JSON serialization"""
    import math
    if isinstance(data, dict):
        return {k: clean_nan(v) for k, v in data.items()}
    elif isinstance(data, list):
        return [clean_nan(item) for item in data]
    elif isinstance(data, float):
        if math.isnan(data) or math.isinf(data):
            return 0.0  # Replace NaN/Inf with 0.0
        return data
    return data

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            if ros_node:
                try:
                    snapshot = ros_node.get_snapshot()
                    clean_data = clean_nan(snapshot)
                    await websocket.send_json(clean_data)
                except Exception as e:
                    print(f"Error sending WebSocket data: {e}")
                    break
            else:
                # Send empty state if ros_node not initialized
                await websocket.send_json({"error": "ROS node not initialized"})
            await asyncio.sleep(0.05) # 20Hz UI Update Rate
    except WebSocketDisconnect:
        pass
    except Exception as e:
        print(f"WebSocket error: {e}")

if __name__ == "__main__":
    # Ensure you source ROS2 before running this!
    uvicorn.run(app, host="0.0.0.0", port=8000)


