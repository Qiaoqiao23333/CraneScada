import uvicorn
import threading
import asyncio
import rclpy
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
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

@app.post("/api/control/motor")
async def control(cmd: ControlCmd):
    # In production: Use ros_node.create_client to call actual services
    print(f"Executing {cmd.command} on {cmd.joint_id}")
    return {"status": "ACK"}

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


