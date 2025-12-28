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

@app.on_event("startup")
async def startup():
    global ros_node
    rclpy.init()
    ros_node = IndustrialRobotNode()
    # Run ROS spin in background thread
    t = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    t.start()

@app.get("/api/ros/interfaces")
async def get_interfaces():
    if not ros_node: return {}
    return ros_node.get_snapshot()["network_topology"]

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

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            if ros_node:
                await websocket.send_json(ros_node.get_snapshot())
            await asyncio.sleep(0.05) # 20Hz UI Update Rate
    except WebSocketDisconnect:
        pass

if __name__ == "__main__":
    # Ensure you source ROS2 before running this!
    uvicorn.run(app, host="0.0.0.0", port=8000)


