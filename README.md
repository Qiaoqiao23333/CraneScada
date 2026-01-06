# Crane SCADA System

A modern industrial SCADA (Supervisory Control and Data Acquisition) system for crane control and monitoring, built with ROS2, FastAPI, and React.

## Features

### 🎛️ Real-time Monitoring
- **Joint Telemetry**: Real-time position, velocity, and current monitoring for slewing, trolley, and hook joints
- **CAN Bus Monitoring**: Live CAN frame analysis with frame ID, DLC, payload, and timing statistics
- **System Resources**: CPU usage, bus voltage, motor temperature, and hook load monitoring
- **ROS2 Logs**: Real-time ROS2 node log streaming with filtering and analysis

### 🔍 ROS2 Network Discovery
- **Auto-Discovery**: Automatic detection of ROS2 services, topics, and actions
- **Network Topology**: Real-time visualization of ROS2 network interfaces
- **Service Management**: Interactive service calling with UI hints (toggle, button, form)

### 🎨 Modern UI
- **Responsive Design**: Works on desktop and mobile devices
- **High-Tech Aesthetic**: Dark theme with industrial SCADA styling
- **Real-time Charts**: Live telemetry visualization with multiple data series
- **Resizable Panels**: Customizable dashboard layout

### 🤖 Control & Safety
- **Motor Control**: Manual control interface for individual joints
- **E-STOP**: Emergency stop functionality
- **Action Execution**: Monitor and control ROS2 actions

## Architecture

```
┌─────────────────┐         ┌──────────────────┐         ┌─────────────────┐
│   React Frontend │ ◄─────► │  FastAPI Backend │ ◄─────► │   ROS2 Network  │
│   (Vite + React) │  HTTP   │   (Python 3.10)  │  rclpy  │   (Humble)      │
└─────────────────┘  WebSocket └──────────────────┘         └─────────────────┘
                                                                    │
                                                                    ▼
                                                          ┌─────────────────┐
                                                          │   CAN Bus       │
                                                          │   (socketcan)   │
                                                          └─────────────────┘
```

## Prerequisites

- **ROS2 Humble** (or compatible version)
- **Python 3.10+**
- **Node.js 18+** and npm
- **CAN Bus Interface** (optional, falls back to simulation mode)

## Installation

### 1. Clone the Repository

```bash
git clone <repository-url>
cd crane_scada
```

### 2. Backend Setup

```bash
cd backend

# Create virtual environment
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

### 3. Frontend Setup

```bash
cd ../frontend

# Install dependencies
npm install
```

### 4. ROS2 Environment

Make sure ROS2 is sourced in your environment:

```bash
source /opt/ros/humble/setup.bash  # Adjust path if needed
```

## Configuration

### Environment Variables

Backend supports the following environment variables:

- `CAN_INTERFACE`: CAN bus interface name (default: `can0`)
- `CAN_BITRATE`: CAN bus bitrate (default: `500000`)

Example:
```bash
export CAN_INTERFACE=can0
export CAN_BITRATE=500000
```

## Running the Application

### 1. Start the Backend

```bash
cd backend
source venv/bin/activate
source /opt/ros/humble/setup.bash  # Source ROS2
(export ROS_DOMAIN_ID=8)
python app.py
```

The backend will start on `http://localhost:8000`

### 2. Start the Frontend

In a new terminal:

```bash
cd frontend
npm run dev
```

The frontend will start on `http://localhost:5173` (or another port if 5173 is busy)

### 3. Access the Dashboard

Open your browser and navigate to:
```
http://localhost:5173
```

## API Endpoints

### WebSocket
- `ws://localhost:8000/ws` - Real-time telemetry stream (20Hz)

### REST API
- `GET /api/ros/interfaces` - Get ROS2 services, topics, and actions
- `POST /api/control/motor` - Control motor commands
- `POST /api/control/estop` - Emergency stop

## Project Structure

```
crane_scada/
├── backend/
│   ├── app.py              # FastAPI application
│   ├── ros_engine.py       # ROS2 node and CAN bus interface
│   └── requirements.txt    # Python dependencies
├── frontend/
│   ├── src/
│   │   ├── App.jsx         # Main React application
│   │   ├── main.jsx        # React entry point
│   │   └── index.css       # Global styles
│   ├── package.json        # Node.js dependencies
│   └── vite.config.js      # Vite configuration
└── README.md
```

## Key Components

### Backend (`ros_engine.py`)

- **IndustrialRobotNode**: ROS2 node that bridges ROS2 network with the web interface
  - Subscribes to `/joint_states` and `/rosout`
  - Auto-discovers ROS2 services, topics, and actions
  - Monitors CAN bus (with fallback to simulation)
  - Provides system statistics

### Frontend (`App.jsx`)

- **Dashboard**: Main monitoring interface with resizable panels
- **Telemetry Charts**: Real-time position, velocity, and current visualization
- **CAN Monitor**: CAN frame analysis table
- **ROS2 Network Panel**: Auto-discovered interfaces viewer
- **Motor Control**: Manual joint control interface

## ROS2 Integration

The system automatically discovers:

- **Services**: Using `get_service_names_and_types()`
- **Topics**: Using `get_topic_names_and_types()`
- **Actions**: Using `get_action_names_and_types()` from `rclpy.action.graph`

Discovery runs at 1Hz and updates the frontend every 5 seconds.

## CAN Bus Support

The system supports:
- Real CAN bus interfaces (socketcan)
- Virtual CAN (vcan0) for testing
- Simulation mode (when no CAN hardware is available)

CAN frames are displayed with:
- Frame ID (hex)
- DLC (Data Length Code)
- Payload (hex)
- Delta time (ms)
- Frame count

## Development

### Backend Development

```bash
cd backend
source venv/bin/activate
# Make changes to Python files
# Restart the server
```

### Frontend Development

```bash
cd frontend
npm run dev
# Hot reload is enabled
```

### Building for Production

```bash
cd frontend
npm run build
# Output will be in frontend/dist/
```

## Troubleshooting

### ROS2 Topics Not Showing

- Ensure ROS2 is properly sourced
- Check that other ROS2 nodes are running and publishing topics
- Verify ROS2 discovery is working: `ros2 topic list`
- Check backend logs for errors

### CAN Bus Not Working

- Verify CAN interface exists: `ip link show can0`
- Check permissions (may need sudo)
- System will automatically fall back to simulation mode

### Frontend Not Connecting

- Verify backend is running on port 8000
- Check browser console for WebSocket errors
- Ensure CORS is properly configured (default allows all origins)

## License

[Add your license here]

## Contributing

[Add contribution guidelines here]

## Authors

[Add author information here]

