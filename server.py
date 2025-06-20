import json
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Request
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse, RedirectResponse
from typing import Dict, List
import socket
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = FastAPI()

# Serve static files (HTML, JS, CSS)
app.mount("/static", StaticFiles(directory="static"), name="static")

# Store for connected clients
class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []
        self.latest_pose: Dict = {}
        self.connection_count: int = 0

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)
        self.connection_count += 1
        logger.info(f"New client connected. Total clients: {self.connection_count}")

    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)
            self.connection_count -= 1
            logger.info(f"Client disconnected. Total clients: {self.connection_count}")

    async def broadcast(self, message: Dict):
        for connection in self.active_connections:
            await connection.send_json(message)
    
    def update_pose(self, pose_data: Dict):
        self.latest_pose = pose_data

    def get_latest_pose(self) -> Dict:
        return self.latest_pose


manager = ConnectionManager()

# Get local IP address
def get_local_ip():
    try:
        # This creates a socket that doesn't actually connect to anything
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # This connects to a public IP, but doesn't send any data
        s.connect(("8.8.8.8", 80))
        # Get the local IP used for this "connection"
        local_ip = s.getsockname()[0]
        s.close()
        return local_ip
    except Exception as e:
        logger.error(f"Error getting local IP: {e}")
        return "127.0.0.1"


@app.get("/", response_class=HTMLResponse)
async def get(request: Request):
    # Redirect to the static HTML page
    return RedirectResponse(url="/static/index.html")


@app.get("/server-info")
async def server_info():
    local_ip = get_local_ip()
    return {
        "message": "WebXR Teleop Server",
        "local_ip": local_ip,
        "clients_connected": manager.connection_count,
        "server_status": "running"
    }


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    client_ip = websocket.client.host
    logger.info(f"WebSocket connection attempt from: {client_ip}")
    await manager.connect(websocket)
    try:
        while True:
            # Receive pose data from client
            data = await websocket.receive_text()
            pose_data = json.loads(data)
            
            # Check if it's from a mobile device
            is_mobile = pose_data.get("isMobile", False)
            device_type = "Mobile" if is_mobile else "VR/Desktop"
            
            # Update the latest pose
            manager.update_pose(pose_data)
            
            # Print pose data for debugging (including quaternion)
            if "position" in pose_data and "orientation" in pose_data:
                pos = pose_data["position"]
                ori = pose_data["orientation"]
                logger.info(f"Received {device_type} pose:")
                logger.info(f"  Position: x={pos['x']}, y={pos['y']}, z={pos['z']}")
                logger.info(f"  Quaternion: x={ori['x']}, y={ori['y']}, z={ori['z']}, w={ori['w']}")
            elif "position" in pose_data:
                pos = pose_data["position"]
                logger.info(f"Received {device_type} pose: x={pos['x']}, y={pos['y']}, z={pos['z']}")
            
            # Broadcast to all clients (optional, for multi-client scenarios)
            await manager.broadcast(pose_data)
            
    except WebSocketDisconnect:
        manager.disconnect(websocket)
        print("Client disconnected")


# Optional REST endpoint to get the latest pose
@app.get("/latest_pose")
async def get_latest_pose():
    return manager.get_latest_pose()
