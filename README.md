# WebXR Teleop

A real-time application that uses FastAPI and WebSockets to transmit 6 DOF (degrees of freedom) pose data from a WebXR-enabled browser to a Python server.

## Features

- FastAPI backend with WebSocket support
- WebXR client for VR pose tracking
- Real-time data transmission using WebSockets
- Simple web interface for connection management
- Pose data visualization

## Requirements

- Python 3.7+
- WebXR-compatible browser (Chrome, Edge, Firefox with VR headset)
- VR headset (for WebXR functionality)
- Mobile device with WebXR support for mobile usage

## Installation

1. Clone this repository
2. Install the required packages:

```bash
pip install -r requirements.txt
```

## Usage

1. Run the server:

```bash
./run_server.sh
```

Or manually with:

```bash
uvicorn server:app --host 0.0.0.0 --port 8000 --reload
```

2. Find your computer's local IP address on your WiFi network:
   ```bash
   ip addr show | grep "inet " | grep -v 127.0.0.1
   ```
   This will show your local IP address (e.g., 192.168.1.100)

3. On your PC or mobile device, open a browser and navigate to:
   ```
   http://<your-local-ip>:8000/static/index.html
   ```
   (Replace <your-local-ip> with the IP address from step 2)

### HTTPS Support (Required for WebXR)

**Important:** WebXR requires HTTPS except on localhost. If you're having issues with WebXR on mobile, you should use the HTTPS server:

1. Set up HTTPS with our provided script:
   ```bash
   ./setup_https.sh
   ```

2. Run the HTTPS server:
   ```bash
   python run_https_server.py
   ```

3. Access your application at:
   ```
   https://<your-local-ip>:8443/static/index.html
   ```

4. You will see a security warning about the self-signed certificate. Click "Advanced" and then "Proceed" to continue.

### Test WebXR Support

To check if your mobile device properly supports WebXR:

1. Access the test page:
   ```
   https://<your-local-ip>:8443/static/webxr_test.html
   ```

2. This minimal test page will show you if WebXR AR/VR is supported on your device.

## Mobile Usage

For using the application on a mobile device:

1. Ensure your mobile device supports WebXR (recent Android devices with Chrome or Samsung Internet)
2. Both your computer (server) and mobile device must be on the same WiFi network
3. Use your computer's local IP address to access the application (e.g., https://192.168.1.100:8443/static/index.html)
4. For AR support on mobile, the application will use the device's position and orientation sensors

## API Endpoints

- `GET /` - Basic server status
- `GET /latest_pose` - Retrieve the latest pose data received from any client
- `WebSocket /ws` - WebSocket endpoint for real-time pose data transmission

## Data Format

The pose data is transmitted as JSON with the following structure:

```json
{
  "timestamp": "2025-06-20T12:34:56.789Z",
  "position": {
    "x": "0.0000",
    "y": "1.6000",
    "z": "0.0000"
  },
  "orientation": {
    "x": "0.0000",
    "y": "0.0000",
    "z": "0.0000",
    "w": "1.0000"
  }
}
```

## Notes on WebXR

- WebXR requires HTTPS in production environments
- For local development, browsers may allow WebXR over HTTP on localhost
- Some browsers require enabling WebXR features in settings
