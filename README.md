# WebXR API Server

A streamlined server that captures pose data (position and orientation) from mobile AR sessions (using WebXR api) and makes it available to server (PC) through WebSocket.

## Requirements

- Python 3.7+
- Mobile device with WebXR API support (most modern Android/iOS devices)
- Both server and client on the same network

## Testing WebXR Support

To check if your device supports WebXR:
1. Visit https://immersive-web.github.io/webxr-samples/ 
2. Or use the included test page at `https://<your-ip>:8443/static/webxr_test.html` (after running the server)

## Quick Setup

1. Clone the repository:
   ```bash
   git clone https://github.com/sudhirpratapyadav/webxr_api_server.git
   cd webxr_api_server
   ```

2. Install required packages:
   ```bash
   pip install -r requirements.txt
   ```

3. Set up HTTPS (required for WebXR):
   ```bash
   ./setup_https.sh
   ```

4. Run the server:
   ```bash
   python run_server.py
   ```

5. To stop the server gracefully:
   ```bash
   # Press Ctrl+C once and wait for "All clients disconnected" message
   ```

### Accessing the web app

Once the server is running, you can access:

- Main interface: `https://<your-ip>:8443/static/index.html`
- Visualization GUI: `https://<your-ip>:8443/static/gui.html`
- WebXR test page: `https://<your-ip>:8443/static/webxr_test.html`

The server will display your IP address when it starts. For example:
```
Starting HTTPS server on 192.168.31.22:8443
Access URLs:
- Main App: https://192.168.31.22:8443/static/index.html
- Visualization: https://192.168.31.22:8443/static/gui.html
- WebXR Test: https://192.168.31.22:8443/static/webxr_test.html
```

## Important Notes

- You will see security warnings about the self-signed certificate in your browser
- Click "Advanced" and then "Proceed" to continue
- For mobile devices, you need to first open the URL in your browser and accept the security warning
- Both your computer and mobile device must be on the same network

## Usage

1. Open the web app on your mobile device
2. Click "Connect to Server" to establish a WebSocket connection
3. Once connected, the "Start AR Session" button will become active
4. Click "Start AR Session" to begin capturing AR pose data
5. An "Exit" button will appear in the AR view to end the session

## GUI Visualization

The server includes a 3D visualization interface accessible at `https://<your-ip>:8443/static/gui.html`. This interface:

- Displays the WebXR pose data as a 3D cube with coordinate axes
- Updates in real-time via WebSocket connection
- Includes interactive camera controls (pan, zoom, rotate)
- Features an overlay panel showing precise position and orientation values
- Allows minimizing the UI panel to focus on the visualization

## Data Format

The pose data is transmitted as JSON with the following structure, containing position and orientation (quaternion):

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

## API endpoints

The server provides two APIs

1. **REST API**: Get the latest pose data by sending a GET request to `https://<your-ip>:8443/latest_pose`

2. **Server Info**: Check server status with `https://<your-ip>:8443/server-info`

## Architecture and Flow Diagrams

For detailed architecture, sequence, data flow, and error handling diagrams, see [diagrams.md](diagrams.md).

## Repository Structure

```
.
├── README.md               # This documentation file
├── diagrams.md             # Architecture and flow diagrams
├── requirements.txt        # Python dependencies
├── run_server.py           # Script to start HTTPS server
├── server.py               # Main FastAPI server implementation
├── setup_https.sh          # Script to generate self-signed certificates
├── certs/                  # Directory for SSL certificates
│   ├── cert.pem            # Self-signed certificate
│   └── key.pem             # Private key for the certificate
└── static/                 # Static web files
    ├── favicon.ico         # Favicon for the web app
    ├── gui.html            # 3D visualization interface
    ├── index.html          # Main web interface
    ├── teleop_client.js    # WebXR client JavaScript code
    └── webxr_test.html     # WebXR compatibility test page
```