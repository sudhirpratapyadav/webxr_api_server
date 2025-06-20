# WebXR API Server

A simple server that captures WebXR data from mobile devices and makes it available through an API.

## Quick Setup

1. Install required packages:
   ```bash
   pip install -r requirements.txt
   ```

2. Set up HTTPS (required for WebXR):
   ```bash
   ./setup_https.sh
   ```

3. Run the server:
   ```bash
   python run_https_server.py
   ```

## Access URLs

Once the server is running, you can access:

- Main interface: `https://<your-ip>:8443/static/index.html`
- WebXR test page: `https://<your-ip>:8443/static/webxr_test.html`

The server will display your IP address when it starts. For example:
```
Starting HTTPS server on 192.168.31.22:8443
Access URLs:
- Local PC: https://192.168.31.22:8443/static/index.html
- Mobile:   https://192.168.31.22:8443/static/index.html
- WebXR Test: https://192.168.31.22:8443/static/webxr_test.html
```

## Important Notes

- You will see security warnings about the self-signed certificate in your browser
- Click "Advanced" and then "Proceed" to continue
- For mobile devices, you need to first open the URL in your browser and accept the security warning
- Both your computer and mobile device must be on the same network

## Testing WebXR Support

To check if your device supports WebXR:
1. Visit https://immersive-web.github.io/webxr-samples/ 
2. Or use the included test page at `https://<your-ip>:8443/static/webxr_test.html`

## Requirements

- Python 3.7+
- Mobile device with WebXR support
- Both server and client on the same network

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

## Repository Structure

```
.
├── README.md               # This documentation file
├── requirements.txt        # Python dependencies
├── run_https_server.py     # Script to start HTTPS server
├── server.py               # Main FastAPI server implementation
├── setup_https.sh          # Script to generate self-signed certificates
├── certs/                  # Directory for SSL certificates
│   ├── cert.pem            # Self-signed certificate
│   └── key.pem             # Private key for the certificate
└── static/                 # Static web files
    ├── index.html          # Main web interface
    └── teleop_client.js    # WebXR client JavaScript code
```