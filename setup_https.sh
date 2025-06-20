#!/bin/bash

# ANSI color codes
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== WebXR Teleop HTTPS Server Setup ===${NC}"

# Check if OpenSSL is installed
if ! command -v openssl &> /dev/null; then
    echo -e "${RED}OpenSSL is not installed. Please install it first.${NC}"
    exit 1
fi

# Create a certificates directory if it doesn't exist
mkdir -p certs
cd certs

echo -e "${BLUE}Generating self-signed certificate for HTTPS...${NC}"

# Generate a private key
openssl genrsa -out key.pem 2048

# Generate a self-signed certificate
openssl req -new -x509 -key key.pem -out cert.pem -days 365 -subj "/CN=localhost" -addext "subjectAltName = IP:127.0.0.1"

cd ..

echo -e "${GREEN}Certificate generated successfully!${NC}"

# Create or update a Python HTTPS server script
cat > run_https_server.py << 'EOF'
import uvicorn
import socket
import argparse
from pathlib import Path

def get_local_ip():
    """Get the local IP address of the machine"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # doesn't even have to be reachable
        s.connect(('8.8.8.8', 1))
        IP = s.getsockname()[0]
    except Exception:
        IP = '127.0.0.1'
    finally:
        s.close()
    return IP

def main():
    parser = argparse.ArgumentParser(description='Run FastAPI server with HTTPS')
    parser.add_argument('--host', default='0.0.0.0', help='Host to bind to')
    parser.add_argument('--port', type=int, default=8443, help='Port to bind to')
    args = parser.parse_args()

    # Check if certificates exist
    cert_path = Path('certs/cert.pem')
    key_path = Path('certs/key.pem')
    
    if not cert_path.exists() or not key_path.exists():
        print("Error: Certificate files not found. Run the setup script first.")
        return

    local_ip = get_local_ip()
    
    print("="*50)
    print(f"Starting HTTPS server on {local_ip}:{args.port}")
    print(f"Access URLs:")
    print(f"- Local PC: https://{local_ip}:{args.port}/static/index.html")
    print(f"- Mobile:   https://{local_ip}:{args.port}/static/index.html")
    print(f"- WebXR Test: https://{local_ip}:{args.port}/static/webxr_test.html")
    print("="*50)
    print("NOTE: You will see security warnings about the self-signed certificate.")
    print("You need to accept these warnings to proceed.")
    print("="*50)
    
    uvicorn.run(
        "server:app",
        host=args.host,
        port=args.port,
        reload=True,
        ssl_keyfile=str(key_path),
        ssl_certfile=str(cert_path)
    )

if __name__ == "__main__":
    main()
EOF

echo -e "${BLUE}Creating HTTPS server script...${NC}"
chmod +x run_https_server.py

echo -e "${GREEN}HTTPS setup complete!${NC}"
echo -e "${YELLOW}To run the server with HTTPS, use:${NC}"
echo -e "python run_https_server.py"
echo -e "${BLUE}----------------------------------------${NC}"
echo -e "${YELLOW}NOTE:${NC} Since this uses a self-signed certificate,"
echo -e "you will need to accept security warnings in your browser."
echo -e "${YELLOW}For mobile devices:${NC}"
echo -e "1. First open the HTTPS URL in your mobile browser"
echo -e "2. You'll see a security warning - click 'Advanced' and proceed anyway"
echo -e "3. This is only required for development purposes"
echo -e "${BLUE}----------------------------------------${NC}"

# Ask if the user wants to run the HTTPS server now
read -p "Do you want to run the HTTPS server now? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    python run_https_server.py
fi
