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
    print(f"- Main App: https://{local_ip}:{args.port}/static/index.html")
    print(f"- WebXR Test: https://{local_ip}:{args.port}/static/webxr_test.html")
    print("="*50)
    print("NOTE: You will see security warnings about the self-signed certificate.")
    print("Press Ctrl+C once to gracefully shut down the server.")
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
