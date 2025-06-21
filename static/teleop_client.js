// DOM Elements
const connectBtn = document.getElementById('connect-btn');
const startXRBtn = document.getElementById('start-xr-btn');
const connectionStatus = document.getElementById('connection-status');
const connectionIndicator = document.getElementById('connection-indicator');

// Global variables
let socket = null;
let xrSession = null;
let referenceSpace = null;
let animationFrameId = null;

// Check WebXR support
function checkXRSupport() {
    // Display HTTPS warning if needed
    if (window.location.protocol !== 'https:' && 
        window.location.hostname !== 'localhost' && 
        !window.location.hostname.startsWith('192.168.') && 
        !window.location.hostname.startsWith('10.') &&
        !window.location.hostname.match(/^127\.\d+\.\d+\.\d+$/)) {
        console.warn('WebXR typically requires HTTPS (except on localhost)');
    }
    
    if ('xr' in navigator) {
        console.log('WebXR is available in navigator');
        
        // Check for AR support
        navigator.xr.isSessionSupported('immersive-ar')
            .then((supported) => {
                if (supported) {
                    console.log('✅ WebXR immersive-ar supported');
                    // Only enable the Start AR button if we're connected to the server
                    startXRBtn.disabled = !(socket && socket.readyState === WebSocket.OPEN);
                    startXRBtn.textContent = 'Start AR Session';
                } else {
                    console.log('❌ AR not supported on this device');
                    startXRBtn.disabled = true;
                }
            })
            .catch(err => {
                console.error('Error checking WebXR support:', err);
            });
    } else {
        console.log('WebXR not supported by this browser');
        startXRBtn.disabled = true;
    }
}

// Connect to WebSocket server
function connectToServer() {
    // Get the current location to build the WebSocket URL dynamically
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const host = window.location.host;
    const wsUrl = `${protocol}//${host}/ws`;
    
    console.log(`Connecting to WebSocket at: ${wsUrl}`);
    socket = new WebSocket(wsUrl);
    
    socket.onopen = () => {
        connectionStatus.textContent = 'WebSocket: Connected';
        connectionIndicator.classList.add('connected');
        connectionIndicator.classList.add('pulse');
        document.getElementById('connection-card').classList.add('connected');
        connectBtn.textContent = 'Disconnect';
        connectBtn.classList.add('connected');
        // Check WebXR support and enable the button if supported
        checkXRSupport();
    };
    
    socket.onclose = () => {
        connectionStatus.textContent = 'WebSocket: Disconnected';
        connectionIndicator.classList.remove('connected');
        connectionIndicator.classList.remove('pulse');
        document.getElementById('connection-card').classList.remove('connected');
        connectBtn.textContent = 'Connect to Server';
        connectBtn.classList.remove('connected');
        startXRBtn.disabled = true;
        
        // If XR session is active, stop it
        if (xrSession) {
            stopXRSession();
        }
    };
    
    socket.onerror = (error) => {
        console.error('WebSocket error:', error);
        connectionStatus.textContent = 'WebSocket: Error';
    };
}

// Disconnect from WebSocket server
function disconnectFromServer() {
    if (socket && socket.readyState === WebSocket.OPEN) {
        socket.close();
    }
}

// Toggle WebSocket connection
function toggleConnection() {
    if (!socket || socket.readyState === WebSocket.CLOSED || socket.readyState === WebSocket.CLOSING) {
        connectToServer();
    } else {
        disconnectFromServer();
    }
}

// Start WebXR session
async function startXRSession() {
    try {
        if ('xr' in navigator) {
            console.log('Starting AR session...');
            
            // Create a DOM overlay container for AR mode
            const overlayElement = document.createElement('div');
            overlayElement.id = 'xr-overlay';
            overlayElement.style.width = '100%';
            overlayElement.style.height = '100%';
            overlayElement.style.position = 'fixed';
            overlayElement.style.top = '0';
            overlayElement.style.left = '0';
            overlayElement.style.zIndex = '999999';
            overlayElement.style.pointerEvents = 'none'; // Allow clicks to pass through except for our button
            document.body.appendChild(overlayElement);
            
            // Create an exit button in the overlay
            const overlayExitBtn = document.createElement('button');
            overlayExitBtn.innerText = 'Exit';
            overlayExitBtn.style.position = 'absolute';
            overlayExitBtn.style.bottom = '30px';
            overlayExitBtn.style.left = '50%';
            overlayExitBtn.style.transform = 'translateX(-50%)';
            overlayExitBtn.style.padding = '16px 30px';
            overlayExitBtn.style.backgroundColor = 'rgba(0, 0, 0, 0.7)';
            overlayExitBtn.style.color = 'white';
            overlayExitBtn.style.border = 'none';
            overlayExitBtn.style.borderRadius = '30px';
            overlayExitBtn.style.fontSize = '18px';
            overlayExitBtn.style.fontWeight = 'bold';
            overlayExitBtn.style.boxShadow = '0 4px 8px rgba(0, 0, 0, 0.3)';
            overlayExitBtn.style.width = 'auto'; // Only as wide as the content
            overlayExitBtn.style.minWidth = 'unset'; // Override any min-width
            overlayExitBtn.style.maxWidth = 'unset'; // Override any max-width
            overlayExitBtn.style.alignItems = 'center'; // Center text vertically
            overlayExitBtn.style.justifyContent = 'center'; // Center text horizontally
            overlayExitBtn.style.pointerEvents = 'auto'; // Make sure the button can be clicked
            
            // Override inherited styles from the page
            overlayExitBtn.style.flex = 'none';
            overlayExitBtn.id = 'xr-exit-btn'; // Give it a unique ID
            
            // Add click event before appending to DOM
            overlayExitBtn.addEventListener('click', () => {
                console.log('Exit button clicked');
                if (xrSession) {
                    xrSession.end();
                }
            });
            
            overlayElement.appendChild(overlayExitBtn);
            
            // Set up AR session
            let sessionOptions = {
                requiredFeatures: ['local']
            };
            
            // Add DOM Overlay as optional feature
            sessionOptions.optionalFeatures = ['dom-overlay'];
            sessionOptions.domOverlay = { root: overlayElement };
            console.log('DOM Overlay configuration:', sessionOptions);
            
            // Request AR session
            xrSession = await navigator.xr.requestSession('immersive-ar', sessionOptions);
            console.log('AR session started with DOM overlay');
            
            // Set up session events
            xrSession.addEventListener('end', onXRSessionEnded);
            
            // Get the canvas for WebXR rendering (required even if we don't render anything)
            const canvas = document.createElement('canvas');
            document.body.appendChild(canvas);
            canvas.style.display = 'none'; // Hide the canvas as we don't need to see it
            
            // Get WebGL context
            const gl = canvas.getContext('webgl', { xrCompatible: true });
            if (!gl) {
                throw new Error('WebGL not supported');
            }
            
            // Initialize WebGL for XR
            const xrGlLayer = new XRWebGLLayer(xrSession, gl);
            await xrSession.updateRenderState({
                baseLayer: xrGlLayer
            });
            
            // Get reference space for pose tracking
            try {
                // First try local-floor for better tracking if available
                referenceSpace = await xrSession.requestReferenceSpace('local-floor');
                console.log('Using local-floor reference space');
            } catch (e) {
                // Fallback to local reference space
                console.log('local-floor not available, using local reference space');
                referenceSpace = await xrSession.requestReferenceSpace('local');
            }
            
            // Start the XR animation loop
            xrSession.requestAnimationFrame(onXRAnimationFrame);
            
            // Update UI
            startXRBtn.disabled = true;
        } else {
            console.error('WebXR not supported');
        }
    } catch (error) {
        console.error('Error starting AR session:', error);
    }
}



// XR Animation Frame callback
function onXRAnimationFrame(time, frame) {
    if (!xrSession) return;
    
    // Request the next frame
    animationFrameId = xrSession.requestAnimationFrame(onXRAnimationFrame);
    
    // Get the pose
    const pose = frame.getViewerPose(referenceSpace);
    
    if (pose) {
        // Extract position and orientation from pose
        const position = pose.transform.position;
        const orientation = pose.transform.orientation;
        
        // Create a pose data object
        const poseData = {
            timestamp: new Date().toISOString(),
            position: {
                x: position.x.toFixed(4),
                y: position.y.toFixed(4),
                z: position.z.toFixed(4)
            },
            orientation: {
                x: orientation.x.toFixed(4),
                y: orientation.y.toFixed(4),
                z: orientation.z.toFixed(4),
                w: orientation.w.toFixed(4)
            }
        };
        
        // Send the pose data via WebSocket if connected
        if (socket && socket.readyState === WebSocket.OPEN) {
            socket.send(JSON.stringify(poseData));
        }
    } else {
        console.log('No pose available in this frame');
    }
}

// Handle XR session end
function onXRSessionEnded() {
    console.log('WebXR session ended');
    
    // Clean up session variables
    xrSession = null;
    referenceSpace = null;
    
    // Remove the overlay if it exists
    const overlayElement = document.getElementById('xr-overlay');
    if (overlayElement) {
        document.body.removeChild(overlayElement);
    }
    
    // Update UI
    startXRBtn.disabled = false;
    
    // If there was a canvas created for the session, remove it
    const tempCanvas = document.querySelector('canvas[style*="display: none"]');
    if (tempCanvas) {
        tempCanvas.remove();
    }
}

// Stop XR session
function stopXRSession() {
    if (xrSession) {
        xrSession.end();
    }
}

// No need for a fallback close button - using DOM overlay exit button

// Event listeners
connectBtn.addEventListener('click', toggleConnection);
startXRBtn.addEventListener('click', startXRSession);

// Initialize
checkXRSupport();
