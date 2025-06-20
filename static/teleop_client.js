// DOM Elements
const connectBtn = document.getElementById('connect-btn');
const startXRBtn = document.getElementById('start-xr-btn');
const stopXRBtn = document.getElementById('stop-xr-btn');
const connectionStatus = document.getElementById('connection-status');
const connectionIndicator = document.getElementById('connection-indicator');
const xrStatus = document.getElementById('xr-status');
const poseDisplay = document.getElementById('pose-display');

// Global variables
let socket = null;
let xrSession = null;
let referenceSpace = null;
let animationFrameId = null;

// Check WebXR support
function checkXRSupport() {
    // Clear any previous status
    xrStatus.textContent = 'Checking WebXR support...';
    
    // Display HTTPS warning if needed
    if (window.location.protocol !== 'https:' && 
        window.location.hostname !== 'localhost' && 
        !window.location.hostname.startsWith('192.168.') && 
        !window.location.hostname.startsWith('10.') &&
        !window.location.hostname.match(/^127\.\d+\.\d+\.\d+$/)) {
        xrStatus.textContent = 'WARNING: WebXR requires HTTPS except on localhost';
        console.warn('WebXR typically requires HTTPS (except on localhost)');
    }
    
    if ('xr' in navigator) {
        console.log('WebXR is available in navigator');
        
        // First check for AR support (preferred for mobile)
        navigator.xr.isSessionSupported('immersive-ar')
            .then((supported) => {
                if (supported) {
                    console.log('✅ WebXR immersive-ar supported');
                    xrStatus.textContent = 'WebXR: AR Supported (Mobile)';
                    startXRBtn.disabled = false;
                    startXRBtn.textContent = 'Start AR Session';
                    return { supported: true, mode: 'immersive-ar' };
                }
                return { supported: false };
            })
            .then((arResult) => {
                if (!arResult.supported) {
                    // If AR is not supported, check for VR
                    return navigator.xr.isSessionSupported('immersive-vr')
                        .then((vrSupported) => {
                            if (vrSupported) {
                                console.log('✅ WebXR immersive-vr supported');
                                xrStatus.textContent = 'WebXR: VR Supported';
                                startXRBtn.disabled = false;
                                startXRBtn.textContent = 'Start VR Session';
                                return { supported: true, mode: 'immersive-vr' };
                            }
                            return { supported: false };
                        });
                }
                return arResult;
            })
            .then((result) => {
                if (!result.supported) {
                    console.log('❌ Neither AR nor VR modes are supported');
                    // Neither AR nor VR is supported, check for device orientation
                    if (window.DeviceOrientationEvent) {
                        xrStatus.textContent = 'Mobile: Using Device Orientation';
                        startXRBtn.disabled = false;
                        startXRBtn.textContent = 'Start Device Tracking';
                    } else {
                        xrStatus.textContent = 'WebXR: Not supported on this device';
                        startXRBtn.disabled = true;
                    }
                }
            })
            .catch(err => {
                console.error('Error checking WebXR support:', err);
                xrStatus.textContent = `Error: ${err.message}`;
            });
    } else {
        // WebXR not available, check for device orientation as fallback
        if (window.DeviceOrientationEvent) {
            xrStatus.textContent = 'Mobile: Device Orientation Available';
            startXRBtn.disabled = false;
        } else {
            xrStatus.textContent = 'WebXR: Not supported by this browser';
        }
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
        connectBtn.textContent = 'Disconnect';
        startXRBtn.disabled = false;
    };
    
    socket.onclose = () => {
        connectionStatus.textContent = 'WebSocket: Disconnected';
        connectionIndicator.classList.remove('connected');
        connectBtn.textContent = 'Connect to Server';
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
            console.log('Starting WebXR session...');
            
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
            overlayExitBtn.innerText = 'Exit AR/VR';
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
            overlayExitBtn.style.pointerEvents = 'auto'; // Make sure the button can be clicked
            
            // Add click event before appending to DOM
            overlayExitBtn.addEventListener('click', () => {
                console.log('Exit button clicked');
                if (xrSession) {
                    xrSession.end();
                }
            });
            
            overlayElement.appendChild(overlayExitBtn);
            
            // First try AR (for mobile) then fallback to VR
            let sessionMode = 'immersive-ar';
            let sessionOptions = {
                requiredFeatures: ['local']
            };
            
            try {
                // Check if AR is supported before requesting
                const arSupported = await navigator.xr.isSessionSupported('immersive-ar');
                if (arSupported) {
                    console.log('Requesting immersive-ar session');
                    
                    // Add DOM Overlay as optional feature
                    sessionOptions.optionalFeatures = ['dom-overlay'];
                    sessionOptions.domOverlay = { root: overlayElement };
                    console.log('DOM Overlay configuration:', sessionOptions);
                    
                    xrSession = await navigator.xr.requestSession(sessionMode, sessionOptions);
                    xrStatus.textContent = 'WebXR: AR Session Active';
                    console.log('AR session started with DOM overlay');
                } else {
                    // AR not supported, try VR
                    console.log('AR not supported, trying VR');
                    const vrSupported = await navigator.xr.isSessionSupported('immersive-vr');
                    if (vrSupported) {
                        sessionMode = 'immersive-vr';
                        sessionOptions = {
                            requiredFeatures: ['local-floor']
                        };
                        console.log('Requesting immersive-vr session');
                        xrSession = await navigator.xr.requestSession(sessionMode, sessionOptions);
                        xrStatus.textContent = 'WebXR: VR Session Active';
                    } else {
                        throw new Error('Neither AR nor VR supported');
                    }
                }
            } catch (err) {
                console.error('Error requesting WebXR session:', err);
                // Fall back to device orientation for mobile
                startDeviceOrientationTracking();
                if (overlayElement) {
                    document.body.removeChild(overlayElement);
                }
                return;
            }
            
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
            
            // Create a standalone floating close button as fallback
            const standaloneCloseButton = createFloatingCloseButton();
            console.log('Created standalone close button as fallback');
            
            // Start the XR animation loop
            xrSession.requestAnimationFrame(onXRAnimationFrame);
            
            // Update UI
            startXRBtn.disabled = true;
            stopXRBtn.disabled = false;
        } else {
            // WebXR not available, use device orientation as fallback
            startDeviceOrientationTracking();
        }
        
    } catch (error) {
        console.error('Error starting XR session:', error);
        xrStatus.textContent = 'WebXR: Error starting session';
        // Try fallback to device orientation
        startDeviceOrientationTracking();
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
            },
            source: 'webxr'
        };
        
        // Display the pose data
        displayPoseData(poseData);
        
        // Send the pose data via WebSocket if connected
        if (socket && socket.readyState === WebSocket.OPEN) {
            socket.send(JSON.stringify(poseData));
        }
    } else {
        console.log('No pose available in this frame');
    }
}

// Display pose data in the UI
function displayPoseData(poseData) {
    poseDisplay.textContent = JSON.stringify(poseData, null, 2);
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
    
    // Remove standalone close button if it exists
    const standaloneCloseButton = document.getElementById('standalone-close-button');
    if (standaloneCloseButton) {
        document.body.removeChild(standaloneCloseButton);
    }
    
    // Update UI
    xrStatus.textContent = 'WebXR: Ended';
    startXRBtn.disabled = false;
    stopXRBtn.disabled = true;
    
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
    
    // Also stop device orientation tracking if active
    stopDeviceOrientationTracking();
}

// Variables for device orientation tracking
let isDeviceOrientationTracking = false;
let deviceOrientationInterval = null;

// Start tracking with device orientation (mobile fallback)
function startDeviceOrientationTracking() {
    if (window.DeviceOrientationEvent) {
        // Request permission for iOS 13+ devices
        if (typeof DeviceOrientationEvent.requestPermission === 'function') {
            DeviceOrientationEvent.requestPermission()
                .then(permissionState => {
                    if (permissionState === 'granted') {
                        enableDeviceOrientation();
                    } else {
                        xrStatus.textContent = 'Mobile: Permission denied for motion sensors';
                    }
                })
                .catch(console.error);
        } else {
            // Non-iOS devices don't need permission
            enableDeviceOrientation();
        }
    } else {
        xrStatus.textContent = 'Mobile: Device orientation not supported';
    }
}

// Enable device orientation tracking
function enableDeviceOrientation() {
    isDeviceOrientationTracking = true;
    xrStatus.textContent = 'Mobile: Using Device Orientation';
    startXRBtn.disabled = true;
    stopXRBtn.disabled = false;
    
    // Store last orientation values
    let alpha = 0, beta = 0, gamma = 0;
    
    // Handle device orientation events
    function handleOrientation(event) {
        alpha = event.alpha || 0; // Z-axis rotation [0, 360)
        beta = event.beta || 0;   // X-axis rotation [-180, 180)
        gamma = event.gamma || 0; // Y-axis rotation [-90, 90)
    }
    
    // Add the event listener
    window.addEventListener('deviceorientation', handleOrientation, true);
    
    // Send pose data at regular intervals
    deviceOrientationInterval = setInterval(() => {
        if (!isDeviceOrientationTracking) return;
        
        // Convert Euler angles to approximate quaternion
        // This is a simplified conversion
        const degToRad = Math.PI / 180;
        const alphaRad = alpha * degToRad;
        const betaRad = beta * degToRad;
        const gammaRad = gamma * degToRad;
        
        // Create a pose data object with device orientation
        const poseData = {
            timestamp: new Date().toISOString(),
            position: {
                x: "0.0000", // We don't have position from orientation sensors
                y: "0.0000",
                z: "0.0000"
            },
            orientation: {
                // Approximated quaternion values from Euler angles
                // This is a simplified conversion and not entirely accurate
                x: (Math.sin(betaRad/2) * Math.cos(gammaRad/2) * Math.cos(alphaRad/2) - 
                   Math.cos(betaRad/2) * Math.sin(gammaRad/2) * Math.sin(alphaRad/2)).toFixed(4),
                y: (Math.cos(betaRad/2) * Math.sin(gammaRad/2) * Math.cos(alphaRad/2) + 
                   Math.sin(betaRad/2) * Math.cos(gammaRad/2) * Math.sin(alphaRad/2)).toFixed(4),
                z: (Math.cos(betaRad/2) * Math.cos(gammaRad/2) * Math.sin(alphaRad/2) - 
                   Math.sin(betaRad/2) * Math.sin(gammaRad/2) * Math.cos(alphaRad/2)).toFixed(4),
                w: (Math.cos(betaRad/2) * Math.cos(gammaRad/2) * Math.cos(alphaRad/2) + 
                   Math.sin(betaRad/2) * Math.sin(gammaRad/2) * Math.sin(alphaRad/2)).toFixed(4)
            },
            deviceOrientation: {
                alpha: alpha.toFixed(2),
                beta: beta.toFixed(2),
                gamma: gamma.toFixed(2)
            },
            isMobile: true
        };
        
        // Display the pose data
        displayPoseData(poseData);
        
        // Send the pose data via WebSocket if connected
        if (socket && socket.readyState === WebSocket.OPEN) {
            socket.send(JSON.stringify(poseData));
        }
    }, 100); // 10 times per second
}

// Stop device orientation tracking
function stopDeviceOrientationTracking() {
    if (isDeviceOrientationTracking) {
        isDeviceOrientationTracking = false;
        window.removeEventListener('deviceorientation', handleOrientation, true);
        clearInterval(deviceOrientationInterval);
        xrStatus.textContent = 'Mobile: Device Orientation Stopped';
        startXRBtn.disabled = false;
        stopXRBtn.disabled = true;
    }
}

// Create a standard DOM close button (fallback for DOM overlay issues)
function createFloatingCloseButton() {
    // Create a standalone close button that's always visible
    const closeButton = document.createElement('button');
    closeButton.id = 'standalone-close-button';
    closeButton.innerText = 'EXIT AR';
    closeButton.style.position = 'fixed';
    closeButton.style.bottom = '50px';
    closeButton.style.left = '50%';
    closeButton.style.transform = 'translateX(-50%)';
    closeButton.style.zIndex = '2147483647'; // Maximum z-index value
    closeButton.style.padding = '20px 40px';
    closeButton.style.backgroundColor = '#FF3B30';
    closeButton.style.color = 'white';
    closeButton.style.border = 'none';
    closeButton.style.borderRadius = '40px';
    closeButton.style.fontSize = '24px';
    closeButton.style.fontWeight = 'bold';
    closeButton.style.boxShadow = '0 8px 16px rgba(0, 0, 0, 0.5)';
    closeButton.style.cursor = 'pointer';
    closeButton.style.userSelect = 'none';
    closeButton.style.WebkitTapHighlightColor = 'transparent';
    closeButton.style.fontFamily = 'Arial, sans-serif';
    closeButton.style.transition = 'transform 0.2s';
    
    // Add hover/active effects
    closeButton.addEventListener('mousedown', () => {
        closeButton.style.transform = 'translateX(-50%) scale(0.95)';
    });
    
    closeButton.addEventListener('mouseup', () => {
        closeButton.style.transform = 'translateX(-50%) scale(1)';
    });
    
    // Add event listener to close the session when button is clicked
    closeButton.addEventListener('click', () => {
        console.log('Standalone close button clicked');
        if (xrSession) {
            xrSession.end();
        }
    });
    
    document.body.appendChild(closeButton);
    return closeButton;
}

// Event listeners
connectBtn.addEventListener('click', toggleConnection);
startXRBtn.addEventListener('click', startXRSession);
stopXRBtn.addEventListener('click', stopXRSession);

// Initialize
checkXRSupport();
