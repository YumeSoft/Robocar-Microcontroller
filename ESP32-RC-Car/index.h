/*
 * This ESP32 code is created by esp32io.com
 *
 * This ESP32 code is released in the public domain
 *
 * For more detail (instruction and wiring diagram), visit https://esp32io.com/tutorials/esp32-controls-car-via-web
 */

 const char HTML_CONTENT[] PROGMEM = R"rawliteral(
    <!DOCTYPE html>
    <html>
    <head>
        <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
        <title>ESP32 RC Car Control</title>
        <style>
            body {
                font-family: Arial, sans-serif;
                text-align: center;
                margin: 0;
                padding: 0;
                max-width: 800px;
                background-color: #222;
                color: #fff;
                min-height: 100vh;
                position: relative;
                overflow-y: auto;
                -webkit-overflow-scrolling: touch; /* Enables smooth scrolling on iOS */
            }
            .container {
                display: flex;
                flex-direction: column;
                padding-bottom: 60px; /* Space for footer */
            }
            .control-area {
                flex: 1;
                display: flex;
                flex-direction: column;
                justify-content: space-between;
                padding: 10px;
            }
            .button-container {
                display: grid;
                grid-template-columns: repeat(3, 1fr);
                grid-template-rows: repeat(3, 1fr);
                gap: 10px;
                margin-bottom: 20px;
            }
            .servo-controls, .speed-control {
                margin-top: 20px;
            }
            .control-button {
                height: 70px;
                font-size: 24px;
                background-color: #444;
                color: white;
                border: none;
                border-radius: 10px;
                cursor: pointer;
                user-select: none;
                -webkit-user-select: none;
                -webkit-touch-callout: none;
            }
            .control-button:active {
                background-color: #666;
            }
            .center-button {
                background-color: #f44336;
            }
            .direction-button {
                background-color: #2196F3;
            }
            .empty {
                visibility: hidden;
            }
            .slider-container {
                width: 100%;
                margin: 20px 0;
                padding: 10px 0;
            }
            .slider {
                width: 100%;
                height: 40px;
                margin: 10px 0;
                -webkit-appearance: none;
                appearance: none;
                background: #444;
                outline: none;
                border-radius: 20px;
            }
            .slider::-webkit-slider-thumb {
                -webkit-appearance: none;
                appearance: none;
                width: 50px;
                height: 50px;
                border-radius: 50%;
                background: #2196F3;
                cursor: pointer;
            }
            .slider::-moz-range-thumb {
                width: 50px;
                height: 50px;
                border-radius: 50%;
                background: #2196F3;
                cursor: pointer;
            }
            .servo-value {
                font-size: 20px;
                font-weight: bold;
                margin: 5px 0;
                display: inline-block;
                width: 50px;
            }
            .footer {
                padding: 10px;
                background-color: #333;
                font-size: 12px;
                position: relative;
                width: 100%;
                bottom: 0;
            }
            /* Speed toggle switch */
            .switch {
                position: relative;
                display: inline-block;
                width: 60px;
                height: 34px;
                margin: 10px;
            }
            .switch input { 
                opacity: 0;
                width: 0;
                height: 0;
            }
            .slider-switch {
                position: absolute;
                cursor: pointer;
                top: 0;
                left: 0;
                right: 0;
                bottom: 0;
                background-color: #ccc;
                transition: .4s;
                border-radius: 34px;
            }
            .slider-switch:before {
                position: absolute;
                content: "";
                height: 26px;
                width: 26px;
                left: 4px;
                bottom: 4px;
                background-color: white;
                transition: .4s;
                border-radius: 50%;
            }
            input:checked + .slider-switch {
                background-color: #2196F3;
            }
            input:focus + .slider-switch {
                box-shadow: 0 0 1px #2196F3;
            }
            input:checked + .slider-switch:before {
                transform: translateX(26px);
            }
        </style>
    </head>
    <body>
        <div class="container">
            <div class="control-area">
                <h2>ESP32 RC Car Control</h2>
                
                <!-- Movement Controls -->
                <div class="button-container">
                    <button class="control-button empty"></button>
                    <button class="control-button direction-button" id="forward" ontouchstart="sendCommand(1); return false;" ontouchend="sendCommand(0); return false;" onmousedown="sendCommand(1)" onmouseup="sendCommand(0)">Forward</button>
                    <button class="control-button empty"></button>
                    
                    <button class="control-button direction-button" id="left" ontouchstart="sendCommand(4); return false;" ontouchend="sendCommand(0); return false;" onmousedown="sendCommand(4)" onmouseup="sendCommand(0)">Left</button>
                    <button class="control-button center-button" id="stop" onclick="sendCommand(0)">Stop</button>
                    <button class="control-button direction-button" id="right" ontouchstart="sendCommand(8); return false;" ontouchend="sendCommand(0); return false;" onmousedown="sendCommand(8)" onmouseup="sendCommand(0)">Right</button>
                    
                    <button class="control-button empty"></button>
                    <button class="control-button direction-button" id="backward" ontouchstart="sendCommand(2); return false;" ontouchend="sendCommand(0); return false;" onmousedown="sendCommand(2)" onmouseup="sendCommand(0)">Backward</button>
                    <button class="control-button empty"></button>
                </div>
    
                <!-- Speed Control -->
                <div class="speed-control">
                    <h3>Motor Speed</h3>
                    <label class="switch">
                        <input type="checkbox" id="speedToggle" onchange="toggleSpeed()">
                        <span class="slider-switch"></span>
                    </label>
                    <span id="speedLabel">Normal Speed</span>
                </div>
                
                <!-- Servo Controls -->
                <div class="servo-controls">
                    <h3>Servo Controls</h3>
                    <div class="slider-container">
                        <span>Servo 1</span>
                        <input type="range" min="0" max="180" value="90" class="slider" id="servo1" oninput="updateServo(1, this.value)">
                        <span id="servo1Value" class="servo-value">90°</span>
                    </div>
                    <div class="slider-container">
                        <span>Servo 2</span>
                        <input type="range" min="0" max="180" value="90" class="slider" id="servo2" oninput="updateServo(2, this.value)">
                        <span id="servo2Value" class="servo-value">90°</span>
                    </div>
                    <div class="slider-container">
                        <span>Servo 3</span>
                        <input type="range" min="0" max="180" value="90" class="slider" id="servo3" oninput="updateServo(3, this.value)">
                        <span id="servo3Value" class="servo-value">90°</span>
                    </div>
                </div>
            </div>
            
            <div class="footer">
                Made by Tran Minh Thuan
            </div>
        </div>
        
        <script>
            var ws;
            var isHighSpeed = true;
            var servoValues = {1: 90, 2: 90, 3: 90};
            var wsConnected = false;
            var connectionAttempts = 0;
            var maxConnectionAttempts = 5;
            
            // Allow page scrolling in general, only prevent on specific elements
            document.addEventListener('touchmove', function(e) {
                // By default, allow scrolling
            }, { passive: true });
            
            function connectWebSocket() {
                // Use the current hostname and port 81
                var wsUrl = 'ws://' + window.location.hostname + ':81';
                
                // If running on localhost for testing, use the ESP32's IP directly
                if (window.location.hostname === 'localhost' || window.location.hostname === '127.0.0.1') {
                    wsUrl = 'ws://192.168.4.1:81';
                }
                
                console.log('Connecting to WebSocket at: ' + wsUrl);
                document.getElementById('stop').style.backgroundColor = "#999"; // Gray when connecting
                document.getElementById('stop').innerText = "Connecting...";
                
                try {
                    ws = new WebSocket(wsUrl);
                    
                    ws.onopen = function() {
                        console.log('Connected to WebSocket server');
                        wsConnected = true;
                        connectionAttempts = 0;
                        document.getElementById('stop').style.backgroundColor = "#f44336"; // Red when connected
                        document.getElementById('stop').innerText = "Stop";
                        // Send a test message to confirm connection
                        ws.send("0");
                    };
                    
                    ws.onclose = function() {
                        console.log('Disconnected from WebSocket server');
                        wsConnected = false;
                        document.getElementById('stop').style.backgroundColor = "#999"; // Gray when disconnected
                        document.getElementById('stop').innerText = "Disconnected";
                        
                        // Attempt to reconnect with increasing delays
                        if (connectionAttempts < maxConnectionAttempts) {
                            connectionAttempts++;
                            var reconnectDelay = 1000 * connectionAttempts; // Increase delay with each attempt
                            console.log('Reconnecting in ' + (reconnectDelay/1000) + ' seconds... (Attempt ' + connectionAttempts + ')');
                            setTimeout(connectWebSocket, reconnectDelay);
                        } else {
                            console.log('Max reconnection attempts reached. Please refresh the page to try again.');
                            document.getElementById('stop').innerText = "Reload Page";
                            document.getElementById('stop').onclick = function() {
                                window.location.reload();
                            };
                        }
                    };
                    
                    ws.onerror = function(err) {
                        console.error('WebSocket error:', err);
                        wsConnected = false;
                    };
                } catch (e) {
                    console.error('WebSocket creation error:', e);
                    document.getElementById('stop').innerText = "Conn Failed";
                    document.getElementById('stop').style.backgroundColor = "#ff9800"; // Orange for error
                }
            }
            
            function sendCommand(cmd) {
                console.log('Attempting to send command: ' + cmd);
                if (ws && ws.readyState === WebSocket.OPEN) {
                    // Apply speed modifier if low speed is selected
                    if (!isHighSpeed && (cmd == 1 || cmd == 2 || cmd == 4 || cmd == 8)) {
                        // Add 100 to normal commands to indicate slow movement
                        cmd += 100;
                    }
                    ws.send(cmd.toString());
                    console.log('Command sent: ' + cmd);
                    return true;
                } else {
                    console.log('WebSocket not connected, trying to connect...');
                    document.getElementById('stop').innerText = "Connecting...";
                    connectWebSocket();
                    return false;
                }
            }
            
            // Setup button event listeners properly after DOM is loaded
            document.addEventListener('DOMContentLoaded', function() {
                // Add touch/click event listeners to direction buttons
                var buttons = document.querySelectorAll('.direction-button');
                buttons.forEach(function(button) {
                    button.addEventListener('touchstart', function(e) {
                        e.preventDefault(); // Prevent default behavior
                        var id = button.id;
                        switch(id) {
                            case 'forward': sendCommand(1); break;
                            case 'backward': sendCommand(2); break;
                            case 'left': sendCommand(4); break;
                            case 'right': sendCommand(8); break;
                        }
                    });
                    
                    button.addEventListener('touchend', function(e) {
                        e.preventDefault(); // Prevent default behavior
                        sendCommand(0); // Stop on touch end
                    });
                });
                
                // Add click event to stop button
                document.getElementById('stop').addEventListener('click', function(e) {
                    e.preventDefault();
                    sendCommand(0);
                });
            });
            
            function updateServo(servoNum, position) {
                // Update display
                document.getElementById('servo' + servoNum + 'Value').innerText = position + '°';
                
                // Store current position
                servoValues[servoNum] = position;
                
                // Send command if connected
                if (ws && ws.readyState === WebSocket.OPEN) {
                    // Convert servo number to command
                    var cmd = 0;
                    switch(servoNum) {
                        case 1: cmd = 16; break;  // CMD_SERVO1
                        case 2: cmd = 32; break;  // CMD_SERVO2
                        case 3: cmd = 64; break;  // CMD_SERVO3
                    }
                    
                    var message = cmd + ':' + position;
                    ws.send(message);
                    console.log('Servo command sent: ' + message);
                } else {
                    console.log('WebSocket not connected for servo command');
                    connectWebSocket();
                }
            }
            
            function toggleSpeed() {
                isHighSpeed = !document.getElementById('speedToggle').checked;
                document.getElementById('speedLabel').innerText = isHighSpeed ? 'Normal Speed' : 'Slow Speed';
            }
            
            // Connect when the page loads
            window.onload = function() {
                console.log('Page loaded, connecting WebSocket...');
                // Add status display to debugging
                document.body.insertAdjacentHTML('afterbegin', 
                    '<div id="debug" style="position:fixed; top:0; right:0; background:rgba(0,0,0,0.7); color:white; padding:5px; font-size:12px; z-index:1000;">Connecting...</div>');
                
                // Start WebSocket connection
                connectWebSocket();
                
                // Update debug info
                setInterval(function() {
                    var debugElem = document.getElementById('debug');
                    if (debugElem) {
                        var status = wsConnected ? 'Connected' : 'Disconnected';
                        var readyState = ws ? ['Connecting', 'Open', 'Closing', 'Closed'][ws.readyState] : 'Unknown';
                        debugElem.innerHTML = 'WebSocket: ' + status + '<br>State: ' + readyState;
                    }
                }, 1000);
                
                // Add preventScroll function to sliders
                document.querySelectorAll('.slider').forEach(function(slider) {
                    slider.addEventListener('touchmove', function(e) {
                        e.stopPropagation();
                    }, { passive: false });
                });
            };
        </script>
    </body>
    </html>
    )rawliteral";