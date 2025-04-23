/*
 * This ESP32 code is created by esp32io.com
 *
 * This ESP32 code is released in the public domain
 *
 * For more detail (instruction and wiring diagram), visit https://esp32io.com/tutorials/esp32-controls-car-via-web
 */

 const char *HTML_CONTENT = R"=====(
    <!DOCTYPE html>
    <html>
    <head>
    <title>ESP32 Control Car via Web</title>
    <meta name="viewport" content="width=device-width, initial-scale=0.7, maximum-scale=1, user-scalable=no">
    <style type="text/css">
    body { 
      text-align: center; 
      font-size: 24px;
      font-family: Arial, sans-serif;
      background-color: #f0f0f0;
      padding: 10px;
      color: #333;
    }
    
    h2, h3 {
      color: #2196F3;
      margin: 15px 0;
    }
    
    button { 
      text-align: center; 
      font-size: 24px;
    }
    
    .controls-container {
      display: flex;
      flex-direction: column;
      align-items: center;
      margin-bottom: 30px;
    }
    
    .control-section {
      margin-bottom: 20px;
      background-color: white;
      border-radius: 15px;
      padding: 15px;
      box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1);
      width: 100%;
      max-width: 450px;
    }
    
    #container {
        margin-right: auto;
        margin-left: auto;
        width: 400px; 
        height: 400px;
        position: relative;
        margin-bottom: 10px;
    }
    
    #smallContainer {
        margin-right: auto;
        margin-left: auto;
        width: 300px; 
        height: 300px;
        position: relative;
        margin-bottom: 10px;
    }
    
    div[class^='button'] { 
        position: absolute;
        display: flex;
        align-items: center;
        justify-content: center;
        border-radius: 15px;
        border: 3px solid #333;
        color: white;
        font-weight: bold;
        cursor: pointer;
        transition: background-color 0.2s;
    }
    
    /* Regular speed controls */
    .button_up, .button_down { width:214px; height:104px;}
    .button_left, .button_right { width:104px; height:214px;}
    .button_stop { width:178px; height:178px;}
    
    /* Slow speed controls */
    .button_slow_up, .button_slow_down { width:160px; height:78px; font-size: 20px;}
    .button_slow_left, .button_slow_right { width:78px; height:160px; font-size: 20px;}
    .button_slow_stop { width:134px; height:134px; font-size: 20px;}
    
    .button_up {
        background-color: #4CAF50;
        left: 200px;
        top: 0px;
        transform: translateX(-50%);
    }
    
    .button_down {
        background-color: #f44336;
        left:200px;
        bottom: 0px;
        transform: translateX(-50%);
    }
    
    .button_right {
        background-color: #2196F3;
        right: 0px;
        top: 200px;
        transform: translateY(-50%);
    }
    
    .button_left {
        background-color: #2196F3;
        left:0px;
        top: 200px;
        transform: translateY(-50%);
    }
    
    .button_stop {
        background-color: #FF9800;
        left:200px;
        top: 200px;
        transform: translate(-50%, -50%);
    }
    
    /* Slow speed button positions */
    .button_slow_up {
        background-color: #4CAF50;
        left: 150px;
        top: 0px;
        transform: translateX(-50%);
    }
    
    .button_slow_down {
        background-color: #f44336;
        left:150px;
        bottom: 0px;
        transform: translateX(-50%);
    }
    
    .button_slow_right {
        background-color: #2196F3;
        right: 0px;
        top: 150px;
        transform: translateY(-50%);
    }
    
    .button_slow_left {
        background-color: #2196F3;
        left:0px;
        top: 150px;
        transform: translateY(-50%);
    }
    
    .button_slow_stop {
        background-color: #FF9800;
        left:150px;
        top: 150px;
        transform: translate(-50%, -50%);
    }
    
    .active {
        background-color: #555 !important;
        transform: scale(0.95) translateX(-50%);
    }
    
    .button_up.active, .button_down.active {
        transform: scale(0.95) translateX(-50%);
    }
    
    .button_left.active, .button_right.active {
        transform: scale(0.95) translateY(-50%);
    }
    
    .button_stop.active {
        transform: scale(0.95) translate(-50%, -50%);
    }
    
    .button_slow_up.active, .button_slow_down.active {
        transform: scale(0.95) translateX(-50%);
    }
    
    .button_slow_left.active, .button_slow_right.active {
        transform: scale(0.95) translateY(-50%);
    }
    
    .button_slow_stop.active {
        transform: scale(0.95) translate(-50%, -50%);
    }

    /* Servo Control Styles */
    .servo-controls {
        width: 90%;
        max-width: 400px;
        margin: 20px auto;
        padding: 15px;
        border: 2px solid #333;
        border-radius: 10px;
        background-color: white;
        box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1);
    }

    .slider-container {
        margin: 20px 0;
        padding: 10px;
        border-radius: 8px;
        background-color: #f9f9f9;
    }

    .slider-container label {
        display: block;
        margin-bottom: 8px;
        font-weight: bold;
        color: #333;
    }

    .servo-value {
        font-weight: bold;
        color: #2196F3;
        margin: 10px 0;
        font-size: 28px;
    }

    .slider-actions {
        display: flex;
        justify-content: space-around;
        margin-top: 10px;
    }

    .slider-actions button {
        padding: 12px 20px;
        font-size: 24px;
        background-color: #4CAF50;
        color: white;
        border: none;
        border-radius: 8px;
        cursor: pointer;
        margin: 0 5px;
        min-width: 60px;
        user-select: none;
        -webkit-user-select: none;
        box-shadow: 0 2px 4px rgba(0, 0, 0, 0.2);
        transition: all 0.2s ease;
    }

    .slider-actions button.decrease {
        background-color: #f44336;
    }

    .slider-actions button.action {
        background-color: #2196F3;
    }

    .slider-actions button:hover {
        filter: brightness(1.1);
    }
    
    .slider-actions button:active {
        transform: scale(0.95);
        box-shadow: 0 1px 2px rgba(0, 0, 0, 0.2);
    }
    
    .connection-status {
        margin: 20px auto;
        padding: 10px;
        background-color: white;
        border-radius: 8px;
        box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
        width: 90%;
        max-width: 400px;
    }
    
    #ws_state {
        font-weight: bold;
    }
    
    #wc_conn {
        margin-top: 10px;
        padding: 10px 20px;
        background-color: #2196F3;
        color: white;
        border: none;
        border-radius: 8px;
        cursor: pointer;
        box-shadow: 0 2px 4px rgba(0, 0, 0, 0.2);
        transition: all 0.2s ease;
    }
    
    #wc_conn:hover {
        background-color: #0b7dda;
    }
    
    .sponsor {
        margin-top: 20px;
        font-size: 18px;
        color: #666;
    }
    
    .sponsor a {
        color: #2196F3;
        text-decoration: none;
    }
    
    .sponsor a:hover {
        text-decoration: underline;
    }
    </style>
    <script>
    var CMD_STOP     = 0;
    var CMD_FORWARD  = 1;
    var CMD_BACKWARD = 2;
    var CMD_LEFT     = 4;
    var CMD_RIGHT    = 8;
    var CMD_SERVO1   = 16;
    var CMD_SERVO2   = 32;
    var CMD_SERVO3   = 64;
    
    // Slow movement commands
    var CMD_SLOW_FORWARD  = 101;
    var CMD_SLOW_BACKWARD = 102;
    var CMD_SLOW_LEFT     = 104;
    var CMD_SLOW_RIGHT    = 108;
    
    var buttonText = {
      [CMD_STOP]:     "STOP",
      [CMD_FORWARD]:  "FORWARD",
      [CMD_BACKWARD]: "BACKWARD",
      [CMD_LEFT]:     "LEFT",
      [CMD_RIGHT]:    "RIGHT",
      [CMD_SLOW_FORWARD]:  "SLOW<br>FORWARD",
      [CMD_SLOW_BACKWARD]: "SLOW<br>BACKWARD",
      [CMD_SLOW_LEFT]:     "SLOW<br>LEFT",
      [CMD_SLOW_RIGHT]:    "SLOW<br>RIGHT"
    }
    var ws = null;
    var servoIntervals = {
      servo1: null,
      servo2: null, 
      servo3: null
    };
    var updateRate = 100; // Update servo position every 100ms (very slow)
    var adjustStep = 1; // Degrees to change per update
    
    function init() 
    {
      // Regular speed controls
      var container = document.querySelector("#container");
      container.addEventListener("touchstart", mouse_down);
      container.addEventListener("touchend", mouse_up);
      container.addEventListener("touchcancel", mouse_up);
      container.addEventListener("mousedown", mouse_down);
      container.addEventListener("mouseup", mouse_up);
      container.addEventListener("mouseout", mouse_up);
      
      // Slow speed controls  
      var smallContainer = document.querySelector("#smallContainer");
      smallContainer.addEventListener("touchstart", mouse_down);
      smallContainer.addEventListener("touchend", mouse_up);
      smallContainer.addEventListener("touchcancel", mouse_up);
      smallContainer.addEventListener("mousedown", mouse_down);
      smallContainer.addEventListener("mouseup", mouse_up);
      smallContainer.addEventListener("mouseout", mouse_up);

      // Initialize servo values
      updateServoDisplay('servo1', 90);
      updateServoDisplay('servo2', 90);
      updateServoDisplay('servo3', 90);
      
      // Set up event listeners for servo control buttons
      setupServoControlButtons();
    }

    function setupServoControlButtons() {
      var servoIds = ['servo1', 'servo2', 'servo3'];
      var cmdIds = [CMD_SERVO1, CMD_SERVO2, CMD_SERVO3];
      
      servoIds.forEach((servoId, index) => {
        // Decrease button - press and hold
        var decreaseBtn = document.getElementById(servoId + 'Decrease');
        decreaseBtn.addEventListener('mousedown', () => startServoAdjustment(servoId, -adjustStep, cmdIds[index]));
        decreaseBtn.addEventListener('touchstart', (e) => {
          e.preventDefault();
          startServoAdjustment(servoId, -adjustStep, cmdIds[index]);
        });
        
        // Increase button - press and hold
        var increaseBtn = document.getElementById(servoId + 'Increase');
        increaseBtn.addEventListener('mousedown', () => startServoAdjustment(servoId, adjustStep, cmdIds[index]));
        increaseBtn.addEventListener('touchstart', (e) => {
          e.preventDefault();
          startServoAdjustment(servoId, adjustStep, cmdIds[index]);
        });
        
        // Stop adjustment when releasing button (for both buttons)
        [decreaseBtn, increaseBtn].forEach(btn => {
          btn.addEventListener('mouseup', () => stopServoAdjustment(servoId));
          btn.addEventListener('mouseleave', () => stopServoAdjustment(servoId));
          btn.addEventListener('touchend', () => stopServoAdjustment(servoId));
          btn.addEventListener('touchcancel', () => stopServoAdjustment(servoId));
        });
      });
    }
    
    function startServoAdjustment(servoId, step, cmdId) {
      // Clear any existing interval
      stopServoAdjustment(servoId);
      
      // Start continuous adjustment
      servoIntervals[servoId] = setInterval(() => {
        var newValue = adjustServo(servoId, step);
        
        // Send the new value immediately to update the servo
        if (ws != null && ws.readyState == 1) {
          ws.send(cmdId + ":" + newValue + "\r\n");
        }
      }, updateRate);
    }
    
    function stopServoAdjustment(servoId) {
      if (servoIntervals[servoId]) {
        clearInterval(servoIntervals[servoId]);
        servoIntervals[servoId] = null;
      }
    }

    function updateServoDisplay(servoId, value) {
      var display = document.getElementById(servoId + 'Value');
      display.textContent = value + '°';
      // Store the current value as a data attribute
      display.setAttribute('data-value', value);
    }
    
    function adjustServo(servoId, amount) {
      var display = document.getElementById(servoId + 'Value');
      var currentValue = parseInt(display.getAttribute('data-value')) || 90;
      var newValue = Math.max(0, Math.min(180, currentValue + amount));
      
      updateServoDisplay(servoId, newValue);
      return newValue;
    }
    
    function ws_onmessage(e_msg)
    {
        e_msg = e_msg || window.event; // MessageEvent
     
        //alert("msg : " + e_msg.data);
    }
    function ws_onopen()
    {
      document.getElementById("ws_state").innerHTML = "CONNECTED";
      document.getElementById("ws_state").style.color = "#4CAF50";
      document.getElementById("wc_conn").innerHTML = "Disconnect";
    }
    function ws_onclose()
    {
      document.getElementById("ws_state").innerHTML = "DISCONNECTED";
      document.getElementById("ws_state").style.color = "#f44336";
      document.getElementById("wc_conn").innerHTML = "Connect";
      console.log("socket was closed");
      ws.onopen = null;
      ws.onclose = null;
      ws.onmessage = null;
      ws = null;
    }
    function wc_onclick()
    {
      if(ws == null)
      {
        ws = new WebSocket("ws://" + window.location.host + ":81");
        document.getElementById("ws_state").innerHTML = "CONNECTING...";
        document.getElementById("ws_state").style.color = "#FF9800";
        
        ws.onopen = ws_onopen;
        ws.onclose = ws_onclose;
        ws.onmessage = ws_onmessage; 
      }
      else
        ws.close();
    }
    function mouse_down(event) 
    {
      if (event.target !== event.currentTarget) 
      {
        var id = event.target.id;
        send_command(id);
        event.target.classList.add('active');
      }
      event.stopPropagation();    
      event.preventDefault();    
    }
    
    function mouse_up(event) 
    {
      if (event.target !== event.currentTarget) 
      {
        var id = event.target.id;
        send_command(CMD_STOP);
        event.target.classList.remove('active');
      }
      event.stopPropagation();   
      event.preventDefault();    
    }
    function send_command(cmd) 
    {   
      if(ws != null)
        if(ws.readyState == 1)
          ws.send(cmd + "\r\n");   
    }
    
    function setServoPosition(servoCmd) {
      var servoId;
      switch(servoCmd) {
        case CMD_SERVO1: servoId = 'servo1'; break;
        case CMD_SERVO2: servoId = 'servo2'; break;
        case CMD_SERVO3: servoId = 'servo3'; break;
      }
      
      if (servoId) {
        var position = document.getElementById(servoId + 'Value').getAttribute('data-value');
        if(ws != null && ws.readyState == 1) {
          ws.send(servoCmd + ":" + position + "\r\n");
        }
      }
    }

    function setMin(servoId) {
      updateServoDisplay(servoId, 0);
      var cmdId;
      switch(servoId) {
        case 'servo1': cmdId = CMD_SERVO1; break;
        case 'servo2': cmdId = CMD_SERVO2; break;
        case 'servo3': cmdId = CMD_SERVO3; break;
      }
      if(ws != null && ws.readyState == 1 && cmdId) {
        ws.send(cmdId + ":0\r\n");
      }
    }

    function setMid(servoId) {
      updateServoDisplay(servoId, 90);
      var cmdId;
      switch(servoId) {
        case 'servo1': cmdId = CMD_SERVO1; break;
        case 'servo2': cmdId = CMD_SERVO2; break;
        case 'servo3': cmdId = CMD_SERVO3; break;
      }
      if(ws != null && ws.readyState == 1 && cmdId) {
        ws.send(cmdId + ":90\r\n");
      }
    }

    function setMax(servoId) {
      updateServoDisplay(servoId, 180);
      var cmdId;
      switch(servoId) {
        case 'servo1': cmdId = CMD_SERVO1; break;
        case 'servo2': cmdId = CMD_SERVO2; break;
        case 'servo3': cmdId = CMD_SERVO3; break;
      }
      if(ws != null && ws.readyState == 1 && cmdId) {
        ws.send(cmdId + ":180\r\n");
      }
    }
    
    window.onload = init;
    </script>
    </head>
    <body>
    <h2>ESP32 - RC Car Control</h2>
    
    <div class="controls-container">
      <div class="control-section">
        <h3>Regular Speed Controls</h3>
        <div id="container">
          <div id="0" class="button_stop">STOP</div>
          <div id="1" class="button_up">FORWARD</div>
          <div id="2" class="button_down">BACKWARD</div>
          <div id="8" class="button_right">RIGHT</div>
          <div id="4" class="button_left">LEFT</div>
        </div>
      </div>
      
      <div class="control-section">
        <h3>Fine Movement Controls (Slow Speed)</h3>
        <div id="smallContainer">
          <div id="0" class="button_slow_stop">STOP</div>
          <div id="101" class="button_slow_up">SLOW FORWARD</div>
          <div id="102" class="button_slow_down">SLOW BACKWARD</div>
          <div id="108" class="button_slow_right">SLOW RIGHT</div>
          <div id="104" class="button_slow_left">SLOW LEFT</div>
        </div>
      </div>
    </div>
    
    <div class="servo-controls">
      <h3>Servo Controls</h3>
      
      <div class="slider-container">
        <label for="servo1">Servo 1 (Pin 13):</label>
        <div class="servo-value" id="servo1Value">90°</div>
        <div class="slider-actions">
          <button id="servo1Decrease" class="decrease">◄</button>
          <button class="action" onclick="setMin('servo1')">Min</button>
          <button class="action" onclick="setMid('servo1')">Mid</button>
          <button class="action" onclick="setMax('servo1')">Max</button>
          <button id="servo1Increase">►</button>
        </div>
      </div>
      
      <div class="slider-container">
        <label for="servo2">Servo 2 (Pin 12):</label>
        <div class="servo-value" id="servo2Value">90°</div>
        <div class="slider-actions">
          <button id="servo2Decrease" class="decrease">◄</button>
          <button class="action" onclick="setMin('servo2')">Min</button>
          <button class="action" onclick="setMid('servo2')">Mid</button>
          <button class="action" onclick="setMax('servo2')">Max</button>
          <button id="servo2Increase">►</button>
        </div>
      </div>
      
      <div class="slider-container">
        <label for="servo3">Servo 3 (Pin 14):</label>
        <div class="servo-value" id="servo3Value">90°</div>
        <div class="slider-actions">
          <button id="servo3Decrease" class="decrease">◄</button>
          <button class="action" onclick="setMin('servo3')">Min</button>
          <button class="action" onclick="setMid('servo3')">Mid</button>
          <button class="action" onclick="setMax('servo3')">Max</button>
          <button id="servo3Increase">►</button>
        </div>
      </div>
    </div>

    <div class="connection-status">
      <p>WebSocket Status: <span id="ws_state" style="color:#f44336">DISCONNECTED</span></p>
      <button id="wc_conn" type="button" onclick="wc_onclick();">Connect</button>
    </div>
    
    <div class="sponsor">Sponsored by <a href="https://amazon.com/diyables">DIYables</a></div>
    </body>
    </html>
    )=====";