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
    body { text-align: center; font-size: 24px;}
    button { text-align: center; font-size: 24px;}
    #container {
        margin-right: auto;
        margin-left: auto;
        width: 400px; 
        height: 400px;
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
    .button_up, .button_down { width:214px; height:104px;}
    .button_left, .button_right { width:104px; height:214px;}
    .button_stop { width:178px; height:178px;}
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

    /* Servo Control Styles */
    .servo-controls {
        width: 90%;
        max-width: 400px;
        margin: 20px auto;
        padding: 15px;
        border: 2px solid #333;
        border-radius: 10px;
        background-color: #f0f0f0;
    }

    .slider-container {
        margin: 15px 0;
    }

    .slider-container label {
        display: block;
        margin-bottom: 5px;
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
        margin-top: 5px;
    }

    .slider-actions button {
        padding: 12px 20px;
        font-size: 24px;
        background-color: #4CAF50;
        color: white;
        border: none;
        border-radius: 5px;
        cursor: pointer;
        margin: 0 5px;
        min-width: 60px;
        user-select: none;
        -webkit-user-select: none;
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
    var CMD_SERVO4   = 128;
    var buttonText = {
      [CMD_STOP]:     "STOP",
      [CMD_FORWARD]:  "FORWARD",
      [CMD_BACKWARD]: "BACKWARD",
      [CMD_LEFT]:     "LEFT",
      [CMD_RIGHT]:    "RIGHT"
    }
    var ws = null;
    var servoIntervals = {
      servo1: null,
      servo2: null, 
      servo3: null,
      servo4: null
    };
    var updateRate = 100; // Update servo position every 100ms (very slow)
    var adjustStep = 1; // Degrees to change per update
    
    function init() 
    {
      var container = document.querySelector("#container");
        container.addEventListener("touchstart", mouse_down);
        container.addEventListener("touchend", mouse_up);
        container.addEventListener("touchcancel", mouse_up);
        container.addEventListener("mousedown", mouse_down);
        container.addEventListener("mouseup", mouse_up);
        container.addEventListener("mouseout", mouse_up);    

      // Initialize servo values
      updateServoDisplay('servo1', 90);
      updateServoDisplay('servo2', 90);
      updateServoDisplay('servo3', 90);
      updateServoDisplay('servo4', 90);
      
      // Set up event listeners for servo control buttons
      setupServoControlButtons();
    }

    function setupServoControlButtons() {
      var servoIds = ['servo1', 'servo2', 'servo3', 'servo4'];
      var cmdIds = [CMD_SERVO1, CMD_SERVO2, CMD_SERVO3, CMD_SERVO4];
      
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
      document.getElementById("ws_state").innerHTML = "OPEN";
      document.getElementById("wc_conn").innerHTML = "Disconnect";
    }
    function ws_onclose()
    {
      document.getElementById("ws_state").innerHTML = "CLOSED";
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
        document.getElementById("ws_state").innerHTML = "CONNECTING";
        
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
        case CMD_SERVO4: servoId = 'servo4'; break;
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
        case 'servo4': cmdId = CMD_SERVO4; break;
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
        case 'servo4': cmdId = CMD_SERVO4; break;
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
        case 'servo4': cmdId = CMD_SERVO4; break;
      }
      if(ws != null && ws.readyState == 1 && cmdId) {
        ws.send(cmdId + ":180\r\n");
      }
    }
    
    window.onload = init;
    </script>
    </head>
    <body>
    <h2>ESP32 - RC Car via Web</h2>
    <div id="container">
      <div id="0" class="button_stop">STOP</div>
      <div id="1" class="button_up">FORWARD</div>
      <div id="2" class="button_down">BACKWARD</div>
      <div id="8" class="button_right">RIGHT</div>
      <div id="4" class="button_left">LEFT</div>
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
      
      <div class="slider-container">
        <label for="servo4">Servo 4 (Pin 27):</label>
        <div class="servo-value" id="servo4Value">90°</div>
        <div class="slider-actions">
          <button id="servo4Decrease" class="decrease">◄</button>
          <button class="action" onclick="setMin('servo4')">Min</button>
          <button class="action" onclick="setMid('servo4')">Mid</button>
          <button class="action" onclick="setMax('servo4')">Max</button>
          <button id="servo4Increase">►</button>
        </div>
      </div>
    </div>

    <p>
    WebSocket : <span id="ws_state" style="color:blue">closed</span><br>
    </p>
    <button id="wc_conn" type="button" onclick="wc_onclick();">Connect</button>
    <br>
    <br>
    <div class="sponsor">Sponsored by <a href="https://amazon.com/diyables">DIYables</a></div>
    </body>
    </html>
    )=====";