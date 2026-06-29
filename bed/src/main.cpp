#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiUdp.h>

WebServer server(80);
WiFiUDP udp;

// ===================== MOTOR PINS =====================
constexpr int PWM_M1 = 25;
constexpr int DIR_M1 = 26;
constexpr int PWM_M2 = 27;
constexpr int DIR_M2 = 14;
constexpr int PWM_M3 = 18; // Side Motor
constexpr int DIR_M3 = 19; // Side Motor

// ===================== PWM CONFIG =====================
constexpr int PWM_FREQ = 20000;
constexpr int PWM_RES  = 8;
constexpr int MAX_DUTY = 255;
constexpr int RAMP_STEP = 10;

// ===================== BUTTON PINS =====================
constexpr int BTN_M1_FWD = 34;
constexpr int BTN_M1_REV = 35;
constexpr int BTN_M2_FWD = 32;
constexpr int BTN_M2_REV = 33;
constexpr int BTN_M3_L   = 5;  // Side Left
constexpr int BTN_M3_R   = 21; // Side Right

// ===================== TIMING =====================
constexpr uint32_t COAST_MS   = 150;
constexpr uint32_t LOCKOUT_MS = 200;
constexpr uint32_t NEXTION_RUN_MS = 5000; // 5 seconds for Nextion movement

enum class State { STOP, RUN_FWD, RUN_REV, COAST, LOCKOUT };

struct Motor {
  int pwmPin;
  int dirPin;
  int channel;
  State state;
  int targetDir;
  int currentDuty;
  uint32_t stateStart;
};

// ===================== GLOBAL OBJECTS =====================
Motor m1 = {PWM_M1, DIR_M1, 0, State::STOP, 0, 0, 0};
Motor m2 = {PWM_M2, DIR_M2, 1, State::STOP, 0, 0, 0};
Motor m3 = {PWM_M3, DIR_M3, 2, State::STOP, 0, 0, 0};

const int btnPins[6] = {BTN_M1_FWD, BTN_M1_REV, BTN_M2_FWD, BTN_M2_REV, BTN_M3_L, BTN_M3_R};
bool btnStates[6] = {false, false, false, false, false, false};

// Web server target directions
int webTargetDir[3] = {0, 0, 0};

// UDP/Nextion target directions and stop timers
int udpTargetDir[3] = {0, 0, 0};
uint32_t udpStopTime[3] = {0, 0, 0};

// WiFi credentials
const char* ssid = "deepan";
const char* password = "hehehehe";

// ===================== NEXTION CONFIG =====================
constexpr int RX_PIN = 16; // Change to your ESP32 RX pin
constexpr int TX_PIN = 17; // Change to your ESP32 TX pin
constexpr uint16_t UDP_PORT = 12345;

int currentPage = 0;
int currentSelection = 0;
int selectedMotor = 0; // 0=None, 1=M1, 2=M2, 3=M3

// ===================== NEXTION HELPERS =====================
void nextionSend(const String& cmd) {
  Serial1.print(cmd);
  Serial1.write(0xFF);
  Serial1.write(0xFF);
  Serial1.write(0xFF);
}

void updateNextionHighlight() {
  int maxSel = (currentPage == 0) ? 3 : 4;
  for (int i = 0; i <= maxSel; i++) {
    String btn = "b" + String(i);
    if (i == currentSelection) {
      nextionSend(btn + ".bco=65535"); // Highlight color (Yellow) - Change as needed
    } else {
      nextionSend(btn + ".bco=33281"); // Default color (Dark Blue) - Change as needed
    }
    nextionSend("ref " + btn);
  }
}

void navigateNextion() {
  int maxSel = (currentPage == 0) ? 3 : 4;
  currentSelection++;
  if (currentSelection > maxSel) currentSelection = 0;
  updateNextionHighlight();
}

void selectNextion() {
  if (currentPage == 0) {
    // Map selection to motor
    if (currentSelection == 0) selectedMotor = 1;      // TOP -> M1
    else if (currentSelection == 1) selectedMotor = 3; // RIGHT -> M3
    else if (currentSelection == 2) selectedMotor = 2; // BOTTOM -> M2
    else if (currentSelection == 3) selectedMotor = 3; // LEFT -> M3

    // Move to page 1
    currentPage = 1;
    currentSelection = 0;
    nextionSend("page 1");
    updateNextionHighlight();
    
  } else { // Page 1
    if (currentSelection == 4) { 
      // "menu" option -> go back to page 0
      currentPage = 0;
      currentSelection = 0;
      nextionSend("page 0");
      updateNextionHighlight();
    } else {
      // Movement options
      int dir = 0;
      if (currentSelection == 0 || currentSelection == 1) dir = 1;  // "0" or "30" -> UP
      if (currentSelection == 2 || currentSelection == 3) dir = -1; // "45" or "60" -> DOWN

      if (selectedMotor > 0 && dir != 0) {
        int motorIdx = selectedMotor - 1;
        udpTargetDir[motorIdx] = dir;
        udpStopTime[motorIdx] = millis() + NEXTION_RUN_MS;
      }
    }
  }
}

// ===================== BUTTON READING =====================
void readButtons() {
  for (int i = 0; i < 6; i++) {
    btnStates[i] = digitalRead(btnPins[i]);
  }
}

int getRequest(bool fwd, bool rev) {
  if (fwd && rev) return 0; // Ignore if both pressed
  if (fwd) return 1;
  if (rev) return -1;
  return 0;
}

// ===================== MOTOR STATE MACHINE =====================
void updateMotor(Motor &m) {
  uint32_t now = millis();
  switch (m.state) {
      case State::STOP:
      m.currentDuty = 0;
      ledcWrite(m.channel, 0);
      if (m.targetDir != 0) {
        int dirVal = (m.targetDir == 1) ? HIGH : LOW;
        digitalWrite(m.dirPin, dirVal);
        
        // Print what the ESP32 is doing
        Serial.printf("[MOTOR] Ch %d | DIR Pin %d set to %s\n", m.channel, m.dirPin, dirVal == HIGH ? "HIGH" : "LOW");
        
        m.state = (m.targetDir == 1) ? State::RUN_FWD : State::RUN_REV;
      }
      break;

    case State::RUN_FWD:
    case State::RUN_REV:
      if (m.currentDuty < MAX_DUTY) {
        m.currentDuty += RAMP_STEP;
        if (m.currentDuty > MAX_DUTY) m.currentDuty = MAX_DUTY;
      }
      ledcWrite(m.channel, m.currentDuty);

      if ((m.targetDir == 0) || 
          (m.targetDir == 1 && m.state == State::RUN_REV) || 
          (m.targetDir == -1 && m.state == State::RUN_FWD)) {
        ledcWrite(m.channel, 0);
        m.currentDuty = 0;
        m.state = State::COAST;
        m.stateStart = now;
      }
      break;

    case State::COAST:
      if (now - m.stateStart >= COAST_MS) {
        m.state = State::LOCKOUT;
        m.stateStart = now;
      }
      break;

    case State::LOCKOUT:
      if (now - m.stateStart >= LOCKOUT_MS) {
        if (m.targetDir == 0) {
          m.state = State::STOP;
        } else {
          int dirVal = (m.targetDir == 1) ? HIGH : LOW;
          digitalWrite(m.dirPin, dirVal);
          
          // Print what the ESP32 is doing
          Serial.printf("[MOTOR] Ch %d | DIR Pin %d set to %s\n", m.channel, m.dirPin, dirVal == HIGH ? "HIGH" : "LOW");
          
          m.state = (m.targetDir == 1) ? State::RUN_FWD : State::RUN_REV;
        }
      }
      break;
  }
}

// ===================== CONFLICT RESOLUTION =====================
void forceStopAll() {
  // Clear UDP timers and targets
  for (int i = 0; i < 3; i++) {
    udpTargetDir[i] = 0;
    udpStopTime[i] = 0;
  }
  
  // Clear Web targets
  webTargetDir[0] = 0;
  webTargetDir[1] = 0;
  webTargetDir[2] = 0;

  // Force state machine to STOP immediately
  Motor* motors[3] = {&m1, &m2, &m3};
  for (int i = 0; i < 3; i++) {
    motors[i]->state = State::STOP;
    motors[i]->currentDuty = 0;
    ledcWrite(motors[i]->channel, 0);
  }
}

// ===================== WEB COMMAND HANDLER =====================
void handleCommand(const String& cmd) {
  // Any web command immediately stops all active movements (Conflict Resolution)
  forceStopAll();

  if (cmd == "m1_fwd")        webTargetDir[0] = 1;
  else if (cmd == "m1_rev")   webTargetDir[0] = -1;
  else if (cmd == "m1_stop")  webTargetDir[0] = 0;
  else if (cmd == "m2_fwd")  { webTargetDir[1] = 1;  Serial.println("[WEB] M2 -> FWD (1)"); }
  else if (cmd == "m2_rev")  { webTargetDir[1] = -1; Serial.println("[WEB] M2 -> REV (-1)"); }
  else if (cmd == "m2_stop") { webTargetDir[1] = 0;  Serial.println("[WEB] M2 -> STOP (0)"); }
  else if (cmd == "m3_left")  webTargetDir[2] = 1;
  else if (cmd == "m3_right") webTargetDir[2] = -1;
  else if (cmd == "m3_stop")  webTargetDir[2] = 0;
  else if (cmd == "stop") {
    webTargetDir[0] = 0;
    webTargetDir[1] = 0;
    webTargetDir[2] = 0;
  }
}

// ===================== HTML PAGE =====================
const char htmlPage[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Motor Control</title>
  <style>
    body {
      font-family: Arial, sans-serif;
      text-align: center;
      background: #1a1a2e;
      color: #eee;
      margin: 0;
      padding: 20px;
    }
    h1 { color: #e94560; }
    .status {
      font-size: 1.2em;
      margin: 15px 0;
      padding: 10px;
      background: #16213e;
      border-radius: 8px;
      color: #e94560;
    }
    .grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 12px;
      max-width: 400px;
      margin: 20px auto;
    }
    button {
      padding: 18px 10px;
      font-size: 1.1em;
      border: none;
      border-radius: 10px;
      cursor: pointer;
      color: #fff;
      font-weight: bold;
      transition: opacity 0.15s;
      user-select: none;
      -webkit-user-select: none;
    }
    button:active { opacity: 0.6; }
    .btn-top    { background: #e94560; }
    .btn-bottom { background: #0f3460; }
    .btn-side   { background: #533483; }
    .btn-stop   { background: #c70039; grid-column: 1 / -1; }
  </style>
</head>
<body>
  <h1>Motor Control</h1>
  <div class="status">Status: <span id="st">IDLE</span></div>
  <div class="grid">
    <button class="btn-top"    ontouchstart="send('m1_fwd')"  onmousedown="send('m1_fwd')"  onmouseup="send('m1_stop')"  ontouchend="send('m1_stop')">TOP UP</button>
    <button class="btn-top"    ontouchstart="send('m1_rev')"  onmousedown="send('m1_rev')"  onmouseup="send('m1_stop')"  ontouchend="send('m1_stop')">TOP DOWN</button>
    <button class="btn-bottom" ontouchstart="send('m2_fwd')"  onmousedown="send('m2_fwd')"  onmouseup="send('m2_stop')"  ontouchend="send('m2_stop')">BOTTOM UP</button>
    <button class="btn-bottom" ontouchstart="send('m2_rev')"  onmousedown="send('m2_rev')"  onmouseup="send('m2_stop')"  ontouchend="send('m2_stop')">BOTTOM DOWN</button>
    <button class="btn-side"   ontouchstart="send('m3_left')" onmousedown="send('m3_left')" onmouseup="send('m3_stop')"  ontouchend="send('m3_stop')">SIDE LEFT</button>
    <button class="btn-side"   ontouchstart="send('m3_right')" onmousedown="send('m3_right')" onmouseup="send('m3_stop')" ontouchend="send('m3_stop')">SIDE RIGHT</button>
    <button class="btn-stop"   onclick="send('stop')">STOP ALL</button>
  </div>
  <script>
    function send(cmd) {
      fetch('/cmd?c=' + cmd)
        .then(r => r.text())
        .then(t => { document.getElementById('st').textContent = t; })
        .catch(() => { document.getElementById('st').textContent = 'ERROR'; });
    }
  </script>
</body>
</html>
)rawliteral";

void handleRoot() {
  server.send_P(200, "text/html", htmlPage);
}

void handleCmd() {
  if (server.hasArg("c")) {
    String cmd = server.arg("c");
    handleCommand(cmd);

    String status = "";
    if (webTargetDir[0] == 1)  status += "M1_UP ";
    else if (webTargetDir[0] == -1) status += "M1_DOWN ";
    if (webTargetDir[1] == 1)  status += "M2_UP ";
    else if (webTargetDir[1] == -1) status += "M2_DOWN ";
    if (webTargetDir[2] == 1)  status += "M3_LEFT ";
    else if (webTargetDir[2] == -1) status += "M3_RIGHT ";
    if (status.length() == 0)  status = "IDLE";

    server.send(200, "text/plain", status);
  } else {
    server.send(400, "text/plain", "Missing command");
  }
}

// ===================== UDP PROCESSING =====================
void processUDP() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char incomingPacket[2] = {0};
    int len = udp.read(incomingPacket, 1);
    if (len > 0) {
      if (incomingPacket[0] == '0') {
        navigateNextion();
      } else if (incomingPacket[0] == '1') {
        selectNextion();
      }
    }
  }
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN); // Nextion Display

  pinMode(DIR_M1, OUTPUT);
  pinMode(DIR_M2, OUTPUT);
  pinMode(DIR_M3, OUTPUT);

  ledcSetup(0, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_M1, 0);
  ledcSetup(1, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_M2, 1);
  ledcSetup(2, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_M3, 2);

  for (int i = 0; i < 6; i++) {
    if (btnPins[i] == 34 || btnPins[i] == 35) {
      pinMode(btnPins[i], INPUT); // Input-only pins — need external 10kΩ pull-down
    } else {
      pinMode(btnPins[i], INPUT_PULLDOWN); // Use internal pull-down
    }
  }

  // Connect WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
  Serial.print("ESP32 IP address: ");
  Serial.println(WiFi.localIP());

  // Start UDP
  udp.begin(UDP_PORT);
  Serial.println("UDP started on port 12345");

  // Start Web Server
  server.on("/", handleRoot);
  server.on("/cmd", handleCmd);
  server.begin();
  Serial.println("Web server started on port 80");

  // Initialize Nextion to Page 0
  nextionSend("page 0");
  updateNextionHighlight();

  Serial.println("3-Motor Controller Ready (Upper, Lower, Side)");
}

// ===================== LOOP =====================
void loop() {
  server.handleClient();
  processUDP();

  readButtons();

  // Check UDP 5-second timers
  uint32_t now = millis();
  for (int i = 0; i < 3; i++) {
    if (udpStopTime[i] > 0 && now >= udpStopTime[i]) {
      udpTargetDir[i] = 0;
      udpStopTime[i] = 0;
    }
  }

  // Combine physical buttons, web commands, and UDP commands
  int btn1 = getRequest(btnStates[0], btnStates[1]);
  int btn2 = getRequest(btnStates[2], btnStates[3]);
  int btn3 = getRequest(btnStates[4], btnStates[5]);

  m1.targetDir = constrain(btn1 + webTargetDir[0] + udpTargetDir[0], -1, 1);
  m2.targetDir = constrain(btn2 + webTargetDir[1] + udpTargetDir[1], -1, 1);
  m3.targetDir = constrain(btn3 + webTargetDir[2] + udpTargetDir[2], -1, 1);

  updateMotor(m1);
  updateMotor(m2);
  updateMotor(m3);
}