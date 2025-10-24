/*
  Electronic Steel Target System — Base Unit
  Board: ESP32‑2432S028R (ESP32 + 2.8" 240x320 TFT over SPI)
  Role: Mesh coordinator / UI hub. Receives hit events from one or more Target Modules
        (ESP32‑C boards with ADXL345/ICM‑40627/piezo) via painlessMesh.
        Displays status and per‑target hit counts, and can arm/disarm and reset.

  Display Library: TFT_eSPI (you must select/enable the ESP32‑2432S028R/ILI9341 setup in User_Setup.h)
  Mesh Library: painlessMesh
  JSON: ArduinoJson (for compact, versioned messages)

  Sensors on Base: optional Hiletgo LM393 vibration sensor (piezo) for local hit detection.

  Message schema (JSON, version 1):
    From Target -> Base:
      {"v":1, "type":"hit", "targetId":"T1", "ts":1680000000, "strength":712}
      {"v":1, "type":"hello", "targetId":"T1", "fw":"1.0.0"}
      {"v":1, "type":"hb", "targetId":"T1", "rssi":-62}
    From Base -> Target(s):
      {"v":1, "type":"cmd", "cmd":"arm"|"disarm"|"reset", "ts":...}

  Mesh SSID/PASS should be identical across base and target modules.
*/

#include <Arduino.h>
#include <painlessMesh.h>
#include <ArduinoJson.h>
#include <TFT_eSPI.h>
#include <SPI.h>

// ========= User Config =========
// Mesh credentials (match these on Target Modules)
static const char* MESH_PREFIX = "SteelTargetsMesh";
static const char* MESH_PASSWORD = "targets123";  // set a stronger password in production
static const uint16_t MESH_PORT = 5555;

// Device identity
static const char* BASE_ID = "BASE01";

// Optional LM393 vibration sensor (digital output). Use an input-only pin (e.g., 34, 35, 36, 39) on ESP32.
// Note: GPIO34 has no internal pullup; LM393 module provides a driven digital output, so that's fine.
#define PIEZO_PIN 34
// Debounce/time between local hit events (ms)
#define LOCAL_HIT_DEBOUNCE_MS 120

// UI buttons (if your 2432S028R variant breaks these out)
// Adjust to match your wiring or set to -1 to disable.
#define BTN_ARM_PIN   13   // e.g., labeled IO13 / key
#define BTN_RESET_PIN 0    // BOOT button often available on the module

// ========== Globals ==========
Scheduler userScheduler;      // to control your personal task
painlessMesh mesh;

TFT_eSPI tft = TFT_eSPI();

// Target state model
struct TargetState {
  String targetId;
  uint32_t hits = 0;
  int lastStrength = 0;
  int rssi = 0;
  uint32_t lastTs = 0;   // last event (ms)
  uint32_t lastSeen = 0; // last any message (ms)
};

// Keep a simple registry by nodeId string (targetId). In production you might map mesh node numbers.
static const int MAX_TARGETS = 16;
TargetState targets[MAX_TARGETS];
int targetCount = 0;

// System state
bool systemArmed = false;
uint32_t lastLocalHitMs = 0;

// UI colors (TFT_eSPI defaults)
#define COL_BG     TFT_BLACK
#define COL_TEXT   TFT_WHITE
#define COL_ACCENT TFT_CYAN
#define COL_WARN   TFT_YELLOW
#define COL_ARMED  TFT_GREEN
#define COL_DISARM TFT_RED

// Forward decls
void drawUI();
void drawHeader();
void drawTargets();
void processInbound(const String& msg, uint32_t from);
void sendCommandAll(const char* cmd);
TargetState* getOrCreateTarget(const String& id);

// ===== Tasks =====
Task taskHeartbeat(3000, TASK_FOREVER, [](){
  // Send a base heartbeat so new targets see we exist
  StaticJsonDocument<128> doc;
  doc["v"] = 1;
  doc["type"] = "hb";
  doc["targetId"] = BASE_ID; // reusing key for simplicity; receivers can treat type=hb specially
  String out;
  serializeJson(doc, out);
  mesh.sendBroadcast(out);
});

Task taskRedraw(250, TASK_FOREVER, [](){
  drawUI();
});

// ========== Setup ==========
void setup() {
  // Basic IO
  pinMode(PIEZO_PIN, INPUT); // LM393 digital out
  if (BTN_ARM_PIN >= 0) pinMode(BTN_ARM_PIN, INPUT_PULLUP);
  if (BTN_RESET_PIN >= 0) pinMode(BTN_RESET_PIN, INPUT_PULLUP);

  // Serial
  Serial.begin(115200);
  delay(200);
  Serial.println("Base Unit booting...");

  // Display
  tft.init();
  tft.setRotation(1); // landscape
  tft.fillScreen(COL_BG);
  tft.setTextColor(COL_TEXT, COL_BG);
  tft.setTextDatum(TL_DATUM);
  tft.setFreeFont(nullptr); // use default

  tft.drawString("Electronic Target System", 8, 8, 2);
  tft.drawString("Base initializing...", 8, 28, 2);

  // Mesh init
  mesh.setDebugMsgTypes(ERROR | STARTUP); // | CONNECTION | COMMUNICATION
  mesh.init(MESH_PREFIX, MESH_PASSWORD, MESH_PORT, WIFI_AP_STA, 6);

  mesh.onReceive([](uint32_t from, String &msg){
    processInbound(msg, from);
  });

  mesh.onNewConnection([](uint32_t nodeId){
    Serial.printf("New connection: %u\n", nodeId);
  });

  mesh.onChangedConnections([](){
    Serial.printf("Mesh topology changed. Size=%d\n", mesh.getNodeList().size());
  });

  // Scheduler tasks
  userScheduler.init();
  userScheduler.addTask(taskHeartbeat);
  userScheduler.addTask(taskRedraw);
  taskHeartbeat.enable();
  taskRedraw.enable();

  // First paint
  drawUI();
}

// ========== Loop ==========
void loop() {
  mesh.update();
  userScheduler.execute();

  // Buttons (active LOW)
  if (BTN_ARM_PIN >= 0 && digitalRead(BTN_ARM_PIN) == LOW) {
    static uint32_t lastBtnMs = 0;
    if (millis() - lastBtnMs > 250) {
      systemArmed = !systemArmed;
      sendCommandAll(systemArmed ? "arm" : "disarm");
      lastBtnMs = millis();
    }
  }
  if (BTN_RESET_PIN >= 0 && digitalRead(BTN_RESET_PIN) == LOW) {
    static uint32_t lastBtnMs2 = 0;
    if (millis() - lastBtnMs2 > 400) {
      // Reset counts
      for (int i=0;i<targetCount;i++) targets[i].hits = 0;
      sendCommandAll("reset");
      lastBtnMs2 = millis();
    }
  }

  // Local hit detection (optional)
  if (systemArmed) {
    int v = digitalRead(PIEZO_PIN); // LM393: HIGH on vibration (adjust module pot as needed)
    uint32_t now = millis();
    if (v == HIGH && (now - lastLocalHitMs) > LOCAL_HIT_DEBOUNCE_MS) {
      lastLocalHitMs = now;
      // Log as synthetic target "LOCAL"
      TargetState* t = getOrCreateTarget("LOCAL");
      if (t) {
        t->hits++;
        t->lastStrength = 1023; // not measured; placeholder
        t->lastTs = now;
        t->lastSeen = now;
      }
      // Broadcast local event so targets (e.g., for synchronized LEDs) can react if desired
      StaticJsonDocument<160> doc;
      doc["v"] = 1;
      doc["type"] = "hit";
      doc["targetId"] = "LOCAL";
      doc["ts"] = (uint32_t)(now);
      doc["strength"] = 1023;
      String out; serializeJson(doc, out);
      mesh.sendBroadcast(out);
    }
  }
}

// ========== Messaging ==========
void processInbound(const String& msg, uint32_t from) {
  Serial.printf("<- %u: %s\n", from, msg.c_str());
  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, msg);
  if (err) {
    Serial.printf("JSON parse error: %s\n", err.c_str());
    return;
  }
  int v = doc["v"] | 0;
  const char* type = doc["type"] | "";
  uint32_t now = millis();

  if (v != 1) return; // ignore unknown versions for now

  if (strcmp(type, "hello") == 0) {
    const char* tid = doc["targetId"] | "?";
    TargetState* t = getOrCreateTarget(String(tid));
    if (t) t->lastSeen = now;
    // Optionally auto-arm state sync
    StaticJsonDocument<128> reply;
    reply["v"] = 1;
    reply["type"] = "cmd";
    reply["cmd"] = systemArmed ? "arm" : "disarm";
    String out; serializeJson(reply, out);
    mesh.sendSingle(from, out);
  }
  else if (strcmp(type, "hb") == 0) {
    const char* tid = doc["targetId"] | "?";
    int rssi = doc["rssi"] | 0;
    TargetState* t = getOrCreateTarget(String(tid));
    if (t) {
      t->rssi = rssi;
      t->lastSeen = now;
    }
  }
  else if (strcmp(type, "hit") == 0) {
    const char* tid = doc["targetId"] | "?";
    int strength = doc["strength"] | 0;
    TargetState* t = getOrCreateTarget(String(tid));
    if (t) {
      t->hits++;
      t->lastStrength = strength;
      t->lastTs = now;
      t->lastSeen = now;
    }
  }
}

void sendCommandAll(const char* cmd) {
  StaticJsonDocument<128> doc;
  doc["v"] = 1;
  doc["type"] = "cmd";
  doc["cmd"] = cmd;
  doc["ts"] = (uint32_t)millis();
  String out; serializeJson(doc, out);
  mesh.sendBroadcast(out);
}

TargetState* getOrCreateTarget(const String& id) {
  // check existing
  for (int i=0;i<targetCount;i++) {
    if (targets[i].targetId == id) return &targets[i];
  }
  if (targetCount >= MAX_TARGETS) return nullptr;
  targets[targetCount].targetId = id;
  targets[targetCount].hits = 0;
  targets[targetCount].lastStrength = 0;
  targets[targetCount].rssi = 0;
  targets[targetCount].lastTs = 0;
  targets[targetCount].lastSeen = millis();
  targetCount++;
  return &targets[targetCount-1];
}

// ========== UI ==========
void drawUI() {
  static bool first = true;
  static bool lastArmed = !systemArmed;

  if (first || lastArmed != systemArmed) {
    tft.fillScreen(COL_BG);
    drawHeader();
    first = false;
    lastArmed = systemArmed;
  }
  drawTargets();
}

void drawHeader() {
  tft.fillRect(0,0,320,30, COL_BG);
  tft.setTextColor(COL_TEXT, COL_BG);
  tft.drawString("Base:" + String(BASE_ID), 6, 6, 2);
  // Armed badge
  uint16_t col = systemArmed ? COL_ARMED : COL_DISARM;
  tft.fillRoundRect(200, 5, 110, 20, 6, col);
  tft.setTextColor(TFT_BLACK, col);
  tft.drawString(systemArmed ? "ARMED" : "DISARMED", 215, 9, 2);
}

void drawTargets() {
  // Column headers
  int y0 = 34;
  tft.setTextColor(COL_ACCENT, COL_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("ID", 6, y0, 2);
  tft.drawString("HITS", 70, y0, 2);
  tft.drawString("STR", 130, y0, 2);
  tft.drawString("RSSI", 180, y0, 2);
  tft.drawString("SEEN", 240, y0, 2);

  // Rows
  int y = y0 + 18;
  uint32_t now = millis();

  // Show up to 10 rows (scrolling/paging could be added later)
  for (int i = 0; i < min(targetCount, 10); i++) {
    TargetState &t = targets[i];

    // Row background wipe to avoid ghosting
    tft.fillRect(0, y-2, 320, 16, COL_BG);

    tft.setTextColor(COL_TEXT, COL_BG);
    tft.drawString(t.targetId, 6, y, 2);
    tft.drawString(String(t.hits), 70, y, 2);
    tft.drawString(String(t.lastStrength), 130, y, 2);
    tft.drawString(String(t.rssi), 180, y, 2);

    uint32_t age = now - t.lastSeen;
    String ageStr;
    if (age < 1000) ageStr = "now";
    else if (age < 60000) ageStr = String(age/1000) + "s";
    else ageStr = String(age/60000) + "m";
    tft.drawString(ageStr, 240, y, 2);

    y += 16;
  }
}

/*
  ===== Build Notes =====
  1) Libraries:
     - painlessMesh by gmag11
     - ArduinoJson (v6+)
     - TFT_eSPI by Bodmer (configure User_Setup for ESP32‑2432S028R / ILI9341)

  2) Pinning for ESP32‑2432S028R display is handled by TFT_eSPI setup; typical SPI:
       TFT_MOSI=23, TFT_MISO=19, TFT_SCLK=18, TFT_CS=5, TFT_DC=27, TFT_RST=33, TFT_BL=32
     Verify your specific board silkscreen and set in TFT_eSPI settings.

  3) LM393 vibration: connect module VCC (3.3V), GND, DO -> GPIO34.
     Adjust module potentiometer to tune sensitivity. If your module outputs LOW on vibration,
     invert the digitalRead logic accordingly.

  4) Targets should send JSON messages as specified; example Arduino (target) payload for a hit:
       {"v":1, "type":"hit", "targetId":"T1", "ts":millis(), "strength":amplitude}

  5) Security: In production, use better mesh credentials and consider message signing / sequence numbers.

  6) Extensions you can add quickly:
     - Paging for >10 targets
     - Per‑target RGB feedback command return
     - SD logging via VSPI
     - BLE or Wi‑Fi captive portal for settings
*/
