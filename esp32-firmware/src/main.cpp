#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Bluepad32.h>

// ─────────────────────────────────────────────
//  Configuration
// ─────────────────────────────────────────────
const char* ssid        = "YOUR_WIFI_SSID";
const char* password    = "YOUR_WIFI_PASSWORD";
const char* mqtt_server = "YOUR_MQTT_BROKER_IP";   // e.g. "192.168.1.53"

// MQTT topics
#define TOPIC_SYSTEM   "quadpod/system"
#define TOPIC_LOGS     "quadpod/logs"
#define TOPIC_GAMEPAD  "quadpod/gamepad"

// ─────────────────────────────────────────────
//  Globals
// ─────────────────────────────────────────────
WiFiClient   espClient;
PubSubClient mqttClient(espClient);

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// Tracks previous button state to detect rising edges (press, not hold)
uint16_t prevButtons[BP32_MAX_GAMEPADS] = {0};
uint8_t  prevDpad[BP32_MAX_GAMEPADS]    = {0};
uint8_t  prevMisc[BP32_MAX_GAMEPADS]    = {0};

// ─────────────────────────────────────────────
//  WiFi
// ─────────────────────────────────────────────
void setup_wifi() {
    Serial.print("Connecting to WiFi");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected: " + WiFi.localIP().toString());
}

// ─────────────────────────────────────────────
//  MQTT
// ─────────────────────────────────────────────
void mqtt_reconnect() {
    while (!mqttClient.connected()) {
        Serial.print("Connecting to MQTT...");
        if (mqttClient.connect("ESP32_Quadpod")) {
            Serial.println("connected");
            mqttClient.publish(TOPIC_SYSTEM, "ESP32 connected");
        } else {
            Serial.printf("failed (rc=%d), retrying in 2s\n", mqttClient.state());
            delay(2000);
        }
    }
}

// Helper: publish a string message
void publish(const char* topic, const String& msg) {
    mqttClient.publish(topic, msg.c_str());
    Serial.println("[MQTT] " + String(topic) + " → " + msg);
}

// ─────────────────────────────────────────────
//  Bluepad32 Callbacks
// ─────────────────────────────────────────────
void onConnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            myControllers[i] = ctl;
            Serial.printf("Gamepad connected on slot %d\n", i);
            publish(TOPIC_SYSTEM, "gamepad_connected:slot" + String(i));
            return;
        }
    }
    Serial.println("No free gamepad slot — rejecting controller");
    ctl->disconnect();
}

void onDisconnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            myControllers[i]  = nullptr;
            prevButtons[i]    = 0;
            prevDpad[i]       = 0;
            prevMisc[i]    = 0;
            Serial.printf("Gamepad disconnected from slot %d\n", i);
            publish(TOPIC_SYSTEM, "gamepad_disconnected:slot" + String(i));
            return;
        }
    }
}

// ─────────────────────────────────────────────
//  Gamepad Button → MQTT Action Mapping
//
//  Edit this function to define what each button does.
//  'slot' identifies which controller (0 = first paired, etc.)
//  Only fires on PRESS (rising edge), not while held.
// ─────────────────────────────────────────────
void handleButtonPress(int slot, const String& button) {
    String msg = "slot" + String(slot) + ":" + button;

    // ── Define actions for each button ──────────────────────────────────
    if      (button == "DPAD_UP")    publish(TOPIC_GAMEPAD, "move:forward");
    else if (button == "DPAD_DOWN")  publish(TOPIC_GAMEPAD, "move:backward");
    else if (button == "DPAD_LEFT")  publish(TOPIC_GAMEPAD, "move:left");
    else if (button == "DPAD_RIGHT") publish(TOPIC_GAMEPAD, "move:right");

    else if (button == "A" || button == "CROSS")    publish(TOPIC_GAMEPAD, "action:A");
    else if (button == "B" || button == "CIRCLE")   publish(TOPIC_GAMEPAD, "action:B");
    else if (button == "X" || button == "SQUARE")   publish(TOPIC_GAMEPAD, "action:X");
    else if (button == "Y" || button == "TRIANGLE")  publish(TOPIC_GAMEPAD, "action:Y");

    else if (button == "L1")  publish(TOPIC_GAMEPAD, "action:L1");
    else if (button == "R1")  publish(TOPIC_GAMEPAD, "action:R1");
    else if (button == "L2")  publish(TOPIC_GAMEPAD, "action:L2");
    else if (button == "R2")  publish(TOPIC_GAMEPAD, "action:R2");

    else if (button == "START")  publish(TOPIC_GAMEPAD, "action:start");
    else if (button == "SELECT") publish(TOPIC_GAMEPAD, "action:select");
    else if (button == "HOME")   publish(TOPIC_GAMEPAD, "action:home");

    else {
        // Unknown / unmapped button — still publish for debugging
        publish(TOPIC_GAMEPAD, "btn:" + button);
    }
}

// ─────────────────────────────────────────────
//  Process one controller per loop iteration
// ─────────────────────────────────────────────
void processGamepad(int slot, ControllerPtr ctl) {
    // ── Regular buttons (rising edge only) ──────────────────────────────
    uint16_t buttons    = ctl->buttons();
    uint16_t newButtons = buttons & ~prevButtons[slot];
    prevButtons[slot]   = buttons;

    const struct { uint16_t mask; const char* name; } BTN_MAP[] = {
        { BUTTON_A,          "A"       },
        { BUTTON_B,          "B"       },
        { BUTTON_X,          "X"       },
        { BUTTON_Y,          "Y"       },
        { BUTTON_SHOULDER_L, "L1"      },   // ← fixed
        { BUTTON_SHOULDER_R, "R1"      },   // ← fixed
        { BUTTON_TRIGGER_L,  "L2"      },   // ← fixed
        { BUTTON_TRIGGER_R,  "R2"      },   // ← fixed
        { BUTTON_THUMB_L,    "THUMB_L" },
        { BUTTON_THUMB_R,    "THUMB_R" },
    };
    for (auto& b : BTN_MAP) {
        if (newButtons & b.mask) handleButtonPress(slot, b.name);
    }

    // ── Misc buttons (Start, Select, Home, System) ───────────────────────
    // These are on a SEPARATE bitmask: ctl->miscButtons()
    uint8_t misc    = ctl->miscButtons();
    uint8_t newMisc = misc & ~prevMisc[slot];
    prevMisc[slot]  = misc;

    const struct { uint8_t mask; const char* name; } MISC_MAP[] = {
        { MISC_BUTTON_START,  "START"  },   // ← fixed
        { MISC_BUTTON_SELECT, "SELECT" },   // ← fixed
        { MISC_BUTTON_HOME,   "HOME"   },   // ← fixed
        { MISC_BUTTON_SYSTEM, "SYSTEM" },   // ← fixed
    };
    for (auto& m : MISC_MAP) {
        if (newMisc & m.mask) handleButtonPress(slot, m.name);
    }

    // ── D-pad (rising edge only) ─────────────────────────────────────────
    uint8_t dpad    = ctl->dpad();
    uint8_t newDpad = dpad & ~prevDpad[slot];
    prevDpad[slot]  = dpad;

    const struct { uint8_t mask; const char* name; } DPAD_MAP[] = {
        { DPAD_UP,    "DPAD_UP"    },
        { DPAD_DOWN,  "DPAD_DOWN"  },
        { DPAD_LEFT,  "DPAD_LEFT"  },
        { DPAD_RIGHT, "DPAD_RIGHT" },
    };
    for (auto& d : DPAD_MAP) {
        if (newDpad & d.mask) handleButtonPress(slot, d.name);
    }
}

// ─────────────────────────────────────────────
//  Arduino setup / loop
// ─────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    setup_wifi();

    mqttClient.setServer(mqtt_server, 1883);

    // Bluepad32 setup
    BP32.setup(&onConnectedController, &onDisconnectedController);

    // Optional: forget previously paired devices so you always pair fresh
    // BP32.forgetBluetoothKeys();

    Serial.println("Ready. Put your gamepad in pairing mode.");
}

void loop() {
    // ── Keep MQTT alive ──────────────────────────────────────────────────
    if (!mqttClient.connected()) {
        mqtt_reconnect();
    }
    mqttClient.loop();

    // ── Poll Bluepad32 ───────────────────────────────────────────────────
    // Returns true if at least one controller sent new data this tick
    bool dataUpdated = BP32.update();

    if (dataUpdated) {
        for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
            ControllerPtr ctl = myControllers[i];
            if (ctl && ctl->isConnected() && ctl->isGamepad()) {
                processGamepad(i, ctl);
            }
        }
    }

    // ── USB serial bridge (unchanged) ────────────────────────────────────
    if (Serial.available() > 0) {
        String receivedData = Serial.readStringUntil('\n');
        receivedData.trim();
        Serial.println("Received from Android: " + receivedData);
        mqttClient.publish(TOPIC_LOGS, receivedData.c_str());
    }
}