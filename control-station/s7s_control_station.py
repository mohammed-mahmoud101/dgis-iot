#!/usr/bin/env python3
"""
s7s_control_station.py — Laptop control station for the S7S Quadrupod.

Architecture
------------
    [Xbox controller] ── Bluetooth (Bluepad32) ─►  [ESP32 + PCA9685 + 12 servos]
                                                          ▲
                                                          │  (USB serial, JSON lines)
                                                          │
    [Laptop user]  ────── clicks UI ──────►  [Tkinter app]
                                                 │
                                                 ▼
                                          [Embedded MQTT broker]
                                                 │  (WiFi)
                                                 ▼
                                          [Phone (USB↔MQTT bridge)]
                                                 │  (USB serial, JSON lines)
                                                 ▼
                                          [ESP32 + PCA9685 + 12 servos]

    The Xbox controller connects directly to the ESP32 via Bluetooth (Bluepad32).
    Priority is handled on the ESP32: laptop MQTT commands override Xbox for 2
    seconds. The laptop app does NOT touch the Xbox — it only displays the
    priority state reported by the ESP32 in telemetry/status.

Priority
--------
    Priority arbitration lives entirely on the ESP32 firmware. The laptop merely
    sends commands over MQTT; the ESP32 decides whether each command should
    override the Xbox (it does, for MANUAL_OVERRIDE_S seconds after any laptop
    command). The ESP32 reports the current priority state back to the laptop
    via the status topic (mqttOverrideActive, controllerConnected).

MQTT topics
-----------
    Commands (laptop → phone → ESP32):
        quadpod/cmd/gait                    payload: gait name (str) or "stand"
        quadpod/cmd/speed                   payload: float (0.1 – 3.0)
        quadpod/cmd/estop                   payload: "1"
        quadpod/cmd/calib/motor/start       payload: (empty)
        quadpod/cmd/calib/motor/set         payload: {"joint":int,"offset":float}
        quadpod/cmd/calib/motor/save        payload: (empty)
        quadpod/cmd/calib/motor/load        payload: (empty)
        quadpod/cmd/calib/motor/end         payload: (empty)
        quadpod/cmd/calib/gyro/set          payload: {"yaw":f,"pitch":f,"roll":f}
        quadpod/cmd/calib/gyro/clear        payload: (empty)
        quadpod/cmd/calib/gyro/save         payload: (empty)
        quadpod/cmd/ping                    payload: (empty)

    State (ESP32 → phone → laptop):
        quadpod/state/telemetry             payload: {"gait","gaitTime","pose[12]","offsets[12]","gyro_calibrated","calibMode","calibJoint"}
        quadpod/state/gyro                  payload: {"yaw","pitch","roll","valid"}   (raw, from phone)
        quadpod/state/status                payload: {"esp32Connected","calibMode","uptime"}
        quadpod/log                         payload: {"level","msg","t"}

Usage
-----
    python s7s_control_station.py
    python s7s_control_station.py --broker-port 1883
    python s7s_control_station.py --external-mqtt 192.168.1.10:1883
"""

from __future__ import annotations

import argparse
import json
import logging
import socket
import sys
import threading
import time
import queue
from dataclasses import dataclass, field
from typing import Any, Callable, Optional

import tkinter as tk
from tkinter import ttk, messagebox

# ─── Optional deps ────────────────────────────────────────────────────────────
try:
    import paho.mqtt.client as mqtt  # type: ignore
    PAHO_OK = True
except ImportError:
    PAHO_OK = False

try:
    from amqtt.broker import Broker  # type: ignore
    AMQTT_OK = True
except ImportError:
    AMQTT_OK = False

# pygame/Xbox support removed — the Xbox controller now connects directly to
# the ESP32 via Bluetooth (Bluepad32). The laptop no longer polls it.
PYGAME_OK = False

try:
    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    MPL_OK = True
except ImportError:
    MPL_OK = False


# =============================================================================
#  Configuration
# =============================================================================

DEFAULT_BROKER_PORT = 1883
DEFAULT_BROKER_HOST = "192.168.1.12"   # the laptop itself
MQTT_CLIENT_ID = "s7s_control_station"
TELEMETRY_RATE_HZ = 20              # how often we *request* telemetry from ESP32
# Reference only — the actual override window is enforced on the ESP32
# firmware (laptop MQTT commands override Xbox for this many seconds).
MANUAL_OVERRIDE_S = 2.0
JOINT_LIMIT = 1.5                   # rad — clamp range for calibration sliders (~±86°)

JOINT_LABELS = [
    "FR_abd", "FL_abd", "RL_abd", "RR_abd",
    "FR_flex", "FL_flex", "RL_flex", "RR_flex",
    "FR_knee", "FL_knee", "RL_knee", "RR_knee",
]
JOINT_GROUPS = [
    ("Abductor",  ["FR_abd", "FL_abd", "RL_abd", "RR_abd"]),
    ("Flex",      ["FR_flex", "FL_flex", "RL_flex", "RR_flex"]),
    ("Knee",      ["FR_knee", "FL_knee", "RL_knee", "RR_knee"]),
]

GAITS = [
    ("stand",         "Stand / Idle"),
    ("trot_forward",  "Trot Forward"),
    ("trot_backward", "Trot Backward"),
    ("turn_left",     "Turn Left"),
    ("turn_right",    "Turn Right"),
    ("arc_left",      "Arc Left"),
    ("arc_right",     "Arc Right"),
]

# =============================================================================
#  MQTT topic registry — single source of truth, shared by all components
# =============================================================================

class Topics:
    # Commands (laptop → ESP32, via phone bridge)
    CMD_GAIT            = "quadpod/cmd/gait"
    CMD_SPEED           = "quadpod/cmd/speed"
    CMD_ESTOP           = "quadpod/cmd/estop"
    CMD_CALIB_MOTOR_START = "quadpod/cmd/calib/motor/start"
    CMD_CALIB_MOTOR_SET   = "quadpod/cmd/calib/motor/set"
    CMD_CALIB_MOTOR_SAVE  = "quadpod/cmd/calib/motor/save"
    CMD_CALIB_MOTOR_LOAD  = "quadpod/cmd/calib/motor/load"
    CMD_CALIB_MOTOR_RESET = "quadpod/cmd/calib/motor/reset"
    CMD_CALIB_MOTOR_END   = "quadpod/cmd/calib/motor/end"
    CMD_CALIB_GYRO_SET    = "quadpod/cmd/calib/gyro/set"
    CMD_CALIB_GYRO_CLEAR  = "quadpod/cmd/calib/gyro/clear"
    CMD_CALIB_GYRO_SAVE   = "quadpod/cmd/calib/gyro/save"
    CMD_PING             = "quadpod/cmd/ping"

    # State (ESP32 → laptop, via phone bridge)
    STATE_TELEMETRY     = "quadpod/state/telemetry"
    STATE_GYRO          = "quadpod/state/gyro"
    STATE_STATUS        = "quadpod/state/status"
    LOG                 = "quadpod/log"

    # Wildcard the phone bridge subscribes to (kept here for parity / docs)
    CMD_WILDCARD        = "quadpod/cmd/#"

    ALL_SUBSCRIBED = [
        STATE_TELEMETRY, STATE_GYRO, STATE_STATUS, LOG,
    ]


# =============================================================================
#  Logging — both to console and into the in-app Log tab
# =============================================================================

class HubLogHandler(logging.Handler):
    """Forwards log records to a queue so the Tkinter Log tab can drain it."""
    def __init__(self, q: queue.Queue):
        super().__init__()
        self.q = q
    def emit(self, record):
        try:
            self.q.put_nowait(record)
        except queue.Full:
            pass


def setup_logging(log_q: queue.Queue) -> logging.Logger:
    logger = logging.getLogger("s7s")
    logger.setLevel(logging.INFO)
    logger.handlers.clear()

    console = logging.StreamHandler(sys.stdout)
    console.setFormatter(logging.Formatter("%(asctime)s [%(levelname)s] %(message)s",
                                            "%H:%M:%S"))
    logger.addHandler(console)

    hub = HubLogHandler(log_q)
    hub.setFormatter(logging.Formatter("%(asctime)s [%(levelname)s] %(message)s", "%H:%M:%S"))
    logger.addHandler(hub)
    return logger


# =============================================================================
#  Thread-safe state snapshot — all UI tabs read from this
# =============================================================================

@dataclass
class TelemetrySnapshot:
    gait: str = "stand"
    gait_time: float = 0.0
    pose: list = field(default_factory=lambda: [0.0] * 12)
    offsets: list = field(default_factory=lambda: [0.0] * 12)
    gyro_calibrated: dict = field(default_factory=lambda: {"yaw": 0.0, "pitch": 0.0, "roll": 0.0})
    calib_mode: bool = False
    calib_joint: int = 0
    timestamp: float = 0.0

@dataclass
class GyroSnapshot:
    yaw: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0
    valid: bool = False
    timestamp: float = 0.0

@dataclass
class StatusSnapshot:
    esp32_connected: bool = False
    calib_mode: bool = False
    uptime: float = 0.0
    # ESP32-reported priority state (Xbox now connects directly to ESP32):
    mqttOverrideActive: bool = False     # laptop MQTT commands currently overriding Xbox
    controllerConnected: bool = False    # Xbox paired to the ESP32 via Bluepad32
    timestamp: float = 0.0


class SharedState:
    """Single shared object protected by a lock. UI reads, MQTT writes."""
    def __init__(self, log: logging.Logger):
        self.log = log
        self.lock = threading.RLock()
        self.telemetry = TelemetrySnapshot()
        self.gyro = GyroSnapshot()
        self.status = StatusSnapshot()
        self.broker_running = False
        self.gyro_offset_local = {"yaw": 0.0, "pitch": 0.0, "roll": 0.0}  # local copy
        # ─── ESP32 connection tracking ─────────────────────────────────
        # We consider the ESP32 "connected" if we've received ANY message
        # from it (telemetry/status/log/gyro) in the last 5 seconds. This is
        # more reliable than the ESP32's own esp32Connected field (which
        # actually means "phone USB link alive").
        self.last_esp32_rx = 0.0   # time.time() of last message FROM the ESP32
        self.esp32_seen = False    # has the ESP32 ever sent us anything?

    def is_manual_override_active(self) -> bool:
        # Priority arbitration moved to the ESP32 firmware. The laptop no
        # longer tracks its own override window — always returns False so any
        # legacy callers behave as if the laptop has no override authority.
        return False

    def arm_manual_override(self, seconds: float = MANUAL_OVERRIDE_S):
        # No-op: override is now enforced on the ESP32. Call sites in the
        # calibration/controls tabs are intentionally left in place (it's
        # cleaner than surgically removing ~10 of them) but do nothing.
        pass

    def update_telemetry(self, payload: dict):
        with self.lock:
            t = self.telemetry
            t.gait = payload.get("gait", t.gait)
            t.gait_time = payload.get("gaitTime", t.gait_time)
            if "pose" in payload and isinstance(payload["pose"], list):
                t.pose = list(payload["pose"])[:12]
            if "offsets" in payload and isinstance(payload["offsets"], list):
                t.offsets = list(payload["offsets"])[:12]
            if "gyro_calibrated" in payload:
                t.gyro_calibrated = payload["gyro_calibrated"]
            t.calib_mode = bool(payload.get("calibMode", t.calib_mode))
            t.calib_joint = int(payload.get("calibJoint", t.calib_joint))
            t.timestamp = time.time()
            self.last_esp32_rx = time.time()
            self.esp32_seen = True

    def update_gyro(self, payload: dict):
        with self.lock:
            self.gyro.yaw = float(payload.get("yaw", 0.0))
            self.gyro.pitch = float(payload.get("pitch", 0.0))
            self.gyro.roll = float(payload.get("roll", 0.0))
            self.gyro.valid = bool(payload.get("valid", False))
            self.gyro.timestamp = time.time()
            self.last_esp32_rx = time.time()
            self.esp32_seen = True

    def update_status(self, payload: dict):
        with self.lock:
            # esp32Connected in the payload means "phone USB link alive" on the
            # ESP32 side. We keep it for display but use last_esp32_rx for our
            # own connection detection.
            self.status.esp32_connected = bool(payload.get("esp32Connected", False))
            self.status.calib_mode = bool(payload.get("calibMode", False))
            self.status.uptime = float(payload.get("uptime", 0.0))
            self.status.mqttOverrideActive = bool(payload.get("mqttOverrideActive", False))
            self.status.controllerConnected = bool(payload.get("controllerConnected", False))
            self.status.timestamp = time.time()
            self.last_esp32_rx = time.time()
            self.esp32_seen = True

    def is_esp32_connected(self) -> bool:
        """True if we've heard from the ESP32 in the last 5 seconds."""
        if not self.esp32_seen:
            return False
        return (time.time() - self.last_esp32_rx) < 5.0

    def snapshot(self) -> dict:
        with self.lock:
            return {
                "telemetry": self.telemetry.__dict__.copy(),
                "gyro": self.gyro.__dict__.copy(),
                "status": self.status.__dict__.copy(),
                "broker_running": self.broker_running,
            }


# =============================================================================
#  Embedded MQTT broker (amqtt) — runs in a background asyncio thread
# =============================================================================

# amqtt ships its own asyncio-compatible broker config. This is the minimal
# listener config — anonymous access, no TLS, no auth.
# New amqtt: auth/topic-check/sys_interval are deprecated. Anonymous access
# is allowed by default when no auth plugin is configured.
AMQTT_BROKER_CONFIG = {
    "listeners": {
        "default": {
            "type": "tcp",
            "bind": f"0.0.0.0:{DEFAULT_BROKER_PORT}",
            "max_connections": 10,
        }
    },
}


class EmbeddedBroker:
    """Runs an amqtt broker inside the Python app. Zero external setup."""

    def __init__(self, port: int, log: logging.Logger):
        self.port = port
        self.log = log
        self._thread: Optional[threading.Thread] = None
        self._loop: Any = None
        self._broker: Any = None
        self._started = threading.Event()
        self._stop = threading.Event()

    def start(self) -> bool:
        if not AMQTT_OK:
            self.log.error("amqtt not installed — embedded broker disabled. "
                           "Install with: pip install amqtt")
            return False

        cfg = {
            "listeners": {
                "default": {
                    "type": "tcp",
                    "bind": f"0.0.0.0:{self.port}",
                    "max_connections": 10,
                }
            },
            # New amqtt config: auth/topic-check/sys_interval are deprecated.
            # Anonymous access is allowed by default when no auth plugin is
            # configured, so we just omit those keys entirely.
        }

        async def _run():
            import asyncio
            self._loop = asyncio.get_running_loop()
            self._broker = Broker(cfg)
            await self._broker.start()
            self._started.set()
            self.log.info(f"Embedded MQTT broker listening on 0.0.0.0:{self.port}")
            try:
                while not self._stop.is_set():
                    await asyncio.sleep(0.5)
            finally:
                await self._broker.shutdown()
                self.log.info("Embedded MQTT broker shut down")

        def _entry():
            import asyncio
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            try:
                loop.run_until_complete(_run())
            except Exception as e:
                self.log.error(f"Broker thread crashed: {e}")

        self._thread = threading.Thread(target=_entry, name="amqtt-broker", daemon=True)
        self._thread.start()
        # Wait up to 3s for the broker to come up
        return self._started.wait(timeout=3.0)

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=3.0)


# =============================================================================
#  MQTT client — publishes commands, subscribes to telemetry
# =============================================================================

class MqttHub:
    """Wraps paho-mqtt. Thread-safe publish + callback-based subscribe."""

    def __init__(self, host: str, port: int, state: SharedState,
                 log: logging.Logger, on_msg: Optional[Callable[[str, str], None]] = None):
        if not PAHO_OK:
            raise RuntimeError("paho-mqtt not installed. Run: pip install paho-mqtt")
        self.host = host
        self.port = port
        self.state = state
        self.log = log
        self.on_msg = on_msg
        self._client: Optional[mqtt.Client] = None
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        # ─── Diagnostic counters ─────────────────────────────────────
        # Track how many messages we've received FROM the ESP32, so the
        # UI can display "MQTT rx: N msgs" and the user can immediately
        # see if the broker is routing messages correctly.
        self.rx_count = 0           # total messages received
        self.rx_count_from_esp32 = 0  # messages on quadpod/state/* or quadpod/log
        self.first_rx_logged = False  # log the first ESP32 message for confirmation
        self.self_test_passed = False      # same-client routing (laptop → laptop)
        self.cross_test_passed = False     # cross-client routing (ESP32 → laptop)
        self.cross_test_pending = False    # waiting for pong from ESP32
        self.cross_test_sent_time = 0.0    # when we sent the ping
        self.verbose_mqtt = False          # log every MQTT message
        self._msg_log_count = 0            # for limiting verbose log spam
        self._topic_first_seen = {}        # topic → bool (logged first message?)

    # ── lifecycle ────────────────────────────────────────────────────────
    def start(self) -> bool:
        # paho-mqtt v2 changed the callback API. Use VERSION2 if available,
        # fall back to v1 for older paho-mqtt installs.
        try:
            self._client = mqtt.Client(
                callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
                client_id=MQTT_CLIENT_ID,
                clean_session=True,
            )
        except (AttributeError, TypeError):
            # paho-mqtt v1 (no callback_api_version param)
            self._client = mqtt.Client(client_id=MQTT_CLIENT_ID, clean_session=True)
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message
        self._client.on_disconnect = self._on_disconnect

        try:
            self._client.connect(self.host, self.port, keepalive=60)
        except Exception as e:
            self.log.error(f"MQTT connect failed: {e}")
            return False

        self._thread = threading.Thread(target=self._client.loop_forever,
                                         name="mqtt-loop", daemon=True)
        self._thread.start()
        return True

    def stop(self):
        self._stop.set()
        if self._client:
            self._client.loop_stop()
            self._client.disconnect()

    # ── paho callbacks ───────────────────────────────────────────────────
    # Use *args so the same callback works with paho-mqtt v1 AND v2:
    #   v1: on_connect(client, userdata, flags, rc)              → 4 args
    #   v2: on_connect(client, userdata, flags, reason, props)   → 5 args
    #   v1: on_disconnect(client, userdata, rc)                  → 3 args
    #   v2: on_disconnect(client, userdata, flags, reason, props)→ 5 args
    def _on_connect(self, client, userdata, *args):
        # rc / reason_code is always at args[1] in both v1 and v2
        rc = args[1] if len(args) >= 2 else 0
        if rc == 0 or str(rc) == "Success":
            self.log.info(f"MQTT connected to {self.host}:{self.port}")
            self.log.info(f"Subscribing to: {', '.join(Topics.ALL_SUBSCRIBED)}")
            for t in Topics.ALL_SUBSCRIBED:
                client.subscribe(t)
            # Also subscribe to the self-test topic
            client.subscribe("quadpod/selftest")
            self.log.info("Subscriptions sent. Waiting for ESP32 messages...")
            # ─── Self-test: publish a message and check if we receive it ──
            # This tests SAME-client routing (laptop → laptop).
            self._run_self_test(client)
        else:
            self.log.error(f"MQTT connect failed rc={rc}")

    def _run_self_test(self, client):
        """Publish a test message to ourselves. Tests same-client routing."""
        import threading as _t
        def _test():
            _t.Event().wait(0.5)  # give subscribe time to take effect
            self.log.info("Running MQTT self-test (same-client routing)...")
            client.publish("quadpod/selftest", "ping", qos=0)
            _t.Event().wait(2.0)
            if not self.self_test_passed:
                self.log.error(
                    "✗ MQTT self-test FAILED — broker is NOT routing messages\n"
                    "  even from a client to itself. The broker is completely broken.\n"
                    "  → FIX: Install Mosquitto and run with --external-mqtt.\n"
                    "     Windows:  Download from https://mosquitto.org/download/\n"
                    "     Linux:    sudo apt install mosquitto\n"
                    "     Then:     python s7s_control_station.py --external-mqtt 127.0.0.1:1883"
                )
            else:
                self.log.info("✓ Self-test passed (same-client routing works)")
                self.log.info("  If ESP32 still not detected, the issue is CROSS-client")
                self.log.info("  routing (ESP32 → laptop). Click 'Test ESP32 Link' to check.")
        _t.Thread(target=_test, daemon=True).start()

    def run_cross_test(self):
        """Publish a ping to the ESP32 and wait for its pong response.
        This tests CROSS-client routing (ESP32 → laptop), which is the actual
        failure mode when the amqtt broker doesn't route between different clients."""
        if not self._client or not self._client.is_connected():
            self.log.error("Cannot run cross-test: MQTT client not connected")
            return
        self.cross_test_pending = True
        self.cross_test_sent_time = time.time()
        self.log.info("Cross-test: sending ping to ESP32 (quadpod/cmd/ping)...")
        self.publish(Topics.CMD_PING, "")

        import threading as _t
        def _wait_for_pong():
            _t.Event().wait(5.0)  # wait up to 5 seconds for pong
            if self.cross_test_pending:
                # Still waiting — pong never arrived
                self.cross_test_pending = False
                self.log.error(
                    "✗ Cross-test FAILED — ESP32 did not respond to ping.\n"
                    "  This means the broker is NOT routing messages from the ESP32\n"
                    "  to the laptop. Same-client routing works (self-test passed),\n"
                    "  but cross-client routing is broken.\n"
                    "  → This is a known bug in the embedded amqtt broker.\n"
                    "  → FIX: Install Mosquitto and run with --external-mqtt:\n"
                    "     1. Download from https://mosquitto.org/download/\n"
                    "     2. Edit C:\\Program Files\\mosquitto\\mosquitto.conf, add:\n"
                    "        listener 1883 0.0.0.0\n"
                    "        allow_anonymous true\n"
                    "     3. Restart: net stop mosquitto && net start mosquitto\n"
                    "     4. Run: python s7s_control_station.py --external-mqtt 127.0.0.1:1883"
                )
            else:
                elapsed = time.time() - self.cross_test_sent_time
                self.log.info(f"✓ Cross-test PASSED — ESP32 responded in {elapsed:.2f}s")
                self.log.info("  Cross-client routing works. ESP32 link is healthy.")
        _t.Thread(target=_wait_for_pong, daemon=True).start()

    def _on_disconnect(self, client, userdata, *args):
        # v1: args = (rc,)             → rc at args[0]
        # v2: args = (flags, reason, props) → reason at args[1]
        if len(args) >= 3:
            rc = args[1]      # v2: reason_code
        elif len(args) >= 1:
            rc = args[0]      # v1: rc
        else:
            rc = 0
        if rc != 0 and str(rc) != "0" and str(rc) != "Success":
            self.log.warning(f"MQTT unexpected disconnect rc={rc}, will retry")

    def _on_message(self, client, userdata, msg):
        topic = msg.topic
        try:
            payload = msg.payload.decode("utf-8", errors="replace")
        except Exception:
            payload = str(msg.payload)
        try:
            data = json.loads(payload) if payload.startswith("{") else payload
        except json.JSONDecodeError:
            data = payload

        # ─── Diagnostic counting ────────────────────────────────────────
        self.rx_count += 1
        is_from_esp32 = (topic.startswith("quadpod/state/") or topic == Topics.LOG)
        if is_from_esp32:
            self.rx_count_from_esp32 += 1
            if not self.first_rx_logged:
                self.first_rx_logged = True
                self.log.info(f"✓✓✓ FIRST MESSAGE FROM ESP32 RECEIVED ✓✓✓")
                self.log.info(f"  topic: {topic}")
                self.log.info(f"  payload: {payload[:200]}")
                self.log.info(f"  The ESP32 link is working!")

        # ─── Verbose MQTT logging (first message per topic + all if --verbose-mqtt) ──
        if topic not in self._topic_first_seen:
            self._topic_first_seen[topic] = True
            short = payload if len(payload) <= 150 else payload[:150] + "..."
            self.log.info(f"MQTT first on {topic}: {short}")
        elif self.verbose_mqtt and self._msg_log_count < 500:
            self._msg_log_count += 1
            short = payload if len(payload) <= 100 else payload[:100] + "..."
            self.log.debug(f"MQTT ← {topic}: {short}")

        if topic == Topics.STATE_TELEMETRY and isinstance(data, dict):
            self.state.update_telemetry(data)
        elif topic == Topics.STATE_GYRO and isinstance(data, dict):
            self.state.update_gyro(data)
        elif topic == Topics.STATE_STATUS and isinstance(data, dict):
            self.state.update_status(data)
        elif topic == Topics.LOG and isinstance(data, dict):
            level = data.get("level", "info").upper()
            msg_txt = data.get("msg", "")
            # Check if this is a pong response (cross-test)
            if data.get("t") == "pong" and self.cross_test_pending:
                self.cross_test_pending = False
                self.cross_test_passed = True
                # Don't log as [ESP32] — the cross-test thread will log success
            else:
                # Map level → logging method name (handle 'WARN' → 'warning')
                method_name = {"WARN": "warning", "ERROR": "error",
                               "DEBUG": "debug", "INFO": "info"}.get(level, "info")
                getattr(self.log, method_name)(f"[ESP32] {msg_txt}")
        elif topic == "quadpod/selftest":
            # Self-test response (we published it ourselves to verify routing)
            self.self_test_passed = True
            self.log.info("✓ Self-test: same-client routing works (broker received our own message)")

        if self.on_msg:
            try:
                self.on_msg(topic, payload)
            except Exception as e:
                self.log.debug(f"on_msg callback error: {e}")

    # ── publishing helpers ───────────────────────────────────────────────
    def publish(self, topic: str, payload: Any, qos: int = 0):
        if not self._client:
            return
        if isinstance(payload, (dict, list)):
            payload = json.dumps(payload)
        elif isinstance(payload, bool):
            payload = "1" if payload else "0"
        elif isinstance(payload, (int, float)):
            payload = str(payload)
        self._client.publish(topic, payload, qos=qos)

    # ── semantic command API ─────────────────────────────────────────────
    def cmd_gait(self, name: str):
        self.publish(Topics.CMD_GAIT, name)

    def cmd_speed(self, value: float):
        self.publish(Topics.CMD_SPEED, f"{value:.3f}")

    def cmd_estop(self):
        self.publish(Topics.CMD_ESTOP, "1")

    def cmd_calib_motor_start(self):
        self.publish(Topics.CMD_CALIB_MOTOR_START, "")

    def cmd_calib_motor_set(self, joint: int, offset: float):
        self.publish(Topics.CMD_CALIB_MOTOR_SET, {"joint": int(joint), "offset": float(offset)})

    def cmd_calib_motor_save(self):
        self.publish(Topics.CMD_CALIB_MOTOR_SAVE, "")

    def cmd_calib_motor_load(self):
        self.publish(Topics.CMD_CALIB_MOTOR_LOAD, "")

    def cmd_calib_motor_reset(self):
        self.publish(Topics.CMD_CALIB_MOTOR_RESET, "")

    def cmd_calib_motor_end(self):
        self.publish(Topics.CMD_CALIB_MOTOR_END, "")

    def cmd_calib_gyro_set(self, yaw: float, pitch: float, roll: float):
        self.publish(Topics.CMD_CALIB_GYRO_SET,
                     {"yaw": float(yaw), "pitch": float(pitch), "roll": float(roll)})

    def cmd_calib_gyro_clear(self):
        self.publish(Topics.CMD_CALIB_GYRO_CLEAR, "")

    def cmd_calib_gyro_save(self):
        self.publish(Topics.CMD_CALIB_GYRO_SAVE, "")

    def cmd_ping(self):
        self.publish(Topics.CMD_PING, "")


# =============================================================================
#  Xbox controller removed — Xbox now connects directly to ESP32 via Bluepad32
# =============================================================================
# Previously this module polled an Xbox controller through pygame and translated
# button presses to gait commands. With the Bluepad32 architecture change the
# controller pairs directly to the ESP32 over Bluetooth; the laptop no longer
# touches it. Priority arbitration (laptop MQTT commands override the Xbox for
# ~2 seconds) is implemented in the ESP32 firmware, which reports the active
# priority state back through the quadpod/state/status topic as the fields
# `mqttOverrideActive` and `controllerConnected`. See SharedState.update_status
# and ControlsTab.refresh.


# =============================================================================
#  UI helpers
# =============================================================================

def safe_float(s: str, default: float = 0.0) -> float:
    try:
        return float(s)
    except (ValueError, TypeError):
        return default


# =============================================================================
#  Tab 1 — Calibration (motor + gyro)
# =============================================================================

class CalibrationTab(ttk.Frame):
    """Motor offset sliders + gyro zero-point capture."""

    STEP_FINE = 0.005   # rad per arrow press on slider
    STEP_COARSE = 0.02  # rad per +/- button

    def __init__(self, parent, hub: MqttHub, state: SharedState, log: logging.Logger):
        super().__init__(parent)
        self.hub = hub
        self.state = state
        self.log = log
        self.sliders: list[ttk.Scale] = []
        self.value_labels: list[ttk.Label] = []
        self._build_ui()

    def _build_ui(self):
        # ─── Top: mode controls ───────────────────────────────────────────
        top = ttk.LabelFrame(self, text="Calibration Mode", padding=10)
        top.pack(fill="x", padx=8, pady=6)

        self.btn_start = ttk.Button(top, text="Enter Calibration Mode", command=self._on_start)
        self.btn_start.pack(side="left", padx=4)
        self.btn_end = ttk.Button(top, text="Exit Calibration Mode", command=self._on_end, state="disabled")
        self.btn_end.pack(side="left", padx=4)

        ttk.Separator(top, orient="vertical").pack(side="left", fill="y", padx=10)

        ttk.Label(top, text="Active joint:").pack(side="left", padx=(4,2))
        # StringVar (not IntVar) — Combobox options are like "3: RR_abd"
        self.active_joint_var = tk.StringVar(value=f"0: {JOINT_LABELS[0]}")
        self.active_joint_menu = ttk.Combobox(
            top, textvariable=self.active_joint_var, state="readonly",
            values=[f"{i}: {lbl}" for i, lbl in enumerate(JOINT_LABELS)],
            width=18,
        )
        self.active_joint_menu.current(0)
        self.active_joint_menu.bind("<<ComboboxSelected>>", self._on_joint_selected)
        self.active_joint_menu.pack(side="left", padx=4)

        ttk.Separator(top, orient="vertical").pack(side="left", fill="y", padx=10)

        self.btn_save = ttk.Button(top, text="Save to ESP32 (NVS)", command=self._on_save, state="disabled")
        self.btn_save.pack(side="left", padx=4)
        self.btn_load = ttk.Button(top, text="Load from ESP32", command=self._on_load, state="disabled")
        self.btn_load.pack(side="left", padx=4)
        self.btn_reset = ttk.Button(top, text="Reset to Factory", command=self._on_reset)
        self.btn_reset.pack(side="left", padx=4)
        self.btn_print = ttk.Button(top, text="Print Calib to Terminal", command=self._on_print_calib)
        self.btn_print.pack(side="left", padx=4)

        self.mode_status_var = tk.StringVar(value="Calibration: OFF")
        ttk.Label(top, textvariable=self.mode_status_var, font=("TkDefaultFont", 9, "bold")).pack(side="right", padx=10)

        # ─── Middle: motor offset sliders ────────────────────────────────
        motors = ttk.LabelFrame(self, text="Motor Idle Offsets (rad)", padding=10)
        motors.pack(fill="both", expand=True, padx=8, pady=6)

        for group_name, labels in JOINT_GROUPS:
            grp = ttk.LabelFrame(motors, text=group_name, padding=8)
            grp.pack(fill="x", pady=4)
            for col, label in enumerate(labels):
                idx = JOINT_LABELS.index(label)
                self._build_motor_row(grp, idx, label, col)

        # ─── Bottom: gyro calibration ────────────────────────────────────
        gyro = ttk.LabelFrame(self, text="Gyro Zero-Point Calibration", padding=10)
        gyro.pack(fill="x", padx=8, pady=6)

        # Live gyro display
        live = ttk.Frame(gyro)
        live.pack(fill="x", pady=(0, 8))
        ttk.Label(live, text="Live raw gyro:", font=("TkDefaultFont", 9, "bold")).grid(row=0, column=0, sticky="w")
        self.gyro_yaw_var   = tk.StringVar(value="yaw:   +0.00°")
        self.gyro_pitch_var = tk.StringVar(value="pitch: +0.00°")
        self.gyro_roll_var  = tk.StringVar(value="roll:  +0.00°")
        ttk.Label(live, textvariable=self.gyro_yaw_var,   width=14).grid(row=0, column=1, padx=4)
        ttk.Label(live, textvariable=self.gyro_pitch_var, width=14).grid(row=0, column=2, padx=4)
        ttk.Label(live, textvariable=self.gyro_roll_var,  width=14).grid(row=0, column=3, padx=4)
        ttk.Label(live, text="Calibrated:").grid(row=1, column=0, sticky="w")
        self.gyro_cal_var = tk.StringVar(value="yaw: 0.00°, pitch: 0.00°, roll: 0.00°")
        ttk.Label(live, textvariable=self.gyro_cal_var).grid(row=1, column=1, columnspan=3, sticky="w", padx=4)

        # Capture + clear
        actions = ttk.Frame(gyro)
        actions.pack(fill="x")
        ttk.Label(actions, text="1. Place the robot in its ideal standing position.\n"
                                "2. Click 'Capture Current as Zero' to send current raw gyro as the offset.\n"
                                "3. The 'Calibrated' values above should now read ~0.").pack(side="left", anchor="w")

        btns = ttk.Frame(actions)
        btns.pack(side="right")
        self.btn_gyro_capture = ttk.Button(btns, text="Capture Current as Zero", command=self._on_gyro_capture)
        self.btn_gyro_capture.pack(side="left", padx=4)
        self.btn_gyro_clear = ttk.Button(btns, text="Clear Offset (0,0,0)", command=self._on_gyro_clear)
        self.btn_gyro_clear.pack(side="left", padx=4)
        self.btn_gyro_save = ttk.Button(btns, text="Save to ESP32 (NVS)", command=self._on_gyro_save)
        self.btn_gyro_save.pack(side="left", padx=4)

    def _build_motor_row(self, parent, idx: int, label: str, col: int):
        row = ttk.Frame(parent)
        row.pack(fill="x", pady=2)
        ttk.Label(row, text=f"{idx:2d}. {label}", width=14, anchor="w").pack(side="left")

        val = tk.StringVar(value="+0.000")
        lbl = ttk.Label(row, textvariable=val, width=8, anchor="e")
        lbl.pack(side="left", padx=4)

        sld = ttk.Scale(row, from_=-JOINT_LIMIT, to=JOINT_LIMIT, orient="horizontal",
                        command=lambda v, i=idx: self._on_slider(i, v))

        # ─── CRITICAL: append to tracking lists BEFORE sld.set() ──────────
        # sld.set(0.0) triggers the command callback, which calls
        # self.value_labels[idx].set(...). If the lists don't have the entry
        # yet, we get an IndexError. So append first, then set.
        self.sliders.append(sld)
        self.value_labels.append(val)

        sld.set(0.0)   # triggers _on_slider — lists are now populated
        sld.pack(side="left", fill="x", expand=True, padx=4)

        minus = ttk.Button(row, text="−", width=3,
                           command=lambda i=idx: self._bump(i, -self.STEP_COARSE))
        minus.pack(side="left")
        plus = ttk.Button(row, text="+", width=3,
                          command=lambda i=idx: self._bump(i, +self.STEP_COARSE))
        plus.pack(side="left")

        zero = ttk.Button(row, text="0", width=3,
                          command=lambda i=idx: self._set_slider(i, 0.0))
        zero.pack(side="left", padx=(2,0))

    # ── motor offset actions ─────────────────────────────────────────────
    def _on_slider(self, idx: int, value: str):
        v = float(value)
        # Guard against index out of range (defensive — shouldn't happen now
        # that lists are populated before sld.set(), but keeps it bulletproof)
        if 0 <= idx < len(self.value_labels):
            self.value_labels[idx].set(f"{v:+.3f}")
        if self.hub:
            self.hub.cmd_calib_motor_set(idx, v)
        self.state.arm_manual_override()

    def _bump(self, idx: int, delta: float):
        cur = self.sliders[idx].get()
        new = max(-JOINT_LIMIT, min(JOINT_LIMIT, cur + delta))
        self._set_slider(idx, new)

    def _set_slider(self, idx: int, value: float):
        self.sliders[idx].set(value)
        self._on_slider(idx, str(value))

    def _on_joint_selected(self, _evt):
        selection = self.active_joint_var.get()  # e.g. "3: RR_abd"
        try:
            idx = int(selection.split(":")[0])
        except (ValueError, IndexError):
            return
        # Inform ESP32 which joint is "active" for visual feedback
        # (ESP32 firmware can pulse the joint slightly to indicate selection)
        if 0 <= idx < 12:
            self.hub.cmd_calib_motor_set(idx, self.sliders[idx].get())
            self.state.arm_manual_override()

    # ── mode actions ─────────────────────────────────────────────────────
    def _on_start(self):
        if not messagebox.askyesno("Enter Calibration Mode",
                                    "This will hold the robot at the stand pose and disable gait playback.\n\n"
                                    "Continue?"):
            return
        self.hub.cmd_calib_motor_start()
        self.state.arm_manual_override(5.0)
        self.btn_start.config(state="disabled")
        self.btn_end.config(state="normal")
        self.btn_save.config(state="normal")
        self.btn_load.config(state="normal")
        self.mode_status_var.set("Calibration: ON")

    def _on_end(self):
        self.hub.cmd_calib_motor_end()
        self.state.arm_manual_override(2.0)
        self.btn_start.config(state="normal")
        self.btn_end.config(state="disabled")
        self.btn_save.config(state="disabled")
        self.btn_load.config(state="disabled")
        self.mode_status_var.set("Calibration: OFF")

    def _on_save(self):
        self.hub.cmd_calib_motor_save()
        self.log.info("Motor calibration saved to ESP32 NVS")

    def _on_load(self):
        self.hub.cmd_calib_motor_load()
        self.log.info("Requested motor calibration load from ESP32 NVS")

    def _on_reset(self):
        """Reset motor calibration to factory defaults (compiled-in offsets
        derived from the test script: abd=0, flex=-30°, knee=-30°)."""
        if not messagebox.askyesno("Reset to Factory Defaults",
                                    "This will clear the NVS calibration and "
                                    "restore the compiled-in factory defaults.\n\n"
                                    "The robot will immediately go to the test-script "
                                    "stand pose (abd=90°, flex=60°, knee=60°).\n\n"
                                    "Continue?"):
            return
        self.hub.cmd_calib_motor_reset()
        self.log.info("Motor calibration reset to factory defaults")

    def _on_print_calib(self):
        """Print the current motor offsets in C++ array format so the user
        can copy-paste them into the firmware's CALIB_OFFSET array and
        permanently burn them into the code (not just NVS)."""
        offsets = self.state.telemetry.offsets[:12]
        print("\n" + "=" * 72)
        print("  CURRENT MOTOR CALIBRATION OFFSETS — copy-paste into firmware")
        print("=" * 72)
        print()
        print("// Paste this into s7s_esp32_main_v2.cpp, replacing the")
        print("// CALIB_OFFSET array (around line 140):")
        print()
        print("static float CALIB_OFFSET[12] = {")
        # Row 1: abductors (indices 0-3)
        row1 = ", ".join(f"{offsets[i]:.6f}f" for i in range(4))
        print(f"  {row1},   // FR_abd, FL_abd, RL_abd, RR_abd")
        # Row 2: flex (indices 4-7)
        row2 = ", ".join(f"{offsets[i]:.6f}f" for i in range(4, 8))
        print(f"  {row2},   // FR_flex, FL_flex, RL_flex, RR_flex")
        # Row 3: knee (indices 8-11)
        row3 = ", ".join(f"{offsets[i]:.6f}f" for i in range(8, 12))
        print(f"  {row3}    // FR_knee, FL_knee, RL_knee, RR_knee")
        print("};")
        print()
        print("// Also print as Python list for reference:")
        print(f"calib_offsets = {offsets}")
        print()
        print("=" * 72)
        # Also log to the in-app log tab
        self.log.info("Calibration offsets printed to terminal — see the "
                       "terminal where you ran 'python s7s_control_station.py'")
        self.log.info(f"Offsets: {offsets}")

    # ── gyro actions ─────────────────────────────────────────────────────
    def _on_gyro_capture(self):
        g = self.state.gyro
        if not g.valid:
            messagebox.showwarning("Gyro not available",
                                    "No gyro data received yet. Make sure the phone bridge is running "
                                    "and publishing to quadpod/state/gyro.")
            return
        self.hub.cmd_calib_gyro_set(g.yaw, g.pitch, g.roll)
        self.state.gyro_offset_local = {"yaw": g.yaw, "pitch": g.pitch, "roll": g.roll}
        self.log.info(f"Gyro offset captured: yaw={g.yaw:.2f} pitch={g.pitch:.2f} roll={g.roll:.2f}")
        self.state.arm_manual_override()

    def _on_gyro_clear(self):
        self.hub.cmd_calib_gyro_clear()
        self.state.gyro_offset_local = {"yaw": 0.0, "pitch": 0.0, "roll": 0.0}
        self.log.info("Gyro offset cleared")

    def _on_gyro_save(self):
        self.hub.cmd_calib_gyro_save()
        self.log.info("Gyro offset saved to ESP32 NVS")

    # ── called periodically by main app to refresh live values ───────────
    def refresh(self):
        snap = self.state.snapshot()
        g = snap["gyro"]
        # Format raw gyro
        self.gyro_yaw_var.set(f"yaw:   {g['yaw']:+7.2f}°")
        self.gyro_pitch_var.set(f"pitch: {g['pitch']:+7.2f}°")
        self.gyro_roll_var.set(f"roll:  {g['roll']:+7.2f}°")
        # Calibrated gyro (from ESP32 telemetry — applies the offset)
        cal = snap["telemetry"]["gyro_calibrated"]
        self.gyro_cal_var.set(
            f"yaw: {cal.get('yaw', 0):+6.2f}°, pitch: {cal.get('pitch', 0):+6.2f}°, roll: {cal.get('roll', 0):+6.2f}°"
        )
        # Mode status
        if snap["telemetry"]["calib_mode"]:
            self.mode_status_var.set(f"Calibration: ON  (joint #{snap['telemetry']['calib_joint']})")
        # Sync sliders with offsets reported by ESP32 telemetry (so changes from
        # load/save reflect in the UI)
        offsets = snap["telemetry"]["offsets"]
        for i, val in enumerate(offsets[:12]):
            try:
                cur = self.sliders[i].get()
                if abs(cur - val) > 0.001 and not self._slider_being_dragged(i):
                    self.sliders[i].set(val)
                    self.value_labels[i].set(f"{val:+.3f}")
            except (IndexError, AttributeError):
                pass

    def _slider_being_dragged(self, idx: int) -> bool:
        # Tk doesn't expose "is being dragged" cleanly; we approximate by
        # checking if the slider has focus (mouse held).
        try:
            return str(self.sliders[idx]) == str(self.focus_get())
        except Exception:
            return False


# =============================================================================
#  Tab 2 — Controls (gait buttons + speed + e-stop)
# =============================================================================

class ControlsTab(ttk.Frame):
    """Manual gait buttons + speed slider + e-stop.

    Priority arbitration is handled on the ESP32 (laptop MQTT commands
    override the Xbox for ~2 seconds). This tab just reports the current
    priority state from ESP32 telemetry.
    """

    def __init__(self, parent, hub: MqttHub, state: SharedState, log: logging.Logger):
        super().__init__(parent)
        self.hub = hub
        self.state = state
        self.log = log
        self._build_ui()

    def _build_ui(self):
        # E-stop banner (always at top, full width, red)
        estop = ttk.Frame(self, padding=8)
        estop.pack(fill="x", padx=8, pady=6)
        self.btn_estop = tk.Button(estop, text="⛔  EMERGENCY STOP  ⛔",
                                    bg="#ff4444", fg="white", font=("TkDefaultFont", 14, "bold"),
                                    height=2, command=self._on_estop)
        self.btn_estop.pack(fill="x")

        # Priority status — reflects the ESP32's view (Xbox ↔ ESP32 link +
        # which side currently has authority). Updated from telemetry in refresh().
        ov = ttk.Frame(self, padding=4)
        ov.pack(fill="x", padx=8)
        self.override_var = tk.StringVar(value="No controller connected to ESP32")
        ttk.Label(ov, text="Priority: ", font=("TkDefaultFont", 9, "bold")).pack(side="left")
        ttk.Label(ov, textvariable=self.override_var).pack(side="left")
        ttk.Label(ov, text="(arbitrated on the ESP32)").pack(side="left", padx=8)

        # Gait buttons (grid)
        gaits = ttk.LabelFrame(self, text="Gait Commands (hold-to-run semantics on ESP32)", padding=10)
        gaits.pack(fill="both", expand=True, padx=8, pady=6)

        for i, (gait_id, label) in enumerate(GAITS):
            row, col = divmod(i, 4)
            btn = ttk.Button(gaits, text=label, width=18,
                             command=lambda g=gait_id: self._on_gait(g))
            btn.grid(row=row, column=col, padx=6, pady=6, sticky="nsew")
        for c in range(4):
            gaits.columnconfigure(c, weight=1)

        # Speed slider
        spd = ttk.LabelFrame(self, text="Gait Speed Multiplier", padding=10)
        spd.pack(fill="x", padx=8, pady=6)
        self.speed_var = tk.DoubleVar(value=1.0)
        self.speed_label_var = tk.StringVar(value="1.00×")
        sld = ttk.Scale(spd, from_=0.1, to=3.0, orient="horizontal",
                        variable=self.speed_var,
                        command=self._on_speed)
        sld.pack(side="left", fill="x", expand=True, padx=4)
        ttk.Label(spd, textvariable=self.speed_label_var, width=8).pack(side="left", padx=8)
        ttk.Button(spd, text="Reset 1.0×", command=lambda: self._set_speed(1.0)).pack(side="left")

        # Quick info
        info = ttk.LabelFrame(self, text="Status", padding=8)
        info.pack(fill="x", padx=8, pady=6)
        self.active_gait_var = tk.StringVar(value="Active gait: stand")
        ttk.Label(info, textvariable=self.active_gait_var).pack(anchor="w")
        self.gait_time_var = tk.StringVar(value="Gait time: 0.00 s")
        ttk.Label(info, textvariable=self.gait_time_var).pack(anchor="w")

    # ── actions ──────────────────────────────────────────────────────────
    def _on_gait(self, gait: str):
        self.hub.cmd_gait(gait)
        self.state.arm_manual_override()
        self.log.info(f"[UI] gait → {gait}")

    def _on_speed(self, _v=None):
        v = self.speed_var.get()
        self.speed_label_var.set(f"{v:.2f}×")
        self.hub.cmd_speed(v)
        self.state.arm_manual_override()

    def _set_speed(self, v: float):
        self.speed_var.set(v)
        self._on_speed()

    def _on_estop(self):
        self.hub.cmd_estop()
        self.hub.cmd_gait("stand")
        self.state.arm_manual_override(5.0)
        self.log.warning("[UI] E-STOP triggered")

    def refresh(self):
        snap = self.state.snapshot()
        t = snap["telemetry"]
        self.active_gait_var.set(f"Active gait: {t['gait']}")
        self.gait_time_var.set(f"Gait time: {t['gait_time']:.2f} s")
        # Priority state is reported by the ESP32 in the status topic.
        s = snap["status"]
        if s.get("mqttOverrideActive"):
            self.override_var.set("Laptop has priority (ESP32 override active)")
        elif s.get("controllerConnected"):
            self.override_var.set("Xbox has control (connected to ESP32)")
        else:
            self.override_var.set("No controller connected to ESP32")


# =============================================================================
#  Tab 3 — Telemetry (live plot + numeric readout)
# =============================================================================

class TelemetryTab(ttk.Frame):
    """Live 12-joint pose + gyro readout. matplotlib is optional."""

    def __init__(self, parent, hub: MqttHub, state: SharedState, log: logging.Logger):
        super().__init__(parent)
        self.hub = hub
        self.state = state
        self.log = log
        self._build_ui()

    def _build_ui(self):
        # Top: connection / status
        top = ttk.LabelFrame(self, text="Connection", padding=8)
        top.pack(fill="x", padx=8, pady=6)
        self.status_var = tk.StringVar(value="ESP32: not connected")
        ttk.Label(top, textvariable=self.status_var, font=("TkDefaultFont", 10, "bold")).pack(anchor="w")
        self.uptime_var = tk.StringVar(value="Uptime: —")
        ttk.Label(top, textvariable=self.uptime_var).pack(anchor="w")

        # Joint pose readout — text grid (always available)
        joints = ttk.LabelFrame(self, text="Joint Angles (rad) — live from ESP32", padding=8)
        joints.pack(fill="x", padx=8, pady=6)
        self.joint_vars = []
        for i, label in enumerate(JOINT_LABELS):
            row, col = divmod(i, 4)
            f = ttk.Frame(joints); f.grid(row=row, column=col, padx=6, pady=2, sticky="ew")
            ttk.Label(f, text=label, width=10, anchor="w").pack(side="left")
            v = tk.StringVar(value="+0.000")
            ttk.Label(f, textvariable=v, width=8, anchor="e",
                      font=("TkDefaultFont", 9, "bold")).pack(side="left")
            self.joint_vars.append(v)
        for c in range(4):
            joints.columnconfigure(c, weight=1)

        # Offsets readout
        offs = ttk.LabelFrame(self, text="Calibration Offsets (rad)", padding=8)
        offs.pack(fill="x", padx=8, pady=6)
        self.offset_vars = []
        for i, label in enumerate(JOINT_LABELS):
            row, col = divmod(i, 4)
            f = ttk.Frame(offs); f.grid(row=row, column=col, padx=6, pady=2, sticky="ew")
            ttk.Label(f, text=label, width=10, anchor="w").pack(side="left")
            v = tk.StringVar(value="+0.000")
            ttk.Label(f, textvariable=v, width=8, anchor="e").pack(side="left")
            self.offset_vars.append(v)
        for c in range(4):
            offs.columnconfigure(c, weight=1)

        # Plot area (matplotlib) — only if available
        if MPL_OK:
            plot = ttk.LabelFrame(self, text="Joint Pose Plot (live)", padding=4)
            plot.pack(fill="both", expand=True, padx=8, pady=6)
            self.fig, self.ax = plt.subplots(figsize=(8, 3), constrained_layout=True)
            self.ax.set_title("Joint Angles (rad)")
            self.ax.set_ylim(-1.3, 1.3)
            self.ax.set_xlim(-0.5, 11.5)
            self.ax.set_xticks(range(12))
            self.ax.set_xticklabels(JOINT_LABELS, rotation=45, ha="right", fontsize=8)
            self.ax.grid(True, alpha=0.3)
            self.bars = self.ax.bar(range(12), [0]*12, color="#4363d8")
            self.canvas = FigureCanvasTkAgg(self.fig, master=plot)
            self.canvas.get_tk_widget().pack(fill="both", expand=True)
        else:
            ttk.Label(self, text="(matplotlib not installed — install with: pip install matplotlib)",
                      foreground="#888").pack(pady=8)

    def refresh(self):
        snap = self.state.snapshot()
        t = snap["telemetry"]
        s = snap["status"]
        # Connection
        if s["esp32_connected"]:
            self.status_var.set(f"ESP32: connected  (calib={'ON' if t['calib_mode'] else 'OFF'})")
        else:
            self.status_var.set("ESP32: not connected")
        self.uptime_var.set(f"Uptime: {s['uptime']:.1f} s")

        pose = t["pose"]
        for i, v in enumerate(pose[:12]):
            self.joint_vars[i].set(f"{v:+.3f}")
        offs = t["offsets"]
        for i, v in enumerate(offs[:12]):
            self.offset_vars[i].set(f"{v:+.3f}")

        if MPL_OK:
            for bar, val in zip(self.bars, pose[:12]):
                bar.set_height(val)
            self.canvas.draw_idle()


# =============================================================================
#  Tab 4 — Log viewer
# =============================================================================

class LogTab(ttk.Frame):
    """Scrolling log — drains the queue fed by HubLogHandler."""

    def __init__(self, parent, log_q: queue.Queue, log: logging.Logger):
        super().__init__(parent)
        self.log = log
        self.log_q = log_q

        top = ttk.Frame(self, padding=4)
        top.pack(fill="x", padx=8, pady=6)
        self.autoscroll_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(top, text="Auto-scroll", variable=self.autoscroll_var).pack(side="left")
        ttk.Button(top, text="Clear", command=self._clear).pack(side="left", padx=8)
        ttk.Button(top, text="Save to file…", command=self._save).pack(side="left")
        self.count_var = tk.StringVar(value="0 lines")
        ttk.Label(top, textvariable=self.count_var).pack(side="right")

        self.text = tk.Text(self, wrap="none", state="disabled",
                            font=("Courier", 9), background="#1e1e1e", foreground="#d4d4d4",
                            insertbackground="#d4d4d4", selectbackground="#264f78")
        self.text.pack(fill="both", expand=True, padx=8, pady=(0,8))

        # Tag colors per level
        self.text.tag_config("DEBUG", foreground="#888")
        self.text.tag_config("INFO",  foreground="#d4d4d4")
        self.text.tag_config("WARN",  foreground="#ffcc00")
        self.text.tag_config("ERROR", foreground="#ff5555")
        self.text.tag_config("ESP32", foreground="#42d4f4")

        self._line_count = 0
        self._max_lines = 2000

    def _append(self, level: str, msg: str):
        self.text.config(state="normal")
        tag = level if level in ("DEBUG","INFO","WARN","ERROR") else "INFO"
        if "[ESP32]" in msg:
            tag = "ESP32"
        self.text.insert("end", msg + "\n", tag)
        # Trim if too long
        if self._line_count > self._max_lines:
            self.text.delete("1.0", "500.0")
            self._line_count -= 500
        self.text.config(state="disabled")
        self._line_count += 1
        self.count_var.set(f"{self._line_count} lines")
        if self.autoscroll_var.get():
            self.text.see("end")

    def _clear(self):
        self.text.config(state="normal")
        self.text.delete("1.0", "end")
        self.text.config(state="disabled")
        self._line_count = 0
        self.count_var.set("0 lines")

    def _save(self):
        from tkinter import filedialog
        path = filedialog.asksaveasfilename(
            defaultextension=".log",
            filetypes=[("Log files", "*.log"), ("Text files", "*.txt"), ("All", "*.*")],
        )
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8") as f:
                f.write(self.text.get("1.0", "end"))
            self.log.info(f"Log saved to {path}")
        except Exception as e:
            messagebox.showerror("Save failed", str(e))

    def refresh(self):
        # Drain the queue without blocking
        drained = 0
        while drained < 50:
            try:
                rec = self.log_q.get_nowait()
            except queue.Empty:
                break
            self._append(rec.levelname, self.log.handlers[1].format(rec) if len(self.log.handlers) > 1 else rec.getMessage())
            drained += 1


# =============================================================================
#  Main application — wires everything together
# =============================================================================

class ControlStation(tk.Tk):
    """Top-level window. Owns broker + mqtt client + tabs."""

    def __init__(self, args: argparse.Namespace, log_q: queue.Queue):
        super().__init__()
        self.args = args
        self.log_q = log_q
        self.log = setup_logging(log_q)

        self.title("S7S Control Station")
        self.geometry("1100x720")
        self.minsize(900, 600)

        self.state = SharedState(self.log)
        self.broker: Optional[EmbeddedBroker] = None
        self.hub: Optional[MqttHub] = None

        self._build_ui()
        self._start_infrastructure()
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self._refresh_scheduled = False
        self._closing = False  # set to True by _on_close to stop the refresh loop
        self._after_id = None  # stores the after() ID for cancellation on close
        self._schedule_refresh()

    def _build_ui(self):
        # Top status bar
        bar = ttk.Frame(self, padding=6)
        bar.pack(fill="x", side="top")
        self.broker_var = tk.StringVar(value="Broker: starting…")
        self.mqtt_var   = tk.StringVar(value="MQTT: …")
        self.esp_var    = tk.StringVar(value="ESP32: …")
        self.rx_var     = tk.StringVar(value="MQTT rx: 0 msgs")
        for v in (self.broker_var, self.mqtt_var, self.esp_var, self.rx_var):
            ttk.Label(bar, textvariable=v, padding=(10,0)).pack(side="left")
        ttk.Button(bar, text="Test ESP32 Link", command=self._on_cross_test).pack(side="right", padx=4)
        ttk.Button(bar, text="Ping ESP32", command=self._on_ping).pack(side="right", padx=4)

        # Notebook with tabs
        self.nb = ttk.Notebook(self)
        self.nb.pack(fill="both", expand=True)

        # Note: hub may be None at this point (broker hasn't started yet).
        # The tab constructors only store the reference; they don't call hub.
        self.tab_calib = CalibrationTab(self.nb, self._stub_hub(), self.state, self.log)
        self.tab_ctrl   = ControlsTab(self.nb,   self._stub_hub(), self.state, self.log)
        self.tab_tel    = TelemetryTab(self.nb,  self._stub_hub(), self.state, self.log)
        self.tab_log    = LogTab(self.nb, self.log_q, self.log)

        self.nb.add(self.tab_calib, text="Calibration")
        self.nb.add(self.tab_ctrl,  text="Controls")
        self.nb.add(self.tab_tel,   text="Telemetry")
        self.nb.add(self.tab_log,   text="Log")

    def _stub_hub(self) -> "MqttHub":
        """We can't construct MqttHub until after broker start. Return a stub
        that queues calls; the real hub will replace it."""
        return _HubStub()

    def _start_infrastructure(self):
        # 1) Start the broker (embedded or external)
        if self.args.external_mqtt:
            host, port = self._parse_addr(self.args.external_mqtt, DEFAULT_BROKER_PORT)
            self.log.info(f"Using external MQTT broker at {host}:{port}")
            self.broker_var.set(f"Broker: external {host}:{port}")
            self.state.broker_running = True
        else:
            if not AMQTT_OK:
                self.log.error("amqtt not installed — falling back to external Mosquitto.\n"
                               "Install with: pip install amqtt")
                self.broker_var.set("Broker: NOT RUNNING (install amqtt)")
            else:
                self.broker = EmbeddedBroker(self.args.broker_port, self.log)
                ok = self.broker.start()
                if ok:
                    self.broker_var.set(f"Broker: embedded 0.0.0.0:{self.args.broker_port}")
                    self.state.broker_running = True
                else:
                    self.broker_var.set("Broker: FAILED to start (port in use?)")
                    self.log.error("Broker failed to start. Try: --external-mqtt 127.0.0.1:1883")

        # 2) Connect MQTT client to the broker
        if self.args.external_mqtt:
            host, port = self._parse_addr(self.args.external_mqtt, DEFAULT_BROKER_PORT)
        else:
            host, port = DEFAULT_BROKER_HOST, self.args.broker_port

        try:
            self.hub = MqttHub(host, port, self.state, self.log,
                               on_msg=self._on_any_mqtt_msg)
            self.hub.verbose_mqtt = self.args.verbose_mqtt
            ok = self.hub.start()
            if ok:
                self.mqtt_var.set(f"MQTT: {host}:{port} ✓")
            else:
                self.mqtt_var.set(f"MQTT: connect failed")
        except Exception as e:
            self.hub = None
            self.mqtt_var.set(f"MQTT: error — {e}")
            self.log.error(f"MQTT hub init failed: {e}")

        # 3) Wire the real hub into the tabs (replace the stub)
        self.tab_calib.hub = self.hub
        self.tab_ctrl.hub  = self.hub
        self.tab_tel.hub   = self.hub

        # Show our LAN IP so the user knows where to point the phone bridge
        lan_ip = self._detect_lan_ip()
        self.log.info(f"Laptop LAN IP: {lan_ip} — point the phone bridge at this address, port {self.args.broker_port}")

    def _parse_addr(self, addr: str, default_port: int) -> tuple[str, int]:
        if ":" in addr:
            host, port_str = addr.rsplit(":", 1)
            return host, int(port_str)
        return addr, default_port

    def _detect_lan_ip(self) -> str:
        """Returns the LAN IP of this laptop (best-effort UDP socket trick)."""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return "127.0.0.1"

    def _on_any_mqtt_msg(self, topic: str, payload: str):
        # Hook for the Log tab — print first 200 chars of every message
        if self.args.verbose_mqtt:
            short = payload if len(payload) <= 200 else payload[:200] + "…"
            self.log.info(f"MQTT ← {topic}: {short}")

    def _on_ping(self):
        if self.hub:
            self.hub.cmd_ping()
            self.log.info("Sent ping to ESP32")

    def _on_cross_test(self):
        """Test cross-client MQTT routing: ping ESP32, wait for pong."""
        if self.hub:
            self.hub.run_cross_test()

    def _schedule_refresh(self):
        """Refresh the visible tab ~20 Hz. Tk after() is single-threaded."""
        if getattr(self, "_closing", False):
            return  # don't schedule more refreshes during shutdown
        self._refresh_all_tabs()
        # Store the after ID so _on_close can cancel the pending callback
        # before destroy() — otherwise the callback fires on a dead widget
        # and raises "invalid command name".
        self._after_id = self.after(50, self._schedule_refresh)  # 20 Hz

    def _refresh_all_tabs(self):
        # Status bar — ESP32 connection based on last-received-message time
        if self.state.is_esp32_connected():
            self.esp_var.set("ESP32: connected")
        elif self.state.esp32_seen:
            self.esp_var.set("ESP32: stale (no data in >5s)")
        else:
            self.esp_var.set("ESP32: waiting for first message…")

        # MQTT rx counter — shows if messages are flowing through the broker
        if self.hub:
            status_parts = [f"MQTT rx: {self.hub.rx_count_from_esp32} msgs"]
            if not self.hub.self_test_passed:
                status_parts.append("self-test FAILED")
            elif self.hub.cross_test_pending:
                status_parts.append("cross-test pending…")
            elif self.hub.cross_test_passed:
                status_parts.append("cross-test OK")
            self.rx_var.set(" | ".join(status_parts))

        # Refresh whichever tab is visible (cheap) — but always refresh log
        # so we don't fall behind on draining the queue.
        try:
            self.tab_log.refresh()
            current = self.nb.select()
            if current:
                tab = self.nametowidget(current)
                if hasattr(tab, "refresh"):
                    tab.refresh()
        except Exception as e:
            # Defensive: never let UI refresh crash the app
            self.log.debug(f"refresh error: {e}")

    def _on_close(self):
        if not messagebox.askokcancel("Quit", "Stop the control station and disconnect?"):
            return
        self.log.info("Shutting down…")
        self._closing = True  # signals _schedule_refresh to stop rescheduling
        # Cancel the pending after() callback BEFORE destroy() — otherwise it
        # fires on a destroyed widget and raises "invalid command name".
        try:
            if getattr(self, "_after_id", None) is not None:
                self.after_cancel(self._after_id)
        except Exception:
            pass
        try:
            if self.hub:  self.hub.stop()
            if self.broker: self.broker.stop()
        except Exception as e:
            self.log.debug(f"shutdown error: {e}")
        self.destroy()


class _HubStub:
    """Stand-in passed to tab constructors before the real hub exists.
    All methods are no-ops; the real hub replaces this instance."""
    def __getattr__(self, name):
        return lambda *a, **kw: None


# =============================================================================
#  Entry point
# =============================================================================

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="S7S Quadrupod Control Station")
    p.add_argument("--broker-port", type=int, default=DEFAULT_BROKER_PORT,
                   help=f"Port for embedded MQTT broker (default {DEFAULT_BROKER_PORT})")
    p.add_argument("--external-mqtt", type=str, default=None,
                   help="Use an external MQTT broker instead of the embedded one. "
                        "Format: host:port or host (default port 1883).")
    p.add_argument("--verbose-mqtt", action="store_true",
                   help="Log every MQTT message to the Log tab (noisy)")
    return p.parse_args()


def main():
    args = parse_args()
    log_q: queue.Queue = queue.Queue(maxsize=2000)
    app = ControlStation(args, log_q)
    try:
        app.mainloop()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()