from __future__ import annotations
import asyncio
import json
import math
import threading
import time
import webbrowser
from collections import deque
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

try:
    from bleak import BleakClient, BleakScanner
except ImportError as exc:
    raise SystemExit(
        "Missing Bleak. Run: python3 -m pip install -r requirements.txt\n"
        "Then run: python ./start.py"
    ) from exc

HOST = "127.0.0.1"
PORT = 8765
DEFAULT_NAME = "BB JEEP"
PREFERRED_CHARACTERISTIC = "f0a00002-0424-4563-8c1a-020000000001"
DIRECTIONS = ("front", "left", "right", "rear")


def parsed_message(text: str):
    text = text.strip()
    if not text:
        return None
    try:
        obj = json.loads(text)
    except ValueError:
        if text.count(",") == 3:
            try:
                a, b, c, d = (float(s) for s in text.split(","))
                obj = {"front": a, "left": b, "right": c, "rear": d}
            except ValueError:
                return None
        else:
            obj = {}
            for p in text.replace(";", ",").split(","):
                split = p.split("=", 1) if "=" in p else p.split(":", 1)
                if len(split) == 2:
                    obj[split[0].strip()] = split[1].strip()
    if not isinstance(obj, dict):
        return None
    obj = {str(k).lower(): v for k, v in obj.items()}
    aliases = {
        "front": ("front", "f"), "left": ("left", "l"),
        "right": ("right", "r"), "rear": ("rear", "back", "b"),
        "battery": ("battery", "battery_v", "voltage", "v"),
    }
    data = {}
    for name, keys in aliases.items():
        v = next((obj[k] for k in keys if k in obj), None)
        try:
            n = float(v) if v is not None and str(v).strip() else None
            data[name] = n if n is None or math.isfinite(n) else None
        except (ValueError, TypeError):
            data[name] = None
    v = next((obj[k] for k in ("moving", "motion", "m") if k in obj), None)
    data["moving"] = bool(v) if isinstance(v, bool) else (
        str(v).lower() in ("1", "true", "moving") if v is not None and str(v).lower() in ("1", "true", "moving", "0", "false", "still") else None
    )
    v = next((obj[k] for k in ("simulated", "demo") if k in obj), None)
    data["simulated"] = bool(v) if isinstance(v, bool) else (
        str(v).lower() in ("true", "1") if v is not None and str(v).lower() in ("true", "1", "false", "0") else None
    )
    if all(data.get(key) is None for key in DIRECTIONS) and data["battery"] is None and data["moving"] is None:
        return None
    return data


class BluetoothMonitor:
    def __init__(self):
        self.lock = threading.Lock()
        self.wanted = threading.Event()
        self.shutdown = threading.Event()
        self.device_name = DEFAULT_NAME
        self.notify_uuid = ""  # blank = autodetect available notifying characteristic
        self.status = "Disconnected"
        self.connected = False
        self.subscribe = []
        self.available = []
        self.last = {}
        self.last_at = None
        self.count = 0
        self.logs = deque(maxlen=80)
        self.samples = deque(maxlen=2500)
        self.buffers = {}
        self.log("Ready. Press Connect to find BB JEEP.")
        self.worker = threading.Thread(target=self._run_thread, daemon=True)
        self.worker.start()

    def log(self, message):
        with self.lock:
            self.logs.append({"time": time.strftime("%H:%M:%S"), "message": str(message)[:300]})
        print("[BB JEEP]", message, flush=True)

    def set_status(self, status, connected=None):
        with self.lock:
            self.status = status
            if connected is not None:
                self.connected = connected

    def snapshot(self):
        with self.lock:
            return {
                "status": self.status, "connected": self.connected,
                "wanted": self.wanted.is_set(), "name": self.device_name,
                "notify_uuid": self.notify_uuid, "characteristics": list(self.available),
                "subscribed": list(self.subscribe), "latest": dict(self.last),
                "last_at": self.last_at, "packets": self.count,
                "logs": list(self.logs),
            }

    def connect(self, name, char_uuid):
        if self.wanted.is_set():
            return False, "Already connecting or connected. Disconnect before changing settings."
        with self.lock:
            self.device_name = name.strip() or DEFAULT_NAME
            self.notify_uuid = char_uuid.strip().lower()
            self.last = {}
            self.last_at = None
            self.count = 0
            self.subscribe = []
            self.available = []
            self.samples.clear()
            self.buffers.clear()
            self.status = "Searching for " + self.device_name
        self.wanted.set()
        self.log("Searching for " + self.device_name)
        return True, "Searching"

    def disconnect(self):
        self.wanted.clear()
        self.set_status("Disconnecting...")
        self.log("Manual disconnect requested.")

    def csv(self):
        import csv
        import io
        with self.lock:
            samples = list(self.samples)
        buf = io.StringIO()
        columns = ["time", "front", "left", "right", "rear", "moving", "battery", "simulated"]
        writer = csv.DictWriter(buf, fieldnames=columns)
        writer.writeheader()
        writer.writerows(samples)
        return buf.getvalue().encode("utf-8")

    def handle_message(self, data_bytes: bytes, uuid: str):
        """Arduino sketch sends JSON in <=20-byte chunks ending in newline."""
        raw = bytes(data_bytes).decode("utf-8", errors="replace").replace("\x00", "")
        key = uuid.lower()
        buffer = self.buffers.get(key, "") + raw
        if len(buffer) > 16000:
            self.buffers[key] = ""
            self.log("Oversize notification stream discarded from " + key)
            return
        while "\n" in buffer:
            frame, buffer = buffer.split("\n", 1)
            self.handle_frame(frame.strip())
        candidate = buffer.strip()
        if candidate.startswith("{") and candidate.endswith("}"):
            try:
                json.loads(candidate)
                self.handle_frame(candidate)
                buffer = ""
            except ValueError:
                pass
        self.buffers[key] = buffer

    def handle_frame(self, frame):
        if not frame:
            return
        parsed = parsed_message(frame)
        if parsed is None:
            self.log("Other BLE data: " + frame[:150])
            return
        when = time.strftime("%Y-%m-%d %H:%M:%S")
        with self.lock:
            for k, v in parsed.items():
                if v is not None:
                    self.last[k] = v
            self.count += 1
            self.last_at = when
            self.samples.append({"time": when, **parsed})

    async def _connected_session(self, device):
        disconnected = asyncio.Event()
        client = BleakClient(device, disconnected_callback=lambda _: disconnected.set())
        try:
            async with client:
                self.set_status("Connected to " + self.device_name, True)
                self.log("CONNECTED to " + self.device_name + " (no automatic disconnect timer).")
                services = client.services
                notify_chars = []
                available = []
                for service in services:
                    for char in service.characteristics:
                        props = [p.lower() for p in char.properties]
                        available.append({"uuid": char.uuid, "properties": props})
                        if "notify" in props or "indicate" in props:
                            notify_chars.append(char)
                with self.lock:
                    self.available = available
                if self.notify_uuid:
                    targets = [c for c in notify_chars if c.uuid.lower() == self.notify_uuid]
                    if not targets:
                        self.log("Specified UUID is not a notifying characteristic.")
                else:
                    preferred = [c for c in notify_chars if c.uuid.lower() == PREFERRED_CHARACTERISTIC]
                    targets = preferred if preferred else notify_chars
                subscribed = []
                for char in targets:
                    try:
                        await client.start_notify(
                            char,
                            lambda sender, payload: self.handle_message(payload, sender.uuid)
                        )
                        subscribed.append(char.uuid)
                        self.log("Listening for BLE notifications: " + char.uuid)
                    except Exception as ex:
                        self.log(f"Could not subscribe to {char.uuid}: {ex}")
                with self.lock:
                    self.subscribe = subscribed
                if not subscribed:
                    self.log("Connected, but no notifications were available.")
                while self.wanted.is_set() and not self.shutdown.is_set() and client.is_connected and not disconnected.is_set():
                    await asyncio.sleep(0.3)
        finally:
            self.set_status("Disconnected", False)
            with self.lock:
                self.subscribe = []
                self.buffers.clear()

    async def _run(self):
        while not self.shutdown.is_set():
            if not self.wanted.is_set():
                await asyncio.sleep(0.2)
                continue
            try:
                self.set_status("Searching for " + self.device_name, False)
                device = await BleakScanner.find_device_by_name(self.device_name, timeout=8)
                if not self.wanted.is_set():
                    continue
                if device is None:
                    self.log(f"{self.device_name} not detected. Retrying search...")
                    self.set_status("Searching again for " + self.device_name, False)
                else:
                    self.set_status("Connecting to " + self.device_name, False)
                    await self._connected_session(device)
                    if self.wanted.is_set():
                        self.log("Connection lost; reconnecting automatically...")
            except Exception as ex:
                self.set_status("Connection issue; retrying", False)
                self.log(type(ex).__name__ + ": " + str(ex))
            for _ in range(15):
                if self.shutdown.is_set() or not self.wanted.is_set():
                    break
                await asyncio.sleep(0.2)
        self.set_status("Stopped", False)

    def _run_thread(self):
        asyncio.run(self._run())


monitor = BluetoothMonitor()


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(Path(__file__).resolve().parent), **kwargs)

    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/api/status":
            return self.json_response(200, monitor.snapshot())
        if path == "/api/export.csv":
            body = monitor.csv()
            self.send_response(200)
            self.send_header("Content-Type", "text/csv; charset=utf-8")
            self.send_header("Content-Disposition", 'attachment; filename="bb-jeep-sensors.csv"')
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
            return
        super().do_GET()

    def json_response(self, status, obj):
        body = json.dumps(obj, allow_nan=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        if urlparse(self.path).path not in ("/api/connect", "/api/disconnect"):
            return self.json_response(404, {"error": "Not found"})
        if self.headers.get("Content-Length") is None:
            return self.json_response(411, {"error": "Missing request length"})
        try:
            n = int(self.headers["Content-Length"])
            if n < 0 or n > 2048:
                return self.json_response(413, {"error": "Invalid body size"})
            args = json.loads(self.rfile.read(n) or b"{}")
            if not isinstance(args, dict):
                raise ValueError("Expected JSON object")
        except (ValueError, UnicodeDecodeError, json.JSONDecodeError):
            return self.json_response(400, {"error": "Invalid JSON"})
        if urlparse(self.path).path == "/api/disconnect":
            monitor.disconnect()
            return self.json_response(200, {"ok": True})
        name = str(args.get("name", DEFAULT_NAME))[:60]
        uuid = str(args.get("uuid", ""))[:100]
        ok, message = monitor.connect(name, uuid)
        return self.json_response(200 if ok else 409, {"ok": ok, "message": message})


if __name__ == "__main__":
    server = ThreadingHTTPServer((HOST, PORT), Handler)
    url = f"http://{HOST}:{PORT}/"
    print(f"BB JEEP Python BLE Monitor: {url}", flush=True)
    print("Press Ctrl+C to stop the server and disconnect Bluetooth.", flush=True)
    webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        monitor.wanted.clear()
        monitor.shutdown.set()
        server.server_close()
        monitor.worker.join(timeout=8)
        print("Stopped.")
