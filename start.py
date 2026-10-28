from __future__ import annotations

import asyncio
import csv
import io
import json
import threading
import time
import webbrowser

from collections import deque
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse


HOST = "127.0.0.1"
PORT = 8765
DEVICE_NAME = "BB JEEP"

SENSOR_NAMES = {
    1: ("left", "Driver (left)"),
    2: ("right", "Passenger (right)"),
    3: ("front", "Front"),
    4: ("rear", "Rear"),
}

UUID_SUFFIX = "-e8f2-537e-4f6c-d104768a1214"


def sensor_uuid(number: int) -> str:
    if not 1 <= number <= 255:
        raise ValueError("Sensor number must be between 1 and 255")

    return f"19b100{number:02x}{UUID_SUFFIX}"


UUID_TO_SENSOR = {
    sensor_uuid(number): {
        "key": key,
        "label": label
    }
    for number, (key, label) in SENSOR_NAMES.items()
}

DIRECTION_UUID = sensor_uuid(5)
DIRECTION_NAMES = {
    0: "Park",
    3: "Drive",
    4: "Reverse",
}


def decode_notification(uuid: str, payload: bytes):
    """Decode the one-byte direction or two-byte distance BLE value."""

    uuid = str(uuid).lower()

    if uuid == DIRECTION_UUID:
        if len(payload) != 1 or payload[0] not in DIRECTION_NAMES:
            return None
        return "direction", payload[0]

    sensor = UUID_TO_SENSOR.get(uuid)

    if sensor is None or len(payload) != 2:
        return None

    distance = int.from_bytes(
        payload,
        byteorder="little",
        signed=False
    )

    return sensor["key"], distance


class BluetoothMonitor:

    def __init__(self):

        self.lock = threading.Lock()
        self.wanted = threading.Event()
        self.shutdown = threading.Event()

        self.connected = False
        self.status = "Disconnected"

        self.latest = {}
        self.direction = None
        self.last_at = None

        self.packets = 0

        self.sensor_packets = {}

        self.available = []
        self.subscribed = []

        self.logs = deque(maxlen=80)
        self.samples = deque(maxlen=2500)

        self.thread = None

        self.log("Ready. Press Connect to find your JEEP.")

    def start(self):

        if self.thread is None:
            self.thread = threading.Thread(
                target=lambda: asyncio.run(self.worker()),
                daemon=True
            )
            self.thread.start()

    def log(self, message):

        with self.lock:
            self.logs.append({
                "time": time.strftime("%H:%M:%S"),
                "message": str(message)[:300]
            })

        print("[BB JEEP]", message, flush=True)

    def set_status(self, text, connected=None):

        with self.lock:
            self.status = text

            if connected is not None:
                self.connected = connected

    def connect(self):

        if self.wanted.is_set():
            return False, "Already connecting or connected"

        with self.lock:

            self.latest.clear()
            self.direction = None
            self.samples.clear()

            self.packets = 0
            self.sensor_packets.clear()

            self.last_at = None
            self.available = []
            self.subscribed = []

        self.wanted.set()

        self.set_status(
            "Searching for " + DEVICE_NAME,
            False
        )

        self.log("Searching for " + DEVICE_NAME)

        return True, "Searching"

    def disconnect(self):

        self.wanted.clear()
        self.set_status("Disconnecting...")

    def handle_notification(self, uuid, payload):

        reading = decode_notification(uuid, payload)

        if reading is None:
            self.log(
                f"Unexpected BLE packet from {uuid}: "
                f"{bytes(payload).hex(' ')}"
            )
            return

        key, value = reading

        now = time.strftime("%Y-%m-%d %H:%M:%S")

        if key == "direction":
            name = DIRECTION_NAMES[value]

            with self.lock:
                changed = self.direction != value
                self.direction = value
                self.last_at = now
                self.packets += 1

                if changed:
                    self.samples.append({
                        "time": now,
                        "direction": name
                    })

            # Log outside the lock to avoid deadlock
            if changed:
                self.log(f"Motion direction: {name}")
            return

        distance_cm = value

        with self.lock:
            self.latest[key] = distance_cm
            self.last_at = now
            self.packets += 1

            sensor_count = self.sensor_packets.get(key, 0) + 1
            self.sensor_packets[key] = sensor_count

            direction_name = (
                DIRECTION_NAMES[self.direction]
                if self.direction is not None else ""
            )
            self.samples.append({
                "time": now,
                key: distance_cm,
                "direction": direction_name
            })

        if sensor_count % 5 == 1:
            label = UUID_TO_SENSOR[
                str(uuid).lower()
            ]["label"]

            text = (
                "No echo"
                if distance_cm == 0
                else f"{distance_cm} cm"
            )

            self.log(f"{label}: {text}")

    def snapshot(self):

        with self.lock:
            return {
                "status": self.status,
                "connected": self.connected,
                "packets": self.packets,
                "latest": dict(self.latest),
                "direction": (
                    DIRECTION_NAMES[self.direction]
                    if self.direction is not None else None
                ),
                "last_at": self.last_at,

                "sensors": [
                    {
                        "key": key,
                        "label": label,
                        "uuid": sensor_uuid(number)
                    }
                    for number, (key, label)
                    in SENSOR_NAMES.items()
                ],

                "available": list(self.available),
                "subscribed": list(self.subscribed),
                "logs": list(self.logs),
            }

    def export_csv(self):

        with self.lock:
            samples = list(self.samples)

        output = io.StringIO()

        fields = ["time"] + [
            key for key, _ in SENSOR_NAMES.values()
        ] + ["direction"]

        writer = csv.DictWriter(
            output,
            fieldnames=fields
        )

        writer.writeheader()
        writer.writerows(samples)

        return output.getvalue().encode("utf-8")

    async def connected_session(self, device, BleakClient):

        disconnected = asyncio.Event()

        client = BleakClient(
            device,
            disconnected_callback=lambda _: disconnected.set()
        )

        try:

            async with client:

                self.set_status(
                    "Connected to " + DEVICE_NAME,
                    True
                )

                self.log("Bluetooth connected")

                choices = []
                available = []

                for service in client.services:

                    for characteristic in service.characteristics:

                        uuid = characteristic.uuid.lower()

                        props = [
                            prop.lower()
                            for prop in characteristic.properties
                        ]

                        available.append({
                            "uuid": uuid,
                            "properties": props
                        })

                        if (
                            (uuid in UUID_TO_SENSOR or uuid == DIRECTION_UUID)
                            and (
                                "notify" in props
                                or "indicate" in props
                            )
                        ):
                            choices.append(characteristic)

                with self.lock:
                    self.available = available

                subscribed = []

                for char in choices:

                    try:

                        await client.start_notify(
                            char,
                            lambda sender, data:
                                self.handle_notification(
                                    sender.uuid,
                                    data
                                )
                        )

                        subscribed.append(char.uuid)

                        if char.uuid.lower() == DIRECTION_UUID:
                            label = "Gear"
                            
                            try:
                                initial = await client.read_gatt_char(char)
                                self.handle_notification(char.uuid, initial)
                            except Exception as exc:
                                self.log(
                                    f"Could not read initial gear: {exc}"
                                )
                        else:
                            label = UUID_TO_SENSOR[
                                char.uuid.lower()
                            ]["label"]

                        self.log(
                            "Listening for " + label
                        )

                    except Exception as exc:

                        self.log(
                            f"Failed to subscribe to "
                            f"{char.uuid}: {exc}"
                        )

                with self.lock:
                    self.subscribed = subscribed

                if not subscribed:
                    self.log(
                        "No configured distance or motion "
                        "characteristics advertised with Notify"
                    )

                while (
                    self.wanted.is_set()
                    and not self.shutdown.is_set()
                    and client.is_connected
                    and not disconnected.is_set()
                ):
                    await asyncio.sleep(0.2)

        finally:

            self.set_status("Disconnected", False)

            with self.lock:
                self.subscribed = []

    async def worker(self):

        try:
            from bleak import BleakClient, BleakScanner

        except ImportError:

            self.set_status(
                "Missing Bleak; run: "
                "python -m pip install -r requirements.txt",
                False
            )

            self.log(
                "Install Bleak with "
                "python -m pip install -r requirements.txt"
            )

            return

        while not self.shutdown.is_set():

            if not self.wanted.is_set():
                await asyncio.sleep(0.2)
                continue

            try:

                self.set_status(
                    "Searching for " + DEVICE_NAME,
                    False
                )

                device = await BleakScanner.find_device_by_name(
                    DEVICE_NAME,
                    timeout=8
                )

                if not self.wanted.is_set():
                    continue

                if device is None:

                    self.log(
                        "BB JEEP not found; retrying"
                    )

                else:

                    await self.connected_session(
                        device,
                        BleakClient
                    )

                    if self.wanted.is_set():
                        self.log(
                            "Disconnected; retrying"
                        )

            except Exception as exc:

                self.set_status(
                    "Connection error; retrying",
                    False
                )

                self.log(
                    f"{type(exc).__name__}: {exc}"
                )

            for _ in range(10):

                if (
                    self.shutdown.is_set()
                    or not self.wanted.is_set()
                ):
                    break

                await asyncio.sleep(0.2)

monitor = BluetoothMonitor()

class Handler(SimpleHTTPRequestHandler):

    def __init__(self, *args, **kwargs):

        super().__init__(
            *args,
            directory=str(
                Path(__file__).resolve().parent
            ),
            **kwargs
        )

    def respond(self, obj, status=200):

        body = json.dumps(obj).encode("utf-8")

        self.send_response(status)

        self.send_header(
            "Content-Type",
            "application/json; charset=utf-8"
        )

        self.send_header(
            "Content-Length",
            str(len(body))
        )

        self.send_header(
            "Cache-Control",
            "no-store"
        )

        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):

        path = urlparse(self.path).path

        if path == "/api/status":
            return self.respond(monitor.snapshot())

        if path == "/api/export.csv":

            body = monitor.export_csv()

            self.send_response(200)

            self.send_header(
                "Content-Type",
                "text/csv; charset=utf-8"
            )

            self.send_header(
                "Content-Disposition",
                'attachment; filename="bb-jeep-sensors.csv"'
            )

            self.send_header(
                "Content-Length",
                str(len(body))
            )

            self.end_headers()
            self.wfile.write(body)

            return

        return super().do_GET()

    def do_POST(self):

        path = urlparse(self.path).path

        if path not in (
            "/api/connect",
            "/api/disconnect"
        ):
            return self.respond(
                {"error": "Not found"},
                404
            )

        if path == "/api/connect":

            ok, message = monitor.connect()

            return self.respond(
                {
                    "ok": ok,
                    "message": message
                },
                200 if ok else 409
            )

        monitor.disconnect()

        return self.respond({"ok": True})


if __name__ == "__main__":

    monitor.start()

    server = ThreadingHTTPServer(
        (HOST, PORT),
        Handler
    )

    address = f"http://{HOST}:{PORT}/"

    print(
        "Parking and Blind Spot Assist Monitor:",
        address,
        flush=True
    )

    webbrowser.open(address)

    try:
        server.serve_forever()

    except KeyboardInterrupt:
        pass

    finally:

        monitor.wanted.clear()
        monitor.shutdown.set()

        server.server_close()

        if monitor.thread:
            monitor.thread.join(timeout=8)
