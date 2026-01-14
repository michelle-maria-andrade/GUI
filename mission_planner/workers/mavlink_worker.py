from PyQt6.QtCore import QThread, pyqtSignal
from pymavlink import mavutil
import time
from datetime import datetime


class MavlinkGPSWorker(QThread):
    gps_update = pyqtSignal(float, float, float, str, bool)
    heartbeat = pyqtSignal(bool)

    # Consider the link "alive" if *any* MAVLink message is received within this window.
    # This avoids status flapping when HEARTBEAT frequency is low/irregular.
    OFFLINE_TIMEOUT_S = 10.0

    def __init__(self, udp_port: int):
        super().__init__()
        self.udp_port = udp_port
        self.running = True

    def run(self):
        conn = mavutil.mavlink_connection(
            f"udp:127.0.0.1:{self.udp_port}"
        )

        # Block until a heartbeat is seen once so we know the stream is valid.
        conn.wait_heartbeat()

        now = time.time()
        last_msg_time = now
        alive = True
        self.heartbeat.emit(True)

        while self.running:
            msg = conn.recv_match(blocking=False)

            if msg:
                # Treat *any* received MAVLink message as proof of liveness.
                last_msg_time = time.time()

                if msg.get_type() == "HEARTBEAT":
                    if not alive:
                        self.heartbeat.emit(True)
                        alive = True

                elif msg.get_type() == "GLOBAL_POSITION_INT":
                    lat = msg.lat / 1e7
                    lon = msg.lon / 1e7
                    alt = msg.relative_alt / 1000.0
                    ts = datetime.now().strftime("%H:%M:%S")
                    self.gps_update.emit(lat, lon, alt, ts, True)

            if time.time() - last_msg_time > self.OFFLINE_TIMEOUT_S:
                if alive:
                    self.heartbeat.emit(False)
                    alive = False

            time.sleep(0.05)

    def stop(self):
        self.running = False
