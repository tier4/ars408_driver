"""
CAN interface wrapper using python-can.
Handles send (#200) and receive (#201) for ARS408.
"""

import threading
import time
from typing import Callable, Optional

try:
    import can
    CAN_AVAILABLE = True
except ImportError:
    CAN_AVAILABLE = False

CAN_ID_RADAR_CFG   = 0x200
CAN_ID_RADAR_STATE = 0x201


class CanInterface:
    def __init__(self, channel: str = "vcan0", bustype: str = "socketcan"):
        self.channel = channel
        self.bustype = bustype
        self.bus: Optional[object] = None
        self.running = False
        self._rx_thread: Optional[threading.Thread] = None
        self._state_callback: Optional[Callable[[bytes], None]] = None

    def connect(self) -> bool:
        if not CAN_AVAILABLE:
            raise RuntimeError("python-can not installed. Run: pip install python-can")
        try:
            self.bus = can.interface.Bus(channel=self.channel, interface=self.bustype)
            self.running = True
            self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self._rx_thread.start()
            return True
        except Exception as e:
            raise RuntimeError(f"CAN connection failed ({self.channel}): {e}")

    def disconnect(self):
        self.running = False
        if self.bus:
            try:
                self.bus.shutdown()
            except Exception:
                pass
            self.bus = None

    def send_cfg(self, data: bytes) -> bool:
        """Send CAN#200 (RadarCfg) message."""
        if not self.bus:
            return False
        try:
            msg = can.Message(
                arbitration_id=CAN_ID_RADAR_CFG,
                data=data,
                is_extended_id=False,
            )
            self.bus.send(msg)
            return True
        except Exception as e:
            raise RuntimeError(f"CAN send failed: {e}")

    def register_state_callback(self, callback: Callable[[bytes], None]):
        """Register callback for received #201 messages. Called from rx thread."""
        self._state_callback = callback

    def _rx_loop(self):
        while self.running and self.bus:
            try:
                msg = self.bus.recv(timeout=0.5)
                if msg is None:
                    continue
                if msg.arbitration_id == CAN_ID_RADAR_STATE:
                    if self._state_callback and len(msg.data) >= 8:
                        self._state_callback(bytes(msg.data[:8]))
            except Exception:
                if self.running:
                    time.sleep(0.1)

    @property
    def is_connected(self) -> bool:
        return self.bus is not None
