"""
CAN interface wrapper using python-can.
Handles send (#200) and receive (#201) for ARS408.

CAN ID calculation (ARS408 spec):
  Send  (RadarCfg)   = 0x200 + sensor_id * 0x10
  Recv  (RadarState) = 0x201 + sensor_id * 0x10
"""

import threading
import time
from typing import Callable, Optional

try:
    import can
    CAN_AVAILABLE = True
except ImportError:
    CAN_AVAILABLE = False

BASE_CFG_ID   = 0x200
BASE_STATE_ID = 0x201


class CanInterface:
    def __init__(self, channel: str = "vcan0", bustype: str = "socketcan"):
        self.channel = channel
        self.bustype = bustype
        self.bus: Optional[object] = None
        self.running = False
        self._rx_thread: Optional[threading.Thread] = None
        self._state_callback: Optional[Callable[[bytes], None]] = None
        self._cfg_id   = BASE_CFG_ID    # 送信 CAN ID
        self._state_id = BASE_STATE_ID  # 受信 CAN ID

    # ------------------------------------------------------------------
    # CAN ID 管理
    # ------------------------------------------------------------------
    @property
    def cfg_id(self) -> int:
        return self._cfg_id

    @property
    def state_id(self) -> int:
        return self._state_id

    def set_sensor_id(self, sensor_id: int):
        """現在のレーダー Sensor ID を設定し、送受信 CAN ID を両方更新する。"""
        self._cfg_id   = BASE_CFG_ID   + sensor_id * 0x10
        self._state_id = BASE_STATE_ID + sensor_id * 0x10

    def set_state_id(self, state_id: int):
        """受信 CAN ID のみを更新する（#200 送信後に SensorID が変わる場合）。"""
        self._state_id = state_id

    # ------------------------------------------------------------------
    # 接続管理
    # ------------------------------------------------------------------
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

    # ------------------------------------------------------------------
    # 送受信
    # ------------------------------------------------------------------
    def send_cfg(self, data: bytes) -> bool:
        """CAN#200 (RadarCfg) を送信する。送信先 ID = self._cfg_id"""
        if not self.bus:
            return False
        try:
            msg = can.Message(
                arbitration_id=self._cfg_id,
                data=data,
                is_extended_id=False,
            )
            self.bus.send(msg)
            return True
        except Exception as e:
            raise RuntimeError(f"CAN send failed: {e}")

    def register_state_callback(self, callback: Callable[[bytes], None]):
        """CAN#201 受信時のコールバックを登録する。RX スレッドから呼ばれる。"""
        self._state_callback = callback

    def _rx_loop(self):
        while self.running and self.bus:
            try:
                msg = self.bus.recv(timeout=0.5)
                if msg is None:
                    continue
                if msg.arbitration_id == self._state_id:
                    if self._state_callback and len(msg.data) >= 8:
                        self._state_callback(bytes(msg.data[:8]))
            except Exception:
                if self.running:
                    time.sleep(0.1)

    @property
    def is_connected(self) -> bool:
        return self.bus is not None
