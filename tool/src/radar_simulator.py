"""
ARS408 Radar Simulator

CAN#200 (RadarCfg) を受信し、設定を内部状態に反映して
CAN#201 (RadarState) として vcan0 (または任意のインターフェース) に送り返す。

実機フローのシミュレーション:
  GUI → #200 → [vcan0] → radar_simulator.py
                           ↓ 内部状態を更新
  GUI ← #201 ← [vcan0] ← radar_simulator.py

使い方:
  python3 radar_simulator.py [--channel vcan0] [--interval 0.5]

オプション:
  --channel   CAN インターフェース名 (default: vcan0)
  --interval  #201 を定期送信する間隔[秒] (default: 0.5)
              0 にすると #200 受信時のみ送信
"""

import argparse
import time
import threading

try:
    import can
    CAN_AVAILABLE = True
except ImportError:
    CAN_AVAILABLE = False

from radar_cfg import RadarCfg, decode_can200
from radar_state import RadarState


# ---------------------------------------------------------------------------
# 内部状態のデフォルト値（実機の初期状態 40 18 80 00 A0 F4 00 00 に対応）
# ---------------------------------------------------------------------------
DEFAULT_STATE = RadarState(
    NVMReadStatus    = 1,   # OK
    NVMwriteStatus   = 0,   # Failed (書き込み未実施)
    MaxDistance      = 196,
    PersistentError  = 0,
    Interference     = 0,
    TemperatureError = 0,
    TemporaryError   = 0,
    VoltageError     = 0,
    RadarPower       = 1,   # -3dB
    SortIndex        = 2,   # By RCS
    SensorID         = 0,
    MotionRxState    = 3,   # Speed+Yaw Missing
    SendExtInfo      = 1,
    SendQuality      = 1,
    OutputType       = 1,   # Objects
    CtrlRelay        = 0,
    RCS_Threshold    = 0,   # Standard
)


def state_to_can201(s: RadarState) -> bytes:
    """
    RadarState → CAN#201 8バイトに変換する。
    ParseRadarState() の逆変換。

    ビット配置:
      byte0         : NVMwriteStatus(bit7), NVMReadStatus(bit6)
      bytes 1-2     : MaxDistance  raw10 = byte1<<2 | byte2>>6  (res=2m)
      byte2 bits5-1 : PersistentError(5) Interference(4) TemperatureError(3)
                      TemporaryError(2) VoltageError(1)
      byte3/byte4b7 : RadarPower = (byte3<<1)|(byte4>>7)  (3bit)
      byte4 bits6-4 : SortIndex
      byte4 bits2-0 : SensorID
      byte5 bit7-6  : MotionRxState
      byte5 bit5    : SendExtInfo
      byte5 bit4    : SendQuality
      byte5 bits3-2 : OutputType
      byte5 bit1    : CtrlRelay
      byte7 bits4-2 : RCS_Threshold
    """
    d = bytearray(8)

    d[0] = ((s.NVMwriteStatus & 1) << 7) | ((s.NVMReadStatus & 1) << 6)

    raw10 = (s.MaxDistance // 2) & 0x3FF
    d[1] = (raw10 >> 2) & 0xFF
    d[2] = (((raw10 & 0x03) << 6) |
            ((s.PersistentError  & 1) << 5) |
            ((s.Interference     & 1) << 4) |
            ((s.TemperatureError & 1) << 3) |
            ((s.TemporaryError   & 1) << 2) |
            ((s.VoltageError     & 1) << 1)) & 0xFF

    rp = s.RadarPower & 0x07
    d[3] = (rp >> 1) & 0xFF
    d[4] = (((rp & 0x01) << 7) |
            ((s.SortIndex & 0x07) << 4) |
            (s.SensorID  & 0x07)) & 0xFF

    d[5] = (((s.MotionRxState & 0x03) << 6) |
            ((s.SendExtInfo   & 1)    << 5) |
            ((s.SendQuality   & 1)    << 4) |
            ((s.OutputType    & 0x03) << 2) |
            ((s.CtrlRelay     & 1)    << 1)) & 0xFF

    d[7] = ((s.RCS_Threshold & 0x07) << 2) & 0xFF

    return bytes(d)


def apply_cfg_to_state(state: RadarState, cfg: RadarCfg) -> RadarState:
    """RadarCfg の valid フィールドを RadarState に反映する。"""
    import copy
    s = copy.copy(state)

    if cfg.MaxDistance_valid:
        s.MaxDistance = (cfg.MaxDistance // 2) * 2   # 2m 単位に丸め

    if cfg.SensorID_valid:
        s.SensorID = cfg.SensorID

    if cfg.RadarPower_valid:
        s.RadarPower = cfg.RadarPower

    if cfg.OutputType_valid:
        s.OutputType = cfg.OutputType

    if cfg.SendQuality_valid:
        s.SendQuality = int(cfg.SendQuality)

    if cfg.SendExtInfo_valid:
        s.SendExtInfo = int(cfg.SendExtInfo)

    if cfg.SortIndex_valid:
        s.SortIndex = cfg.SortIndex

    if cfg.CtrlRelay_valid:
        s.CtrlRelay = int(cfg.CtrlRelay)

    if cfg.RCS_Threshold_valid:
        s.RCS_Threshold = cfg.RCS_Threshold

    # #200 を受け付けたので NVMwriteStatus を OK に
    if cfg.StoreInNVM_valid and cfg.StoreInNVM:
        s.NVMwriteStatus = 1
        s.NVMReadStatus  = 1

    return s


class RadarSimulator:
    def __init__(self, channel: str = "vcan0", interval: float = 0.5):
        self.channel  = channel
        self.interval = interval
        self._state   = DEFAULT_STATE
        self._lock    = threading.Lock()
        self._bus     = None
        self._running = False

    def start(self):
        if not CAN_AVAILABLE:
            raise RuntimeError("python-can が見つかりません。pip install python-can を実行してください。")

        self._bus = can.interface.Bus(channel=self.channel, interface="socketcan")
        self._running = True

        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        if self.interval > 0:
            self._tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
            self._tx_thread.start()

        print(f"[Simulator] 起動しました  channel={self.channel}  interval={self.interval}s")
        print(f"[Simulator] CAN#200 待受中 ... (Ctrl+C で終了)\n")

    def stop(self):
        self._running = False
        if self._bus:
            self._bus.shutdown()

    def _rx_loop(self):
        while self._running:
            try:
                msg = self._bus.recv(timeout=1.0)
                if msg is None:
                    continue
                if msg.arbitration_id == 0x200 and len(msg.data) >= 8:
                    self._on_received_200(bytes(msg.data))
            except Exception as e:
                if self._running:
                    print(f"[Simulator] RX エラー: {e}")

    def _on_received_200(self, data: bytes):
        hex_str = " ".join(f"{b:02X}" for b in data)
        print(f"[Simulator] #200 受信: {hex_str}")

        cfg = decode_can200(data)

        with self._lock:
            self._state = apply_cfg_to_state(self._state, cfg)
            raw201 = state_to_can201(self._state)

        hex_201 = " ".join(f"{b:02X}" for b in raw201)
        print(f"[Simulator] #201 送信: {hex_201}\n")
        self._send_201(raw201)

    def _tx_loop(self):
        """定期的に #201 を送信する（実機レーダーの定期送信に相当）。"""
        while self._running:
            time.sleep(self.interval)
            with self._lock:
                raw201 = state_to_can201(self._state)
            self._send_201(raw201)

    def _send_201(self, data: bytes):
        try:
            msg = can.Message(
                arbitration_id=0x201,
                data=data,
                is_extended_id=False,
            )
            self._bus.send(msg)
        except Exception as e:
            print(f"[Simulator] #201 送信エラー: {e}")


def main():
    parser = argparse.ArgumentParser(description="ARS408 Radar Simulator")
    parser.add_argument("--channel",  default="vcan0", help="CAN インターフェース名")
    parser.add_argument("--interval", type=float, default=0.5,
                        help="#201 定期送信間隔[秒] (0=受信時のみ)")
    args = parser.parse_args()

    sim = RadarSimulator(channel=args.channel, interval=args.interval)
    sim.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[Simulator] 停止します ...")
        sim.stop()


if __name__ == "__main__":
    main()
