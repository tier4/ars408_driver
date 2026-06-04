"""
CAN#201 (RadarState) decoder for ARS408.
Porting logic from ars408_driver.cpp ParseRadarState().
"""

from dataclasses import dataclass, field

SORT_INDEX = {0: "No Sort", 1: "By Range", 2: "By RCS", 3: "Sort Error"}
RADAR_POWER = {0: "Standard", 1: "-3dB", 2: "-6dB", 3: "-9dB", 4: "Error"}
OUTPUT_TYPE = {0: "None", 1: "Objects", 2: "Clusters", 3: "Error"}
MOTION_RX   = {0: "Input OK", 1: "Speed Missing", 2: "Yaw Rate Missing", 3: "Speed+Yaw Missing"}
RCS_THRESH  = {0: "Standard", 1: "High Sensitivity"}
STATUS      = {0: "Failed", 1: "OK"}
ERROR_FLAG  = {0: "OK", 1: "ERROR"}


@dataclass
class RadarState:
    # Byte 0
    NVMReadStatus:    int = 0  # bit 6
    NVMwriteStatus:   int = 0  # bit 7

    # Bytes 1-2: MaxDistance (start=22, len=10, res=2m)
    MaxDistance: int = 0

    # Byte 2: error flags
    PersistentError:   int = 0  # bit 21
    Interference:      int = 0  # bit 20
    TemperatureError:  int = 0  # bit 19
    TemporaryError:    int = 0  # bit 18
    VoltageError:      int = 0  # bit 17

    # Bytes 3-4: RadarPower (start=39, len=3)
    RadarPower: int = 0

    # Byte 4
    SortIndex: int = 0  # bits 38:36
    SensorID:  int = 0  # bits 34:32

    # Byte 5
    MotionRxState: int = 0  # bits 47:46
    SendExtInfo:   int = 0  # bit 45
    SendQuality:   int = 0  # bit 44
    OutputType:    int = 0  # bits 43:42
    CtrlRelay:     int = 0  # bit 41

    # Byte 7: RCS_Threshold (start=58, len=3 → byte7 bits 4:2)
    RCS_Threshold: int = 0


def parse_can201(data: bytes) -> RadarState:
    """
    Parse CAN#201 (RadarState) 8-byte message.
    Logic ported from ars408_driver.cpp ParseRadarState().
    """
    if len(data) < 8:
        raise ValueError(f"Expected 8 bytes, got {len(data)}")

    s = RadarState()

    # Byte 0
    s.NVMwriteStatus = (data[0] & 0x80) >> 7
    s.NVMReadStatus  = (data[0] & 0x40) >> 6

    # MaxDistance: bytes 1-2
    # raw10 = (byte1 << 2) | (byte2 >> 6)
    # distance_m = raw10 * 2   (res = 2m)
    raw10 = ((data[1] << 2) & 0x3FF) | ((data[2] & 0xC0) >> 6)
    s.MaxDistance = raw10 * 2

    # Byte 2: error flags
    s.PersistentError  = (data[2] & 0x20) >> 5
    s.Interference     = (data[2] & 0x10) >> 4
    s.TemperatureError = (data[2] & 0x08) >> 3
    s.TemporaryError   = (data[2] & 0x04) >> 2
    s.VoltageError     = (data[2] & 0x02) >> 1

    # RadarPower: bytes 3-4
    # power = (byte3 << 1) | (byte4 >> 7)
    s.RadarPower = ((data[3] & 0xFF) << 1) | ((data[4] & 0x80) >> 7)

    # Byte 4
    s.SortIndex = (data[4] & 0x70) >> 4
    s.SensorID  =  data[4] & 0x07

    # Byte 5
    s.MotionRxState = (data[5] & 0xC0) >> 6
    s.SendExtInfo   = (data[5] & 0x20) >> 5
    s.SendQuality   = (data[5] & 0x10) >> 4
    s.OutputType    = (data[5] & 0x0C) >> 2
    s.CtrlRelay     = (data[5] & 0x02) >> 1

    # RCS_Threshold: byte 7, bits 4:2 (start=58, len=3)
    s.RCS_Threshold = (data[7] & 0x1C) >> 2

    return s


def state_to_dict(s: RadarState) -> dict:
    """Convert RadarState to labeled dict for display."""
    return {
        "NVMReadStatus":   (s.NVMReadStatus,   STATUS.get(s.NVMReadStatus, "?")),
        "NVMwriteStatus":  (s.NVMwriteStatus,  STATUS.get(s.NVMwriteStatus, "?")),
        "MaxDistance":     (s.MaxDistance,      f"{s.MaxDistance} m"),
        "SensorID":        (s.SensorID,         str(s.SensorID)),
        "RadarPower":      (s.RadarPower,        RADAR_POWER.get(s.RadarPower, "?")),
        "OutputType":      (s.OutputType,        OUTPUT_TYPE.get(s.OutputType, "?")),
        "SendQuality":     (s.SendQuality,       "Active" if s.SendQuality else "Inactive"),
        "SendExtInfo":     (s.SendExtInfo,       "Active" if s.SendExtInfo else "Inactive"),
        "SortIndex":       (s.SortIndex,         SORT_INDEX.get(s.SortIndex, "?")),
        "CtrlRelay":       (s.CtrlRelay,         "Active" if s.CtrlRelay else "Inactive"),
        "RCS_Threshold":   (s.RCS_Threshold,     RCS_THRESH.get(s.RCS_Threshold, "?")),
        "MotionRxState":   (s.MotionRxState,     MOTION_RX.get(s.MotionRxState, "?")),
        "VoltageError":    (s.VoltageError,      ERROR_FLAG[s.VoltageError]),
        "TemporaryError":  (s.TemporaryError,    ERROR_FLAG[s.TemporaryError]),
        "TemperatureError":(s.TemperatureError,  ERROR_FLAG[s.TemperatureError]),
        "Interference":    (s.Interference,      ERROR_FLAG[s.Interference]),
        "PersistentError": (s.PersistentError,   ERROR_FLAG[s.PersistentError]),
    }
