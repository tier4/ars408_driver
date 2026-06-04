"""
CAN#200 (RadarCfg) encoder/decoder for ARS408.
Porting logic from ars408_driver.cpp GenerateRadarConfiguration().
"""

from dataclasses import dataclass, field

# Value labels (same as user config table)
SORT_INDEX_OPTS  = {0: "No Sort", 1: "By Range", 2: "By RCS"}
RADAR_POWER_OPTS = {0: "Standard", 1: "-3dB", 2: "-6dB", 3: "-9dB"}
OUTPUT_TYPE_OPTS = {0: "None", 1: "Objects", 2: "Clusters"}
RCS_THRESH_OPTS  = {0: "Standard", 1: "High Sensitivity"}

# Mapping: #200 field name → #201 field name (for comparison)
CFG_TO_STATE_MAP = {
    "MaxDistance":  "MaxDistance",
    "SensorID":     "SensorID",
    "RadarPower":   "RadarPower",
    "OutputType":   "OutputType",
    "SendQuality":  "SendQuality",
    "SendExtInfo":  "SendExtInfo",
    "SortIndex":    "SortIndex",
    "CtrlRelay":    "CtrlRelay",
    "RCS_Threshold":"RCS_Threshold",
}


@dataclass
class RadarCfg:
    # --- valid flags (byte 0) ---
    MaxDistance_valid:  bool = False
    SensorID_valid:     bool = False
    RadarPower_valid:   bool = False
    OutputType_valid:   bool = False
    SendQuality_valid:  bool = False
    SendExtInfo_valid:  bool = False
    SortIndex_valid:    bool = False
    StoreInNVM_valid:   bool = False
    # --- valid flags (byte 5/6, outside byte 0) ---
    CtrlRelay_valid:    bool = False
    RCS_Threshold_valid:bool = False

    # --- values ---
    MaxDistance:   int  = 196   # m (ARS408 min 196m, max 260m standard)
    SensorID:      int  = 0     # 0-7
    RadarPower:    int  = 1     # 0=Standard, 1=-3dB, 2=-6dB, 3=-9dB
    OutputType:    int  = 1     # 0=None, 1=Objects, 2=Clusters
    SendQuality:   bool = True
    SendExtInfo:   bool = True
    SortIndex:     int  = 1     # 0=No Sort, 1=By Range, 2=By RCS
    CtrlRelay:     bool = False
    StoreInNVM:    bool = True
    RCS_Threshold: int  = 0     # 0=Standard, 1=High Sensitivity


def encode_can200(cfg: RadarCfg) -> bytes:
    """
    Encode CAN#200 (RadarCfg) 8-byte message.
    Logic ported from ars408_driver.cpp GenerateRadarConfiguration().

    Byte layout:
      [0] valid flags (bits 0-7)
      [1] MaxDistance high  (bits 9:2 of raw10)
      [2] MaxDistance low   (bits 1:0 of raw10 → top 2 bits of byte2)
      [3] 0x00              (unused in #200)
      [4] SensorID[2:0] | OutputType[4:3] | RadarPower[7:5]
      [5] CtrlRelay_valid[0] | CtrlRelay[1] | SendQuality[2] |
          SendExtInfo[3] | SortIndex[5:4] | StoreInNVM[7]
      [6] RCS_Threshold_valid[0] | RCS_Threshold[3:1]
      [7] 0x00
    """
    data = bytearray(8)

    # --- Byte 0: valid flags ---
    if cfg.MaxDistance_valid:  data[0] |= 0x01
    if cfg.SensorID_valid:     data[0] |= 0x02
    if cfg.RadarPower_valid:   data[0] |= 0x04
    if cfg.OutputType_valid:   data[0] |= 0x08
    if cfg.SendQuality_valid:  data[0] |= 0x10
    if cfg.SendExtInfo_valid:  data[0] |= 0x20
    if cfg.SortIndex_valid:    data[0] |= 0x40
    if cfg.StoreInNVM_valid:   data[0] |= 0x80

    # --- Bytes 1-2: MaxDistance (res=2m → raw10 = value // 2) ---
    if cfg.MaxDistance_valid:
        raw10 = (cfg.MaxDistance // 2) & 0x3FF
        data[1] = (raw10 >> 2) & 0xFF
        data[2] = ((raw10 & 0x03) << 6) & 0xFF

    # --- Byte 4: SensorID | OutputType | RadarPower ---
    if cfg.SensorID_valid:
        data[4] |= cfg.SensorID & 0x07          # bits 2:0
    if cfg.OutputType_valid:
        ot = cfg.OutputType & 0x03
        data[4] |= (ot << 3) & 0x18             # bits 4:3
    if cfg.RadarPower_valid:
        rp = cfg.RadarPower & 0x07
        data[4] |= (rp << 5) & 0xE0             # bits 7:5

    # --- Byte 5: CtrlRelay_valid/CtrlRelay/SendQuality/SendExtInfo/SortIndex/StoreInNVM ---
    if cfg.CtrlRelay_valid:                data[5] |= 0x01  # bit 0 (bit 40)
    if cfg.CtrlRelay:                      data[5] |= 0x02  # bit 1 (bit 41)
    if cfg.SendQuality_valid and cfg.SendQuality:  data[5] |= 0x04  # bit 2 (bit 42)
    if cfg.SendExtInfo_valid and cfg.SendExtInfo:  data[5] |= 0x08  # bit 3 (bit 43)
    if cfg.SortIndex_valid:
        si = cfg.SortIndex
        if si == 1:   data[5] |= 0x10           # bit 4 (bit 44) BY_RANGE
        elif si == 2: data[5] |= 0x20           # bit 5 (bit 45) BY_RCS
    if cfg.StoreInNVM_valid and cfg.StoreInNVM: data[5] |= 0x80  # bit 7 (bit 47)

    # --- Byte 6: RCS_Threshold_valid / RCS_Threshold ---
    if cfg.RCS_Threshold_valid:
        data[6] |= 0x01                         # bit 0 (bit 48)
        data[6] |= (cfg.RCS_Threshold & 0x07) << 1  # bits 3:1 (bits 51:49)

    return bytes(data)


def decode_can200(data: bytes) -> RadarCfg:
    """Decode CAN#200 bytes back to RadarCfg (for verification)."""
    if len(data) < 8:
        raise ValueError(f"Expected 8 bytes, got {len(data)}")

    cfg = RadarCfg()

    cfg.MaxDistance_valid  = bool(data[0] & 0x01)
    cfg.SensorID_valid     = bool(data[0] & 0x02)
    cfg.RadarPower_valid   = bool(data[0] & 0x04)
    cfg.OutputType_valid   = bool(data[0] & 0x08)
    cfg.SendQuality_valid  = bool(data[0] & 0x10)
    cfg.SendExtInfo_valid  = bool(data[0] & 0x20)
    cfg.SortIndex_valid    = bool(data[0] & 0x40)
    cfg.StoreInNVM_valid   = bool(data[0] & 0x80)

    if cfg.MaxDistance_valid:
        raw10 = ((data[1] << 2) & 0x3FF) | ((data[2] & 0xC0) >> 6)
        cfg.MaxDistance = raw10 * 2

    cfg.SensorID    =  data[4] & 0x07
    cfg.OutputType  = (data[4] & 0x18) >> 3
    cfg.RadarPower  = (data[4] & 0xE0) >> 5

    cfg.CtrlRelay_valid = bool(data[5] & 0x01)
    cfg.CtrlRelay       = bool(data[5] & 0x02)
    cfg.SendQuality     = bool(data[5] & 0x04)
    cfg.SendExtInfo     = bool(data[5] & 0x08)
    si = (data[5] & 0x30) >> 4
    cfg.SortIndex       = si
    cfg.StoreInNVM      = bool(data[5] & 0x80)

    cfg.RCS_Threshold_valid = bool(data[6] & 0x01)
    cfg.RCS_Threshold       = (data[6] & 0x0E) >> 1

    return cfg


def cfg_to_dict(cfg: RadarCfg) -> dict:
    """Convert RadarCfg to labeled dict for display/save."""
    return {
        "MaxDistance_valid":   cfg.MaxDistance_valid,
        "SensorID_valid":      cfg.SensorID_valid,
        "RadarPower_valid":    cfg.RadarPower_valid,
        "OutputType_valid":    cfg.OutputType_valid,
        "SendQuality_valid":   cfg.SendQuality_valid,
        "SendExtInfo_valid":   cfg.SendExtInfo_valid,
        "SortIndex_valid":     cfg.SortIndex_valid,
        "StoreInNVM_valid":    cfg.StoreInNVM_valid,
        "CtrlRelay_valid":     cfg.CtrlRelay_valid,
        "RCS_Threshold_valid": cfg.RCS_Threshold_valid,
        "MaxDistance":         cfg.MaxDistance,
        "SensorID":            cfg.SensorID,
        "RadarPower":          cfg.RadarPower,
        "OutputType":          cfg.OutputType,
        "SendQuality":         cfg.SendQuality,
        "SendExtInfo":         cfg.SendExtInfo,
        "SortIndex":           cfg.SortIndex,
        "CtrlRelay":           cfg.CtrlRelay,
        "StoreInNVM":          cfg.StoreInNVM,
        "RCS_Threshold":       cfg.RCS_Threshold,
    }


def dict_to_cfg(d: dict) -> RadarCfg:
    cfg = RadarCfg()
    for k, v in d.items():
        if hasattr(cfg, k):
            setattr(cfg, k, v)
    return cfg
