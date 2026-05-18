"""
各設定フィールドの送受信テスト

各フィールドについて:
  1. RadarCfg に設定値を入れ CAN#200 をエンコード
  2. 実際のレーダーが返すべき CAN#201 バイト列を手動構築
  3. #201 をデコードして送信値と一致するか検証

#200 と #201 はビット配置が異なるため、対応表を元に手動で #201 を構築する。
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from radar_cfg import RadarCfg, encode_can200, decode_can200
from radar_state import parse_can201, SORT_INDEX, RADAR_POWER, OUTPUT_TYPE, RCS_THRESH

PASS = "\033[32mPASS\033[0m"
FAIL = "\033[31mFAIL\033[0m"

results = []


def check(label: str, sent_val, recv_val, note: str = ""):
    ok = (sent_val == recv_val)
    tag = PASS if ok else FAIL
    print(f"  [{tag}] {label}: sent={sent_val}  received={recv_val}  {note}")
    results.append((label, ok))


def hex_bytes(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)


# ============================================================
# ベースとなる #201 初期状態（実機確認済み）
#   40 18 80 00 A0 F4 00 00
#   → MaxDistance=196m, SensorID=0, RadarPower=1(-3dB),
#     SortIndex=2(By RCS), OutputType=1(Objects),
#     SendQuality=1(Active), SendExtInfo=1(Active),
#     CtrlRelay=0(Inactive), RCS_Threshold=0(Standard)
# ============================================================
BASE_201 = bytearray(bytes.fromhex("4018800 0A0F40000".replace(" ", "")))

def make_201(**kwargs) -> bytes:
    """
    初期状態の #201 バイト列を元に、指定フィールドを上書きした #201 を生成する。

    #201 各フィールドのビット位置（ars408_driver.cpp ParseRadarState より）:
      MaxDistance   : byte1 = raw10[9:2], byte2[7:6] = raw10[1:0]  (res=2m)
      SensorID      : byte4 bits[2:0]
      RadarPower    : (byte3<<1) | (byte4 bit7)    (値 0–3 の 3bit)
      SortIndex     : byte4 bits[6:4]
      OutputType    : byte5 bits[3:2]
      SendQuality   : byte5 bit4
      SendExtInfo   : byte5 bit5
      CtrlRelay     : byte5 bit1
      RCS_Threshold : byte7 bits[4:2]
    """
    d = bytearray(bytes.fromhex("401880 00A0F40000".replace(" ", "")))

    if "MaxDistance" in kwargs:
        v = kwargs["MaxDistance"]
        raw10 = (v // 2) & 0x3FF
        d[1] = (raw10 >> 2) & 0xFF
        d[2] = (d[2] & 0x3F) | (((raw10 & 0x03) << 6) & 0xC0)

    if "SensorID" in kwargs:
        d[4] = (d[4] & ~0x07) | (kwargs["SensorID"] & 0x07)

    if "RadarPower" in kwargs:
        rp = kwargs["RadarPower"] & 0x07
        # byte3 = rp >> 1, byte4 bit7 = rp & 1
        d[3] = (rp >> 1) & 0xFF
        d[4] = (d[4] & ~0x80) | ((rp & 0x01) << 7)

    if "SortIndex" in kwargs:
        si = kwargs["SortIndex"] & 0x07
        d[4] = (d[4] & ~0x70) | ((si << 4) & 0x70)

    if "OutputType" in kwargs:
        ot = kwargs["OutputType"] & 0x03
        d[5] = (d[5] & ~0x0C) | ((ot << 2) & 0x0C)

    if "SendQuality" in kwargs:
        if kwargs["SendQuality"]:
            d[5] |= 0x10
        else:
            d[5] &= ~0x10

    if "SendExtInfo" in kwargs:
        if kwargs["SendExtInfo"]:
            d[5] |= 0x20
        else:
            d[5] &= ~0x20

    if "CtrlRelay" in kwargs:
        if kwargs["CtrlRelay"]:
            d[5] |= 0x02
        else:
            d[5] &= ~0x02

    if "RCS_Threshold" in kwargs:
        rt = kwargs["RCS_Threshold"] & 0x07
        d[7] = (d[7] & ~0x1C) | ((rt << 2) & 0x1C)

    return bytes(d)


# ============================================================
# テスト定義
# (label, RadarCfg設定, 期待#201フィールド値dict, #200バイト期待値 or None)
# ============================================================
test_cases = [
    # ---- MaxDistance ----
    dict(
        label    = "MaxDistance = 200 m",
        cfg_kw   = dict(MaxDistance_valid=True, MaxDistance=200),
        state_kw = dict(MaxDistance=200),
        check_field = "MaxDistance",
    ),
    dict(
        label    = "MaxDistance = 260 m",
        cfg_kw   = dict(MaxDistance_valid=True, MaxDistance=260),
        state_kw = dict(MaxDistance=260),
        check_field = "MaxDistance",
    ),
    # ---- SensorID ----
    dict(
        label    = "SensorID = 1",
        cfg_kw   = dict(SensorID_valid=True, SensorID=1),
        state_kw = dict(SensorID=1),
        check_field = "SensorID",
    ),
    dict(
        label    = "SensorID = 7 (max)",
        cfg_kw   = dict(SensorID_valid=True, SensorID=7),
        state_kw = dict(SensorID=7),
        check_field = "SensorID",
    ),
    # ---- RadarPower ----
    dict(
        label    = "RadarPower = 0 (Standard)",
        cfg_kw   = dict(RadarPower_valid=True, RadarPower=0),
        state_kw = dict(RadarPower=0),
        check_field = "RadarPower",
    ),
    dict(
        label    = "RadarPower = 2 (-6dB)",
        cfg_kw   = dict(RadarPower_valid=True, RadarPower=2),
        state_kw = dict(RadarPower=2),
        check_field = "RadarPower",
    ),
    dict(
        label    = "RadarPower = 3 (-9dB)",
        cfg_kw   = dict(RadarPower_valid=True, RadarPower=3),
        state_kw = dict(RadarPower=3),
        check_field = "RadarPower",
    ),
    # ---- OutputType ----
    dict(
        label    = "OutputType = 0 (None)",
        cfg_kw   = dict(OutputType_valid=True, OutputType=0),
        state_kw = dict(OutputType=0),
        check_field = "OutputType",
    ),
    dict(
        label    = "OutputType = 2 (Clusters)",
        cfg_kw   = dict(OutputType_valid=True, OutputType=2),
        state_kw = dict(OutputType=2),
        check_field = "OutputType",
    ),
    # ---- SendQuality ----
    dict(
        label    = "SendQuality = True (Active)",
        cfg_kw   = dict(SendQuality_valid=True, SendQuality=True),
        state_kw = dict(SendQuality=True),
        check_field = "SendQuality",
    ),
    dict(
        label    = "SendQuality = False (Inactive)",
        cfg_kw   = dict(SendQuality_valid=True, SendQuality=False),
        state_kw = dict(SendQuality=False),
        check_field = "SendQuality",
    ),
    # ---- SendExtInfo ----
    dict(
        label    = "SendExtInfo = True (Active)",
        cfg_kw   = dict(SendExtInfo_valid=True, SendExtInfo=True),
        state_kw = dict(SendExtInfo=True),
        check_field = "SendExtInfo",
    ),
    dict(
        label    = "SendExtInfo = False (Inactive)",
        cfg_kw   = dict(SendExtInfo_valid=True, SendExtInfo=False),
        state_kw = dict(SendExtInfo=False),
        check_field = "SendExtInfo",
    ),
    # ---- SortIndex ----
    dict(
        label    = "SortIndex = 0 (No Sort)",
        cfg_kw   = dict(SortIndex_valid=True, SortIndex=0),
        state_kw = dict(SortIndex=0),
        check_field = "SortIndex",
    ),
    dict(
        label    = "SortIndex = 1 (By Range)",
        cfg_kw   = dict(SortIndex_valid=True, SortIndex=1),
        state_kw = dict(SortIndex=1),
        check_field = "SortIndex",
    ),
    dict(
        label    = "SortIndex = 2 (By RCS)",
        cfg_kw   = dict(SortIndex_valid=True, SortIndex=2),
        state_kw = dict(SortIndex=2),
        check_field = "SortIndex",
    ),
    # ---- CtrlRelay ----
    dict(
        label    = "CtrlRelay = False (Inactive)",
        cfg_kw   = dict(CtrlRelay_valid=True, CtrlRelay=False),
        state_kw = dict(CtrlRelay=False),
        check_field = "CtrlRelay",
    ),
    dict(
        label    = "CtrlRelay = True (Active)",
        cfg_kw   = dict(CtrlRelay_valid=True, CtrlRelay=True),
        state_kw = dict(CtrlRelay=True),
        check_field = "CtrlRelay",
    ),
    # ---- RCS_Threshold ----
    dict(
        label    = "RCS_Threshold = 0 (Standard)",
        cfg_kw   = dict(RCS_Threshold_valid=True, RCS_Threshold=0),
        state_kw = dict(RCS_Threshold=0),
        check_field = "RCS_Threshold",
    ),
    dict(
        label    = "RCS_Threshold = 1 (High Sensitivity)",
        cfg_kw   = dict(RCS_Threshold_valid=True, RCS_Threshold=1),
        state_kw = dict(RCS_Threshold=1),
        check_field = "RCS_Threshold",
    ),
]

# ============================================================
# 実行
# ============================================================
print("=" * 65)
print(" ARS408 フィールド別 送受信テスト")
print("=" * 65)

for tc in test_cases:
    label       = tc["label"]
    cfg_kw      = tc["cfg_kw"]
    state_kw    = tc["state_kw"]
    check_field = tc["check_field"]

    print(f"\n▶ {label}")

    # 1) #200 エンコード
    cfg = RadarCfg(**cfg_kw)
    raw200 = encode_can200(cfg)
    print(f"  #200 hex : {hex_bytes(raw200)}")

    # 2) #200 デコード→ラウンドトリップ確認
    cfg_rt = decode_can200(raw200)
    sent_val = getattr(cfg, check_field)
    rt_val   = getattr(cfg_rt, check_field)
    check(f"#200 encode→decode ({check_field})", sent_val, rt_val,
          note=f"({type(sent_val).__name__})")

    # 3) 対応する #201 を手動構築してデコード
    raw201 = make_201(**state_kw)
    print(f"  #201 hex : {hex_bytes(raw201)}")
    state = parse_can201(raw201)
    recv_val = getattr(state, check_field)

    # bool/int 統一
    if isinstance(sent_val, bool):
        recv_val_cmp = bool(recv_val)
    else:
        recv_val_cmp = recv_val

    check(f"#200 sent == #201 decoded ({check_field})", sent_val, recv_val_cmp)


# ============================================================
# 追加: valid=False のフィールドは #200 に値が載らないか
# ============================================================
print("\n" + "=" * 65)
print(" valid=False 確認テスト（値がバイト列に反映されないこと）")
print("=" * 65)

for field_name, val in [("MaxDistance", 260), ("SensorID", 5),
                         ("RadarPower", 3), ("OutputType", 2),
                         ("SortIndex", 2), ("RCS_Threshold", 1)]:
    cfg = RadarCfg()
    setattr(cfg, f"{field_name}_valid", False)
    setattr(cfg, field_name, val)
    raw = encode_can200(cfg)
    # valid=False なのですべて 0x00 のはず
    all_zero = all(b == 0 for b in raw)
    tag = PASS if all_zero else FAIL
    print(f"  [{tag}] {field_name}_valid=False → #200 hex: {hex_bytes(raw)}  (all 0x00: {all_zero})")
    results.append((f"{field_name}_valid=False", all_zero))


# ============================================================
# サマリ
# ============================================================
total  = len(results)
passed = sum(1 for _, ok in results if ok)
failed = total - passed
print("\n" + "=" * 65)
print(f" 結果: {passed}/{total} PASS  |  {failed} FAIL")
print("=" * 65)
if failed:
    print("FAILED :")
    for label, ok in results:
        if not ok:
            print(f"  ✗ {label}")
    sys.exit(1)
else:
    print("全テスト合格")
    sys.exit(0)
