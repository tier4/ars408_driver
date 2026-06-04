"""
Test script:
  1. CAN#201 初期値デコードの確認
  2. CAN#200 ID1/ID2 エンコード・ユーザ提供値との比較
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from radar_state import (
    parse_can201, state_to_dict,
    SORT_INDEX, RADAR_POWER, OUTPUT_TYPE, MOTION_RX, RCS_THRESH, STATUS, ERROR_FLAG,
)
from radar_cfg import (
    RadarCfg, encode_can200, decode_can200,
    RADAR_POWER_OPTS, OUTPUT_TYPE_OPTS, SORT_INDEX_OPTS,
)


def hex_str(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


def print_section(title: str):
    print(f"\n{'='*60}")
    print(f"  {title}")
    print('='*60)


# ============================================================
# TEST 1: #201 初期値デコード
# ============================================================
print_section("TEST 1: CAN#201 初期値デコード")
print("Input : 40 18 80 00 A0 F4 00 00")

raw_initial = bytes([0x40, 0x18, 0x80, 0x00, 0xA0, 0xF4, 0x00, 0x00])
state = parse_can201(raw_initial)

print(f"\n{'Signal':<22} {'Raw':>5}  {'Value'}")
print("-" * 50)
print(f"{'NVMReadStatus':<22} {state.NVMReadStatus:>5}  {STATUS.get(state.NVMReadStatus, '?')}")
print(f"{'NVMwriteStatus':<22} {state.NVMwriteStatus:>5}  {STATUS.get(state.NVMwriteStatus, '?')}")
print(f"{'MaxDistance':<22} {state.MaxDistance:>5}  {state.MaxDistance} m")
print(f"{'SensorID':<22} {state.SensorID:>5}  {state.SensorID}")
print(f"{'RadarPower':<22} {state.RadarPower:>5}  {RADAR_POWER.get(state.RadarPower, '?')}")
print(f"{'OutputType':<22} {state.OutputType:>5}  {OUTPUT_TYPE.get(state.OutputType, '?')}")
print(f"{'SendQuality':<22} {state.SendQuality:>5}  {'Active' if state.SendQuality else 'Inactive'}")
print(f"{'SendExtInfo':<22} {state.SendExtInfo:>5}  {'Active' if state.SendExtInfo else 'Inactive'}")
print(f"{'SortIndex':<22} {state.SortIndex:>5}  {SORT_INDEX.get(state.SortIndex, '?')}")
print(f"{'CtrlRelay':<22} {state.CtrlRelay:>5}  {'Active' if state.CtrlRelay else 'Inactive'}")
print(f"{'RCS_Threshold':<22} {state.RCS_Threshold:>5}  {RCS_THRESH.get(state.RCS_Threshold, '?')}")
print(f"{'MotionRxState':<22} {state.MotionRxState:>5}  {MOTION_RX.get(state.MotionRxState, '?')}")
print(f"{'VoltageError':<22} {state.VoltageError:>5}  {ERROR_FLAG[state.VoltageError]}")
print(f"{'TemporaryError':<22} {state.TemporaryError:>5}  {ERROR_FLAG[state.TemporaryError]}")
print(f"{'TemperatureError':<22} {state.TemperatureError:>5}  {ERROR_FLAG[state.TemperatureError]}")
print(f"{'Interference':<22} {state.Interference:>5}  {ERROR_FLAG[state.Interference]}")
print(f"{'PersistentError':<22} {state.PersistentError:>5}  {ERROR_FLAG[state.PersistentError]}")

if state.MaxDistance < 196:
    print(f"\n  ⚠  MaxDistance={state.MaxDistance}m は ARS408 最小値 196m より小さい")


# ============================================================
# TEST 2: #200 エンコード（ID1: SensorID=1）
# ============================================================
print_section("TEST 2: CAN#200 エンコード  ID1 (SensorID=1)")

cfg_id1 = RadarCfg(
    MaxDistance_valid=True, SensorID_valid=True, RadarPower_valid=True,
    OutputType_valid=True, SendQuality_valid=True, SendExtInfo_valid=True,
    SortIndex_valid=True, StoreInNVM_valid=True,
    CtrlRelay_valid=False, RCS_Threshold_valid=False,
    MaxDistance=148,   # user's specified value (note: below ARS408 min 196m)
    SensorID=1,
    RadarPower=1,      # -3dB
    OutputType=1,      # Objects
    SendQuality=True,
    SendExtInfo=True,
    SortIndex=1,       # By Range
    StoreInNVM=True,
    CtrlRelay=False,
    RCS_Threshold=0,
)

encoded_id1 = encode_can200(cfg_id1)
print(f"Correct encode : {hex_str(encoded_id1)}")
print(f"User provided  : FF 12 80 29 9C 00 00 00")

user_id1 = bytes([0xFF, 0x12, 0x80, 0x29, 0x9C, 0x00, 0x00, 0x00])
match_id1 = (encoded_id1 == user_id1)
print(f"Match          : {'✓ OK' if match_id1 else '✗ MISMATCH'}")

if not match_id1:
    print("\n  Byte-by-byte diff:")
    print(f"  {'Byte':<6} {'Correct':>10}  {'User':>6}  {'Status'}")
    for i, (c, u) in enumerate(zip(encoded_id1, user_id1)):
        flag = "✓" if c == u else "✗"
        print(f"  [{i}]    0x{c:02X} ({c:08b})  0x{u:02X}   {flag}")
    print()
    print("  → ユーザデータはバイト位置が1つずれています:")
    print(f"     正しい: {hex_str(encoded_id1)}")
    print(f"     ユーザ: {hex_str(user_id1)}")
    print("     byte[3] = 0x00 (未使用) が抜けています")


# ============================================================
# TEST 3: #200 エンコード（ID2: SensorID=2）
# ============================================================
print_section("TEST 3: CAN#200 エンコード  ID2 (SensorID=2)")

cfg_id2 = RadarCfg(
    MaxDistance_valid=True, SensorID_valid=True, RadarPower_valid=True,
    OutputType_valid=True, SendQuality_valid=True, SendExtInfo_valid=True,
    SortIndex_valid=True, StoreInNVM_valid=True,
    CtrlRelay_valid=False, RCS_Threshold_valid=False,
    MaxDistance=148,
    SensorID=2,        # ← SensorID=2
    RadarPower=1,
    OutputType=1,
    SendQuality=True,
    SendExtInfo=True,
    SortIndex=1,
    StoreInNVM=True,
    CtrlRelay=False,
    RCS_Threshold=0,
)

encoded_id2 = encode_can200(cfg_id2)
print(f"Correct encode : {hex_str(encoded_id2)}")
print(f"User provided  : FF 12 80 29 5C 00 00 00")

user_id2 = bytes([0xFF, 0x12, 0x80, 0x29, 0x5C, 0x00, 0x00, 0x00])
match_id2 = (encoded_id2 == user_id2)
print(f"Match          : {'✓ OK' if match_id2 else '✗ MISMATCH'}")

if not match_id2:
    print(f"\n  → 正しいID2: {hex_str(encoded_id2)}")
    print(f"     byte[4] の違い: 0x{encoded_id2[4]:02X} (SensorID=2+Objects+-3dB) vs"
          f" 0x{user_id2[4]:02X}")


# ============================================================
# TEST 4: エンコード→デコード往復チェック
# ============================================================
print_section("TEST 4: encode → decode 往復チェック (ID1)")

decoded = decode_can200(encoded_id1)
ok = (decoded.SensorID == 1 and decoded.RadarPower == 1 and
      decoded.OutputType == 1 and decoded.MaxDistance == 148 and
      decoded.SortIndex == 1 and decoded.SendQuality and decoded.SendExtInfo and
      decoded.StoreInNVM)
print(f"  SensorID={decoded.SensorID}  RadarPower={RADAR_POWER_OPTS[decoded.RadarPower]}"
      f"  OutputType={OUTPUT_TYPE_OPTS[decoded.OutputType]}")
print(f"  MaxDistance={decoded.MaxDistance}m  SortIndex={SORT_INDEX_OPTS[decoded.SortIndex]}")
print(f"  SendQuality={decoded.SendQuality}  SendExtInfo={decoded.SendExtInfo}"
      f"  StoreInNVM={decoded.StoreInNVM}")
print(f"  Result: {'✓ All OK' if ok else '✗ MISMATCH'}")


# ============================================================
# Summary
# ============================================================
print_section("Summary")
print(f"  TEST1 (#201 decode)        : ✓ Decoded successfully")
print(f"  TEST2 (#200 ID1 encode)    : {'✓ OK' if match_id1 else '✗ User data has 1-byte offset error'}")
print(f"  TEST3 (#200 ID2 encode)    : {'✓ OK' if match_id2 else '✗ User data has 1-byte offset error'}")
print(f"  TEST4 (encode→decode roundtrip): {'✓ OK' if ok else '✗ FAIL'}")

if not match_id1 or not match_id2:
    print()
    print("  ⚠  ユーザ提供の入力値について:")
    print(f"     正しい ID1 (SensorID=1): {hex_str(encoded_id1)}")
    print(f"     正しい ID2 (SensorID=2): {hex_str(encoded_id2)}")
    print("     byte[3] = 0x00 (Row3は#200で未使用) を追加してください")
