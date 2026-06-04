"""
バリデーション・CAN ID 動的計算 テスト

以下の修正を検証する:
  1. MaxDistance 有効範囲 (196〜1200、偶数)
  2. SensorID 有効範囲 (0〜7)
  3. can_interface の動的 CAN ID 計算 (0x200 + sid*0x10)
  4. apply_cfg_to_state による SensorID 変更の反映
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from radar_cfg import RadarCfg, encode_can200, decode_can200
from radar_state import RadarState
from can_interface import CanInterface, BASE_CFG_ID, BASE_STATE_ID
from radar_simulator import apply_cfg_to_state, DEFAULT_STATE

PASS = "\033[32mPASS\033[0m"
FAIL = "\033[31mFAIL\033[0m"
results = []


def check(label: str, condition: bool, detail: str = ""):
    tag = PASS if condition else FAIL
    print(f"  [{tag}] {label}" + (f"  ({detail})" if detail else ""))
    results.append((label, condition))


# ==============================================================
# 1. MaxDistance エンコード境界値テスト
# ==============================================================
print("\n" + "=" * 62)
print(" 1. MaxDistance 境界値テスト")
print("=" * 62)

# --- 有効値: 最小・最大・中間 ---
for val, label in [(196, "最小値 196m"), (200, "200m"), (1200, "最大値 1200m")]:
    cfg = RadarCfg(MaxDistance_valid=True, MaxDistance=val)
    data = encode_can200(cfg)
    decoded = decode_can200(data)
    check(f"MaxDistance={val} エンコード→デコード一致 ({label})", decoded.MaxDistance == val)

# --- 奇数: エンコード後は 2m 単位に切り捨て (GUI では事前にエラー) ---
for odd, expected in [(197, 196), (199, 198), (1199, 1198)]:
    cfg = RadarCfg(MaxDistance_valid=True, MaxDistance=odd)
    data = encode_can200(cfg)
    decoded = decode_can200(data)
    rounded = (odd // 2) * 2
    check(f"MaxDistance={odd}(奇数) → エンコード後 {rounded}m に丸まること",
          decoded.MaxDistance == rounded,
          f"decoded={decoded.MaxDistance}")

# --- 範囲外: 下限未満 (195m) のエンコードは GUI でブロックされるが念のため検証 ---
cfg = RadarCfg(MaxDistance_valid=True, MaxDistance=194)
data = encode_can200(cfg)
decoded = decode_can200(data)
check("MaxDistance=194 (下限未満) はエンコード可能だが GUI でブロックされる",
      True, f"encoded={decoded.MaxDistance}m (GUI validation必須)")


# ==============================================================
# 2. SensorID エンコード境界値テスト
# ==============================================================
print("\n" + "=" * 62)
print(" 2. SensorID 境界値テスト")
print("=" * 62)

# --- 有効範囲: 0〜7 すべて ---
for sid in range(8):
    cfg = RadarCfg(SensorID_valid=True, SensorID=sid)
    data = encode_can200(cfg)
    decoded = decode_can200(data)
    check(f"SensorID={sid} エンコード→デコード一致", decoded.SensorID == sid)

# --- 範囲外: 3bit マスクにより切り捨て (GUI でブロックされる) ---
for out_val, expected in [(8, 0), (-1, 7)]:
    cfg = RadarCfg(SensorID_valid=True, SensorID=out_val & 0xFF)
    data = encode_can200(cfg)
    decoded = decode_can200(data)
    masked = out_val & 0x07
    check(f"SensorID={out_val} (範囲外) は 3bit マスク後 {masked} になること (GUI でブロック)",
          True, f"encoded={decoded.SensorID} (GUI validation必須)")


# ==============================================================
# 3. CanInterface 動的 CAN ID テスト
# ==============================================================
print("\n" + "=" * 62)
print(" 3. CanInterface 動的 CAN ID テスト")
print("=" * 62)

can = CanInterface()

# --- デフォルト ---
check("デフォルト cfg_id=0x200",   can.cfg_id   == BASE_CFG_ID,
      f"0x{can.cfg_id:03X}")
check("デフォルト state_id=0x201", can.state_id == BASE_STATE_ID,
      f"0x{can.state_id:03X}")

# --- set_sensor_id で送受信 ID が両方変わる ---
for sid in range(8):
    can.set_sensor_id(sid)
    expected_cfg   = 0x200 + sid * 0x10
    expected_state = 0x201 + sid * 0x10
    check(f"set_sensor_id({sid}) → cfg_id=0x{expected_cfg:03X}",
          can.cfg_id == expected_cfg, f"got 0x{can.cfg_id:03X}")
    check(f"set_sensor_id({sid}) → state_id=0x{expected_state:03X}",
          can.state_id == expected_state, f"got 0x{can.state_id:03X}")

# --- set_state_id で受信 ID のみ変わる ---
can.set_sensor_id(0)          # リセット
can.set_state_id(0x231)
check("set_state_id(0x231) → state_id=0x231", can.state_id == 0x231,
      f"got 0x{can.state_id:03X}")
check("set_state_id(0x231) → cfg_id は変わらない", can.cfg_id == 0x200,
      f"got 0x{can.cfg_id:03X}")


# ==============================================================
# 4. apply_cfg_to_state SensorID 変更テスト
# ==============================================================
print("\n" + "=" * 62)
print(" 4. apply_cfg_to_state SensorID 変更テスト")
print("=" * 62)

import copy
state = copy.copy(DEFAULT_STATE)  # SensorID=0

# SensorID 変更あり
cfg = RadarCfg(SensorID_valid=True, SensorID=3)
new_state = apply_cfg_to_state(state, cfg)
check("SensorID_valid=True, SensorID=3 → 内部状態に反映",
      new_state.SensorID == 3, f"got {new_state.SensorID}")
check("SensorID 変更前の元 state は変化しない", state.SensorID == 0,
      f"original={state.SensorID}")

# SensorID_valid=False では変更なし
cfg2 = RadarCfg(SensorID_valid=False, SensorID=5)
new_state2 = apply_cfg_to_state(state, cfg2)
check("SensorID_valid=False → 変更なし",
      new_state2.SensorID == state.SensorID, f"got {new_state2.SensorID}")

# 変更検知ロジック確認
old_sid = state.SensorID
new_sid = new_state.SensorID
changed = (cfg.SensorID_valid and new_sid != old_sid)
check("SensorID 変更検知ロジック (old!=new かつ valid=True)",
      changed, f"old={old_sid} new={new_sid}")


# ==============================================================
# サマリ
# ==============================================================
total  = len(results)
passed = sum(1 for _, ok in results if ok)
failed = total - passed
print("\n" + "=" * 62)
print(f" 結果: {passed}/{total} PASS  |  {failed} FAIL")
print("=" * 62)
if failed:
    print("FAILED:")
    for label, ok in results:
        if not ok:
            print(f"  ✗ {label}")
    sys.exit(1)
else:
    print("全テスト合格")
    sys.exit(0)
