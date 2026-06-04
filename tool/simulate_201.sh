#!/bin/bash
# ARS408 Radar Simulator ラッパー
#
# CAN#200 を受信し、設定を反映した CAN#201 を自動送信します。
# radar_simulator.py を呼び出すラッパーです。
#
# 使い方:
#   bash simulate_201.sh                  # vcan0, Sensor ID=0
#   bash simulate_201.sh vcan0            # インターフェース指定
#   bash simulate_201.sh vcan0 1          # インターフェース + Sensor ID 指定

IFACE="${1:-vcan0}"
SENSOR_ID="${2:-0}"
INTERVAL="0.5"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

if ! ip link show "$IFACE" &>/dev/null; then
    echo "エラー: $IFACE が見つかりません。先に setup_vcan.sh を実行してください。"
    exit 1
fi

echo "=== ARS408 Radar Simulator ==="
echo "  interface : $IFACE"
echo "  sensor_id : $SENSOR_ID"
echo "  interval  : ${INTERVAL}s  (#201 定期送信間隔)"
echo "  動作: CAN#200 受信 → 設定反映 → CAN#201 自動送信"
echo "  停止: Ctrl+C"
echo ""

python3 "${SCRIPT_DIR}/src/radar_simulator.py" \
    --channel "$IFACE" \
    --interval "$INTERVAL" \
    --sensor-id "$SENSOR_ID"
