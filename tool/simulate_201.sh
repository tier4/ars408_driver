#!/bin/bash
# CAN#201 シミュレート送信スクリプト
# GUIを vcan0 に接続した状態で実行する
#
# 使い方:
#   bash simulate_201.sh          # 初期値を1回送信
#   bash simulate_201.sh loop     # 1秒おきに繰り返し送信

IFACE="vcan0"

# ================================================================
# #201 テストデータ
# ================================================================

# 初期値: MaxDist=196m, SensorID=0, RadarPower=-3dB, OutputType=Objects,
#         SendQuality=Active, SendExtInfo=Active, SortIndex=ByRCS
# バイト構成: [0x40][0x18][0x80][0x00][0xA0][0xF4][0x00][0x00]
INITIAL="40188000A0F40000"

# SensorID=1 に変化した場合 (byte4 bit0: 0xA0→0xA1)
SENSOR_ID_1="40188000A1F40000"

# SensorID=2 に変化した場合 (byte4 bit1: 0xA0→0xA2)
SENSOR_ID_2="40188000A2F40000"

# OutputType=Clusters に変化した場合 (byte5 bits3:2=10: 0xF4→0xF8)
OUTPUT_CLUSTERS="40188000A0F80000"

# MaxDist=148m (raw=74): byte1=0x12, byte2=0x80
MAXDIST_148="40128000A0F40000"

echo "=== CAN#201 シミュレート送信 (interface: $IFACE) ==="
echo ""

if ! ip link show "$IFACE" &>/dev/null; then
    echo "エラー: $IFACE が見つかりません。先に setup_vcan.sh を実行してください。"
    exit 1
fi

send_frame() {
    local label="$1"
    local data="$2"
    echo "送信: $label"
    echo "  201#${data}"
    cansend "$IFACE" "201#${data}"
}

if [ "$1" = "loop" ]; then
    echo "ループモード (Ctrl+C で停止)"
    echo ""
    while true; do
        send_frame "初期値 (MaxDist=196m, SensorID=0)" "$INITIAL"
        sleep 2
    done
else
    echo "--- 初期値 ---"
    send_frame "初期値 (MaxDist=196m, SensorID=0, RadarPower=-3dB)" "$INITIAL"
    sleep 1

    echo ""
    echo "--- SensorID 変化テスト ---"
    send_frame "SensorID=1" "$SENSOR_ID_1"
    sleep 1
    send_frame "SensorID=2" "$SENSOR_ID_2"
    sleep 1

    echo ""
    echo "--- OutputType 変化テスト ---"
    send_frame "OutputType=Clusters" "$OUTPUT_CLUSTERS"
    sleep 1
    send_frame "初期値に戻す (OutputType=Objects)" "$INITIAL"

    echo ""
    echo "完了。GUIの比較表示を確認してください。"
fi
