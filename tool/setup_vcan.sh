#!/bin/bash
# vcan0 セットアップスクリプト
# 実行: bash setup_vcan.sh

set -e

echo "=== vcan0 セットアップ ==="

echo "[1/3] vcan カーネルモジュールをロード..."
sudo modprobe vcan

echo "[2/3] vcan0 インターフェースを作成..."
if ip link show vcan0 &>/dev/null; then
    echo "  vcan0 は既に存在します。スキップ。"
else
    sudo ip link add dev vcan0 type vcan
fi

echo "[3/3] vcan0 を起動..."
sudo ip link set up vcan0

echo ""
echo "=== 完了 ==="
ip link show vcan0
echo ""
echo "次のステップ:"
echo "  ターミナル1: candump vcan0          (受信監視)"
echo "  ターミナル2: python3 main.py        (GUIを起動)"
echo "  ターミナル3: bash simulate_201.sh   (#201シミュレート送信)"
