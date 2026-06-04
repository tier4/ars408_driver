# ARS408 Radar Config Tool

Continental ARS408 レーダーの設定を CAN バス経由で変更し、正しく反映されたかを確認するための GUI ツールです。

---

## ファイル構成

```text
tools/
├── src/
│   ├── main.py               # GUI（設定送信・比較表示）
│   ├── radar_simulator.py    # レーダー本体の代役シミュレーター
│   ├── radar_cfg.py          # CAN#200 (RadarCfg) エンコード・デコード
│   ├── radar_state.py        # CAN#201 (RadarState) デコード
│   └── can_interface.py      # python-can 通信ラッパー
├── test/
│   ├── test_decode.py        # デコードの単体テスト
│   └── test_fields.py        # 各フィールドの送受信テスト
├── decode_201_gui.py         # CAN#201 hex → ビットレイアウト確認 GUI
├── setup_vcan.sh             # vcan0 仮想 CAN インターフェース セットアップ
├── simulate_201.sh           # #201 シミュレート送信スクリプト
├── settings.json             # 前回の設定を保存（自動生成）
└── README.md
```

---

## 使い方

### 1. 事前準備

```bash
# python-can インストール
pip3 install python-can

# vcan0 セットアップ（実機がない場合）
bash setup_vcan.sh
```

### 2. 起動手順

**ターミナル 1: シミュレーター起動（実機がない場合）**

```bash
python3 src/radar_simulator.py --channel vcan0
```

**ターミナル 2: GUI 起動**

```bash
python3 src/main.py
```

### 3. GUI の操作手順

1. **Connect** ボタンで vcan0（または実機の CAN インターフェース）に接続
2. 変更したい設定の `_valid` チェックボックスをオンにする
3. 値を入力する
4. **Send #200** ボタンで設定を送信
5. レーダー再起動後（実機）または自動（シミュレーター）で #201 を受信
6. 比較表で ✓ / ✗ を確認する

---

## CAN#200 送信バイト列の生成場所

`main.py` の **Send Config パネル**（左側）最下部に、設定値から生成された CAN#200 の hex バイト列がリアルタイムで表示されます。

```text
┌─────────────────────────────────────────────┐
│ Send Config  (CAN 0x200)                    │
│                                             │
│  [チェックボックス・値の入力欄 ...]           │
│                                             │
│ CAN 0x200 hex (cansend用):                  │
│ FF 09 C0 00 29 9E 00 00                     │  ← ここ
│   cansend: can0 200#FF09C000299E0000        │  ← ここ
└─────────────────────────────────────────────┘
```

- チェックボックスや値を変えるたびに**自動更新**されます（Send ボタンを押す前に確認可能）。
- `cansend: <インターフェース> <CAN_ID>#<データ>` の形式で表示されるため、そのままターミナルに貼り付けて送信できます。
- **Send ボタン**を押すと、同じ hex バイト列がポップアップにも表示されます。

生成ロジックは `src/radar_cfg.py` の `encode_can200()` 関数です。

---

## decode_201_gui.py — CAN#201 ビットレイアウト確認 GUI

`candump` 等で取得した CAN#201 の hex バイト列を入力すると、データシートと同形式のビットレイアウト表で各フィールドを確認できます。

```bash
python3 decode_201_gui.py
```

| エリア | 内容 |
|--------|------|
| 上部の入力欄 | hex バイト列を貼り付けて **Decode** を押す |
| Bit Layout 表 | 定義フィールドのビット = 白、未使用ビット = 灰。Decode 後は各セルにビット値（0/1）も表示 |
| Decoded Fields | 各フィールドの解釈値を一覧表示。エラーフラグが立っている場合は赤背景 |

**入力フォーマット**（いずれも対応）

```bash
40 18 80 00 A0 F4 00 00        # スペース区切り
401880 00A0F40000              # スペースなし
201#401880 00A0F40000          # candump 形式
```

---

## 実機での計測手順

### 前提条件

| 項目 | 内容 |
|---|---|
| CAN インターフェース | USB-CAN アダプター（Peak PCAN、Kvaser 等）または車載 ECU 経由 |
| Linux カーネルドライバ | socketcan（`ip link show` で `can0` 等が見えること） |
| 終端抵抗 | CAN バスの両端に 120Ω が必要 |
| ボーレート | ARS408 デフォルト: **500 kbps** |

---

### 1. CAN インターフェースのセットアップ

```bash
# インターフェース名を確認
ip link show

# CAN インターフェースを 500kbps で起動（can0 の場合）
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0

# 起動確認
ip link show can0
```

---

### 2. 通信確認（GUI 起動前）

```bash
# ターミナル 1: CAN 受信モニター（レーダーから #201 が来ているか確認）
candump can0

# 正常なら以下のような出力が 0.5秒ごとに流れる
#   can0  201  [8]  40 18 80 00 A0 F4 00 00
```

> **注意**: `candump` で何も表示されない場合は配線・終端抵抗・ボーレートを確認してください。

---

### 3. GUI 起動と設定変更

```bash
python3 src/main.py
```

1. **CAN Interface** 欄に `can0`（実際のインターフェース名）を入力
2. **Connect** ボタンをクリック → `● Connected`（緑）になることを確認
3. 変更したいフィールドの `_valid` チェックボックスをオンにする
4. 値を設定する
5. **Save** ボタンで設定を保存（再起動後の確認に備えて）
6. **Send #200** ボタンで設定コマンドを送信

---

### 4. レーダー再起動と設定反映の確認

```
Send #200 送信
   ↓
レーダーの電源を OFF → ON（再起動）
   ↓
レーダーが新しい設定で起動し #201 を送信し始める
   ↓
GUI の比較表が自動更新される
   ↓
✓（緑）= 設定が正しく反映された
✗（赤）= 設定が反映されていない
```

> **注意**: ARS408 は #200 受信だけでは設定を即時反映しません。  
> **電源再投入（再起動）後に初めて新しい設定が #201 に反映されます。**

---

### 5. 確認のポイント

| 確認項目 | 方法 |
|---|---|
| 送信前の現在設定を知りたい | Connect → #201 が自動受信されて右パネルに表示される |
| 設定変更を確実にしたい | 変更したいフィールドのみ `_valid` をオンにする（他フィールドは上書きされない） |
| 設定を保存したい | Send 前に **Save** ボタンで `settings.json` に保存 → 次回起動時に **Load** で復元 |
| 再起動前後を比較したい | Send #200 直後の比較表（✗が多い）と再起動後の比較表（✓が増える）を比較する |

---

### ros2_socketcan との共存

`ros2_socketcan` と本ツールを同じ CAN インターフェースで同時使用する場合、  
両プロセスが同じ `can0` を読み書きします。  
**受信は両方で行われますが、送信が衝突する可能性があります。**

設定変更時は `ros2_socketcan` のノードを一時停止することを推奨します。

```bash
# ros2_socketcan のノードを停止する場合
ros2 lifecycle set /socket_can_receiver shutdown
ros2 lifecycle set /socket_can_sender shutdown
```

---

## 処理フロー

### 全体の通信フロー

```
【GUI: main.py】              【vcan0】          【Simulator: radar_simulator.py】

ユーザーが設定入力
  → encode_can200()
  → #200 送信 ─────────────── 0x200 ──────────────▶ decode_can200()
                                                       apply_cfg_to_state()
                                                       内部状態を更新
                                                       state_to_can201()
parse_can201()                                         #201 送信
_update_state_display() ◀─── 0x201 ──────────────────
比較表に ✓/✗ 表示
```

---

### `main.py`（GUI）の処理フロー

#### Connect ボタン

```
入力: CAN インターフェース名（vcan0）
  ↓
CanInterface.connect()  vcan0 に接続
  ↓
RX スレッド起動  CAN ID=0x201 の受信待機を開始
```

#### Send #200 ボタン

```
入力: GUI の各ウィジェット（チェックボックス・スピンボックス・コンボボックス）
  ↓
_read_cfg_from_ui()     ウィジェットの値 → RadarCfg（Python オブジェクト）
  ↓
encode_can200()         RadarCfg → 8 バイトのバイト列
  ↓
decode_can200()         バイト列に戻して実効値を保存（2m 丸め誤差を吸収）
  ↓
vcan0 に CAN ID=0x200 で送信
  ↓
バナーを「#200 送信済 — #201 待ち」（黄色）に更新
比較表の Match 列を「未受信」（灰色）に更新
```

#### #201 受信時

```
入力: vcan0 から受信した #201 の 8 バイト
  ↓  ※ RX スレッドで受信 → メインスレッドに .after(0, ...) で転送
parse_can201()          バイト列 → RadarState（各フィールドの値）
  ↓
COMPARE_FIELDS を順番に比較
  ├─ #200 未送信              → Sent 列・Match 列に「未設定」
  ├─ valid=False のフィールド → 「未設定」
  └─ valid=True のフィールド
       ├─ 一致 → Match 列に「✓」（緑）
       └─ 不一致 → Match 列に「✗」（赤）
  ↓
バナーを「#200 送信済 ＋ #201 受信 — 比較有効」（緑）に更新
センサー状態パネル（エラーフラグ・NVM 状態など）も更新
```

---

### `radar_simulator.py`（シミュレーター）の処理フロー

#### 起動時

```
起動
  ├─ RX スレッド（常時待機）──── vcan0 を監視 → #200 受信したら処理
  └─ TX スレッド（定期実行）──── 0.5 秒ごとに内部状態を #201 として送信
```

初期状態は実機の工場出荷値（`40 18 80 00 A0 F4 00 00`）に相当する値で起動します。

#### #200 受信時

```
入力: CAN#200 の 8 バイト（例: FF 09 C0 00 29 9E 00 00）
  ↓
decode_can200()         バイト列 → RadarCfg（各フィールドの値と valid フラグ）
  ↓
apply_cfg_to_state()   valid=True のフィールドだけ内部状態に上書き
                        ※ valid=False のフィールドは変更しない
  ↓
state_to_can201()       更新後の内部状態 → #201 の 8 バイトに変換
  ↓
出力: CAN ID=0x201 で vcan0 に送信（例: C0 09 C0 00 91 F4 00 00）
```

#### 定期送信（TX スレッド）

```
0.5 秒ごとに現在の内部状態 → state_to_can201() → vcan0 に #201 送信
```

GUI が Connect した瞬間から現在状態が表示されるのはこのためです。

---

## CAN メッセージのビット配置

### CAN#200 (RadarCfg) — 送信コマンド

| Byte | 内容 |
|------|------|
| 0    | valid フラグ（bit0=MaxDistance, bit1=SensorID, bit2=RadarPower, bit3=OutputType, bit4=SendQuality, bit5=SendExtInfo, bit6=SortIndex, bit7=StoreInNVM） |
| 1-2  | MaxDistance（10bit, 解像度 2m） |
| 3    | 未使用 |
| 4    | SensorID[2:0] / OutputType[4:3] / RadarPower[7:5] |
| 5    | CtrlRelay_valid[0] / CtrlRelay[1] / SendQuality[2] / SendExtInfo[3] / SortIndex[5:4] / StoreInNVM[7] |
| 6    | RCS_Threshold_valid[0] / RCS_Threshold[3:1] |
| 7    | 未使用 |

### CAN#201 (RadarState) — 受信ステータス

| Byte | 内容 |
|------|------|
| 0    | NVMwriteStatus[7] / NVMReadStatus[6] |
| 1-2  | MaxDistance（10bit, 解像度 2m） / エラーフラグ[5:1] |
| 3    | RadarPower 上位ビット |
| 4    | RadarPower 最下位ビット[7] / SortIndex[6:4] / SensorID[2:0] |
| 5    | MotionRxState[7:6] / SendExtInfo[5] / SendQuality[4] / OutputType[3:2] / CtrlRelay[1] |
| 6    | 未使用 |
| 7    | RCS_Threshold[4:2] |

> **注意**: #200 と #201 は同じフィールドでもビット位置が異なります。  
> 例: RadarPower は #200 では byte4[7:5] ですが、#201 では byte3 + byte4[7] に分散しています。

---

## 比較表のステータス

| 表示 | 意味 |
|------|------|
| 未設定（灰色） | #200 を未送信、または valid=OFF のフィールド |
| 未受信（灰色） | #200 送信済みだが #201 がまだ届いていない |
| ✓（緑） | 送信値と受信値が一致 |
| ✗（赤） | 送信値と受信値が不一致（まだ設定が反映されていない） |

---

## テストの実行

```bash
# #201 デコードと #200 エンコードの基本テスト
python3 test/test_decode.py

# 各フィールドの送受信テスト（全 46 ケース）
python3 test/test_fields.py
```

---

## オプション

### `radar_simulator.py`

| オプション | デフォルト | 説明 |
|---|---|---|
| `--channel` | `vcan0` | CAN インターフェース名 |
| `--interval` | `0.5` | #201 定期送信間隔（秒）。`0` にすると #200 受信時のみ送信 |

```bash
# 実機 CAN インターフェースを使用する場合
python3 src/radar_simulator.py --channel can0

# #200 受信時のみ #201 を返す場合
python3 src/radar_simulator.py --interval 0
```
