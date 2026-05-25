"""
ARS408 Radar Configuration GUI Tool
CAN#200 送信 / CAN#201 受信・比較表示
"""

import json
import os
import tkinter as tk
from tkinter import messagebox, ttk

from can_interface import CanInterface
from radar_cfg import (
    OUTPUT_TYPE_OPTS, RADAR_POWER_OPTS, RCS_THRESH_OPTS, SORT_INDEX_OPTS,
    RadarCfg, cfg_to_dict, dict_to_cfg, encode_can200, decode_can200,
)
from radar_state import (
    ERROR_FLAG, MOTION_RX, OUTPUT_TYPE, RADAR_POWER, RCS_THRESH, SORT_INDEX,
    STATUS, parse_can201,
)

SETTINGS_FILE = os.path.join(os.path.dirname(__file__), "settings.json")

# Fields compared between #200 sent value and #201 received value
COMPARE_FIELDS = [
    ("MaxDistance",  lambda v: f"{v} m",       None),
    ("SensorID",     str,                       None),
    ("RadarPower",   RADAR_POWER.get,           RADAR_POWER_OPTS),
    ("OutputType",   OUTPUT_TYPE.get,           OUTPUT_TYPE_OPTS),
    ("SendQuality",  lambda v: "Active" if v else "Inactive", None),
    ("SendExtInfo",  lambda v: "Active" if v else "Inactive", None),
    ("SortIndex",    SORT_INDEX.get,            SORT_INDEX_OPTS),
    ("CtrlRelay",    lambda v: "Active" if v else "Inactive", None),
    ("RCS_Threshold",RCS_THRESH.get,            RCS_THRESH_OPTS),
]

ERROR_FIELDS = [
    "VoltageError", "TemporaryError", "TemperatureError",
    "Interference", "PersistentError",
]


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ARS408 Radar Config Tool")
        self.resizable(False, False)

        self._can = CanInterface()
        self._last_cfg: RadarCfg = RadarCfg()
        self._last_state = None
        self._cfg_sent: bool = False      # #200を一度でも送ったか
        self._state_received_after_send: bool = False  # 送信後に#201を受信したか

        self._build_ui()
        self._load_settings()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------
    def _build_ui(self):
        pad = {"padx": 6, "pady": 3}

        # === Top: CAN interface bar ===
        top = tk.Frame(self, bd=1, relief="groove")
        top.pack(fill="x", padx=8, pady=(8, 0))
        tk.Label(top, text="CAN Interface:").pack(side="left", **pad)
        self._iface_var = tk.StringVar(value="vcan0")
        self._entry_iface = tk.Entry(top, textvariable=self._iface_var, width=12)
        self._entry_iface.pack(side="left", **pad)
        self._btn_connect = tk.Button(top, text="Connect",    width=9, command=self._on_connect)
        self._btn_connect.pack(side="left", **pad)
        self._btn_disconnect = tk.Button(top, text="Disconnect", width=9,
                                         command=self._on_disconnect, state="disabled")
        self._btn_disconnect.pack(side="left", **pad)
        self._lbl_status = tk.Label(top, text="● Disconnected", fg="red")
        self._lbl_status.pack(side="left", **pad)

        ttk.Separator(top, orient="vertical").pack(side="left", fill="y", padx=8)

        tk.Label(top, text="Current Sensor ID:").pack(side="left", **pad)
        self._cur_sensor_id_var = tk.IntVar(value=0)
        self._spn_sensor_id = tk.Spinbox(top, from_=0, to=7, increment=1,
                                          textvariable=self._cur_sensor_id_var,
                                          width=3)
        self._spn_sensor_id.pack(side="left", **pad)
        tk.Button(top, text="Apply", width=6,
                  command=self._on_apply_sensor_id).pack(side="left", **pad)


        # === Main: left (send) / right (compare + sensor) ===
        main = tk.Frame(self)
        main.pack(fill="both", padx=8, pady=8)

        self._build_send_panel(main)
        self._build_compare_panel(main)

    # --- Left: send (#200) ---
    def _build_send_panel(self, parent):
        self._frm_send = tk.LabelFrame(parent, text="Send Config  (CAN 0x200)", padx=6, pady=6)
        self._frm_send.pack(side="left", fill="y", padx=(0, 6))
        frame = self._frm_send

        self._valid_vars: dict[str, tk.BooleanVar] = {}
        self._value_widgets: dict[str, tk.Widget] = {}
        self._value_vars: dict[str, tk.Variable] = {}

        rows = [
            # (label, field, widget_type, options_or_range)
            ("MaxDistance [m]", "MaxDistance",  "spinbox", (196, 1200, 2)),
            ("SensorID",        "SensorID",     "spinbox", (0, 7, 1)),
            ("OutputType",      "OutputType",   "combo",   OUTPUT_TYPE_OPTS),
            ("SendQuality",     "SendQuality",  "combo",   {0: "Inactive", 1: "Active"}),
            ("SendExtInfo",     "SendExtInfo",  "combo",   {0: "Inactive", 1: "Active"}),
            ("SortIndex",       "SortIndex",    "combo",   SORT_INDEX_OPTS),
            ("CtrlRelay",       "CtrlRelay",    "combo",   {0: "Inactive", 1: "Active"}),
            ("StoreInNVM",      "StoreInNVM",   "combo",   {0: "Inactive", 1: "Active"}),
            ("RCS_Threshold",   "RCS_Threshold","combo",   RCS_THRESH_OPTS),
        ]

        for i, (label, field, wtype, opts) in enumerate(rows):
            vv = tk.BooleanVar(value=False)
            self._valid_vars[field] = vv
            chk = tk.Checkbutton(frame, text=f"{field}_valid",
                                 variable=vv,
                                 command=lambda f=field: self._on_valid_toggle(f))
            chk.grid(row=i, column=0, sticky="w", pady=1)

            if wtype == "spinbox":
                from_, to, inc = opts
                var = tk.IntVar(value=from_)
                w = tk.Spinbox(frame, from_=from_, to=to, increment=inc,
                               textvariable=var, width=8, state="disabled")
            else:
                keys = list(opts.keys())
                labels = list(opts.values())
                var = tk.IntVar(value=keys[0])
                w = ttk.Combobox(frame, values=labels, width=14, state="disabled")
                w._keys = keys
                w._labels = labels
                w.current(0)
                var._combo = w

            self._value_vars[field] = var
            self._value_widgets[field] = w
            tk.Label(frame, text=label, width=16, anchor="w").grid(row=i, column=1, sticky="w", padx=4)
            w.grid(row=i, column=2, sticky="w")

        # RadarPower 固定表示ラベル（RCS_Threshold の下）
        fixed_row = len(rows)
        tk.Label(frame,
                 text="※ RadarPower: -3dB 固定（毎回更新）",
                 fg="#333333", font=("", 10, "bold"),
                 anchor="w").grid(row=fixed_row, column=0, columnspan=3,
                                  sticky="w", pady=(6, 0))

        # Buttons
        btn_frame = tk.Frame(frame)
        btn_frame.grid(row=fixed_row + 1, column=0, columnspan=3, pady=(10, 0))
        self._btn_send = tk.Button(btn_frame, text="Send 0x200", width=12,
                                   command=self._on_send)
        self._btn_send.pack(side="left", padx=4)
        tk.Button(btn_frame, text="Save", width=8,
                  command=self._save_settings).pack(side="left", padx=4)
        tk.Button(btn_frame, text="Load", width=8,
                  command=self._load_settings).pack(side="left", padx=4)

        # Hex preview (#200)
        self._lbl_hex200_title = tk.Label(frame, text="CAN 0x200 hex (cansend用):")
        self._lbl_hex200_title.grid(row=fixed_row+2, column=0, columnspan=3, sticky="w", pady=(8, 0))
        self._lbl_hex200 = tk.Label(frame, text="--", font=("Courier", 10), anchor="w")
        self._lbl_hex200.grid(row=fixed_row+3, column=0, columnspan=3, sticky="w")

    # --- Right: compare (#200 vs #201) + sensor status ---
    def _build_compare_panel(self, parent):
        right = tk.Frame(parent)
        right.pack(side="left", fill="both", expand=True)

        # === Compare table ===
        self._frm_cmp = tk.LabelFrame(right,
                                      text="Config Comparison  (0x200 sent vs 0x201 received)",
                                      padx=6, pady=6)
        self._frm_cmp.pack(fill="x")
        cmp_frame = self._frm_cmp

        # 状態バナー
        self._lbl_send_state = tk.Label(
            cmp_frame,
            text="⚠ #200 未送信 — 比較は無効（Send #200 を押してください）",
            fg="white", bg="#CC6600", font=("", 9, "bold"),
            anchor="w", padx=6, pady=3,
        )
        self._lbl_send_state.grid(row=0, column=0, columnspan=4, sticky="ew", pady=(0, 4))

        self._lbl_cmp_hdr_sent = tk.Label(cmp_frame, text="Sent (0x200)",
                                           font=("", 9, "bold"), width=14, anchor="w")
        self._lbl_cmp_hdr_recv = tk.Label(cmp_frame, text="Received (0x201)",
                                           font=("", 9, "bold"), width=16, anchor="w")
        headers_static = ["Signal", "Match"]
        tk.Label(cmp_frame, text="Signal", font=("", 9, "bold"),
                 width=18, anchor="w").grid(row=1, column=0, padx=2)
        self._lbl_cmp_hdr_sent.grid(row=1, column=1, padx=2)
        self._lbl_cmp_hdr_recv.grid(row=1, column=2, padx=2)
        tk.Label(cmp_frame, text="Match", font=("", 9, "bold"),
                 width=6, anchor="w").grid(row=1, column=3, padx=2)

        self._cmp_rows: dict[str, list[tk.Label]] = {}
        for i, (field, fmt, _) in enumerate(COMPARE_FIELDS, start=2):
            row_labels = []
            for col in range(4):
                lbl = tk.Label(cmp_frame, text="--",
                               width=[18, 14, 16, 6][col], anchor="w",
                               relief="sunken", bd=1)
                lbl.grid(row=i, column=col, padx=2, pady=1, sticky="w")
                row_labels.append(lbl)
            row_labels[0].configure(text=field)
            self._cmp_rows[field] = row_labels


        # #201 受信 hex 表示（candump 確認用・編集不可）
        hex_frame = tk.Frame(cmp_frame)
        hex_frame.grid(row=len(COMPARE_FIELDS)+2, column=0, columnspan=4,
                       sticky="w", pady=(6, 2))
        self._lbl_hex201_title = tk.Label(hex_frame, text="CAN 0x201 hex (candump用):")
        self._lbl_hex201_title.pack(side="left", padx=(2, 6))
        self._lbl_hex201 = tk.Label(hex_frame, text="--",
                                    font=("Courier", 10), anchor="w",
                                    relief="sunken", bd=1, width=28)
        self._lbl_hex201.pack(side="left")

        # === Sensor status ===
        self._frm_status = tk.LabelFrame(right, text="Sensor Status  (0x201)", padx=6, pady=6)
        self._frm_status.pack(fill="x", pady=(8, 0))
        st_frame = self._frm_status

        status_fields = [
            ("NVMReadStatus",  STATUS),
            ("NVMwriteStatus", STATUS),
            ("MotionRxState",  MOTION_RX),
        ] + [(f, ERROR_FLAG) for f in ERROR_FIELDS]

        self._status_labels: dict[str, tk.Label] = {}
        for i, (field, _lmap) in enumerate(status_fields):
            col = (i % 2) * 2
            row = i // 2
            tk.Label(st_frame, text=f"{field}:", anchor="e", width=20).grid(
                row=row, column=col, sticky="e", padx=(4, 2))
            lbl = tk.Label(st_frame, text="--", width=20, anchor="w",
                           relief="sunken", bd=1)
            lbl.grid(row=row, column=col+1, sticky="w", padx=(0, 8))
            self._status_labels[field] = lbl

    # ------------------------------------------------------------------
    # Event handlers
    # ------------------------------------------------------------------
    def _on_valid_toggle(self, field: str):
        enabled = self._valid_vars[field].get()
        w = self._value_widgets[field]
        if isinstance(w, ttk.Combobox):
            w.configure(state="readonly" if enabled else "disabled")
        else:
            w.configure(state="normal" if enabled else "disabled")
        self._update_hex_preview()

    def _on_apply_sensor_id(self):
        """Current Sensor ID を反映し、送受信 CAN ID を更新する。"""
        try:
            sid = int(self._cur_sensor_id_var.get())
        except (ValueError, tk.TclError):
            self._spn_sensor_id.configure(bg="#FF9999")
            messagebox.showerror("入力エラー", "Sensor ID は整数で入力してください（0〜7）")
            return
        if sid < 0 or sid > 7:
            self._spn_sensor_id.configure(bg="#FF9999")
            messagebox.showerror("入力エラー", f"Sensor ID は 0〜7 の範囲で入力してください（入力値: {sid}）")
            return
        self._spn_sensor_id.configure(bg="white")
        self._can.set_sensor_id(sid)
        self._refresh_can_id_display()

    def _refresh_can_id_display(self):
        """送受信 CAN ID に関わるすべての表示を更新する。"""
        cfg_id   = self._can.cfg_id
        state_id = self._can.state_id
        # Send Config パネルのタイトル
        self._frm_send.configure(text=f"Send Config  (CAN 0x{cfg_id:03X})")
        # Send ボタンのテキスト
        self._btn_send.configure(text=f"Send 0x{cfg_id:03X}")
        # Config Comparison パネルのタイトルとヘッダー
        self._frm_cmp.configure(
            text=f"Config Comparison  (0x{cfg_id:03X} sent vs 0x{state_id:03X} received)")
        self._lbl_cmp_hdr_sent.configure(text=f"Sent (0x{cfg_id:03X})")
        self._lbl_cmp_hdr_recv.configure(text=f"Received (0x{state_id:03X})")
        # hex ラベルタイトル
        self._lbl_hex200_title.configure(text=f"CAN 0x{cfg_id:03X} hex (cansend用):")
        self._lbl_hex201_title.configure(text=f"CAN 0x{state_id:03X} hex (candump用):")
        # Sensor Status パネルのタイトル
        self._frm_status.configure(text=f"Sensor Status  (0x{state_id:03X})")
        # cansend プレビューも更新
        self._update_hex_preview()

    def _on_connect(self):
        iface = self._iface_var.get().strip()
        if not iface:
            self._entry_iface.configure(bg="#FF9999")
            messagebox.showerror("入力エラー", "CAN インターフェース名を入力してください")
            return
        try:
            self._can.channel = iface
            self._can.connect()
            self._can.register_state_callback(self._on_state_received)
            self._entry_iface.configure(bg="white")
            self._lbl_status.configure(text="● Connected", fg="green")
            self._btn_connect.configure(state="disabled")
            self._btn_disconnect.configure(state="normal")
        except Exception as e:
            self._entry_iface.configure(bg="#FF9999")
            messagebox.showerror("Connection Error", str(e))

    def _on_disconnect(self):
        self._can.disconnect()
        self._lbl_status.configure(text="● Disconnected", fg="red")
        self._btn_connect.configure(state="normal")
        self._btn_disconnect.configure(state="disabled")

    def _validate_send_inputs(self) -> bool:
        """送信前の入力値バリデーション。問題があれば該当ウィジェットを赤くして False を返す。"""
        ok = True

        # MaxDistance: 有効時は 196〜1200 の偶数
        w_dist = self._value_widgets["MaxDistance"]
        if self._valid_vars["MaxDistance"].get():
            try:
                v = int(self._value_vars["MaxDistance"].get())
            except (ValueError, tk.TclError):
                v = None
            if v is None:
                w_dist.configure(bg="#FF9999")
                messagebox.showerror("入力エラー", "MaxDistance は整数で入力してください（196〜1200、偶数）")
                ok = False
            elif v < 196 or v > 1200:
                w_dist.configure(bg="#FF9999")
                messagebox.showerror("入力エラー",
                    f"MaxDistance は 196〜1200 m の範囲で入力してください（入力値: {v}）")
                ok = False
            elif v % 2 != 0:
                w_dist.configure(bg="#FF9999")
                messagebox.showerror("入力エラー",
                    f"MaxDistance は偶数（2m 単位）で入力してください（入力値: {v}）")
                ok = False
            else:
                w_dist.configure(bg="white")
        else:
            w_dist.configure(bg="white")

        # SensorID: 有効時は 0〜7 の範囲
        w_sid = self._value_widgets["SensorID"]
        if self._valid_vars["SensorID"].get():
            try:
                v = int(self._value_vars["SensorID"].get())
            except (ValueError, tk.TclError):
                v = None
            if v is None:
                w_sid.configure(bg="#FF9999")
                if ok:
                    messagebox.showerror("入力エラー", "SensorID は整数で入力してください（0〜7）")
                ok = False
            elif v < 0 or v > 7:
                w_sid.configure(bg="#FF9999")
                if ok:
                    messagebox.showerror("入力エラー",
                        f"SensorID は 0〜7 の範囲で入力してください（入力値: {v}）")
                ok = False
            else:
                w_sid.configure(bg="white")
        else:
            w_sid.configure(bg="white")

        return ok

    def _on_send(self):
        if not self._validate_send_inputs():
            return
        cfg = self._read_cfg_from_ui()
        data = encode_can200(cfg)
        # エンコード→デコードして実際に送出される値を保存（解像度丸めを反映）
        self._last_cfg = decode_can200(data)
        self._cfg_sent = True
        self._state_received_after_send = False
        hex_str = " ".join(f"{b:02X}" for b in data)

        # SensorID を変更する場合、再起動後の受信 CAN ID を自動更新
        if self._last_cfg.SensorID_valid:
            new_state_id = 0x201 + self._last_cfg.SensorID * 0x10
            self._can.set_state_id(new_state_id)

        self._refresh_can_id_display()
        self._update_send_banner()
        # 比較列を「未設定→待機中」に更新
        self._reset_compare_to_waiting()
        if self._can.is_connected:
            try:
                self._can.send_cfg(data)
                messagebox.showinfo("Sent", f"CAN#200 sent:\n{hex_str}")
            except Exception as e:
                messagebox.showerror("Send Error", str(e))
        else:
            messagebox.showwarning("Not Connected", f"Not connected. Encoded data:\n{hex_str}")

    def _on_state_received(self, raw: bytes):
        """Called from RX thread; schedule UI update on main thread."""
        self.after(0, lambda: self._update_state_display(raw))

    def _update_send_banner(self):
        if not self._cfg_sent:
            self._lbl_send_state.configure(
                text="⚠ #200 未送信 — 比較は無効（Send #200 を押してください）",
                bg="#CC6600",
            )
        elif not self._state_received_after_send:
            self._lbl_send_state.configure(
                text="◑ #200 送信済 — #201 自動受信待ち（レーダーを再起動してください）",
                bg="#888800",
            )
        else:
            self._lbl_send_state.configure(
                text="✔ #200 送信済 ＋ #201 受信 — 比較有効",
                bg="#006600",
            )

    def _reset_compare_to_waiting(self):
        """#200送信直後、比較列をリセットする（#201受信前の状態）。"""
        cfg = self._last_cfg
        for field, fmt, opts in COMPARE_FIELDS:
            row_labels = self._cmp_rows[field]
            sent_val = getattr(cfg, field, None)
            valid_flag = getattr(cfg, f"{field}_valid", None)
            if valid_flag is False:
                sent_str = "未設定"
            else:
                if opts:
                    sent_str = opts.get(int(sent_val), str(sent_val)) if sent_val is not None else "未設定"
                else:
                    sent_str = fmt(sent_val) if sent_val is not None else "未設定"
            row_labels[1].configure(text=sent_str)
            row_labels[2].configure(text="--")
            row_labels[3].configure(text="未受信", bg="lightgray")

    # ------------------------------------------------------------------
    # State display
    # ------------------------------------------------------------------
    def _update_state_display(self, raw: bytes):
        try:
            state = parse_can201(raw)
        except Exception as e:
            messagebox.showerror("Decode Error", str(e))
            return

        self._last_state = state
        cfg = self._last_cfg

        # #201 受信 hex を更新（candump 確認用）
        self._lbl_hex201.configure(
            text=" ".join(f"{b:02X}" for b in raw))

        if self._cfg_sent:
            self._state_received_after_send = True
        self._update_send_banner()

        # Compare table
        for field, fmt, opts in COMPARE_FIELDS:
            row_labels = self._cmp_rows[field]
            recv_val = getattr(state, field, None)

            # Received value (常に表示)
            if opts:
                recv_str = opts.get(int(recv_val), str(recv_val)) if recv_val is not None else "--"
            else:
                recv_str = fmt(recv_val) if recv_val is not None else "--"
            row_labels[2].configure(text=recv_str)

            if not self._cfg_sent:
                # #200未送信: Sent列・Match列は「未設定」
                row_labels[1].configure(text="未設定")
                row_labels[3].configure(text="未設定", bg="lightgray")
                continue

            # #200送信済みの場合: 比較する
            sent_val = getattr(cfg, field, None)
            valid_flag = getattr(cfg, f"{field}_valid", None)
            if valid_flag is False:
                sent_str = "未設定"
                row_labels[1].configure(text=sent_str)
                row_labels[3].configure(text="未設定", bg="lightgray")
            else:
                if opts:
                    sent_str = opts.get(int(sent_val), str(sent_val)) if sent_val is not None else "--"
                else:
                    sent_str = fmt(sent_val) if sent_val is not None else "--"
                row_labels[1].configure(text=sent_str)
                match = (sent_val == recv_val)
                if match:
                    row_labels[3].configure(text="✓", bg="#90EE90")
                else:
                    row_labels[3].configure(text="✗", bg="#FF9999")

        # Sensor status labels
        status_map = {
            "NVMReadStatus":   STATUS.get(state.NVMReadStatus, "?"),
            "NVMwriteStatus":  STATUS.get(state.NVMwriteStatus, "?"),
            "MotionRxState":   MOTION_RX.get(state.MotionRxState, "?"),
            "VoltageError":    ERROR_FLAG[state.VoltageError],
            "TemporaryError":  ERROR_FLAG[state.TemporaryError],
            "TemperatureError":ERROR_FLAG[state.TemperatureError],
            "Interference":    ERROR_FLAG[state.Interference],
            "PersistentError": ERROR_FLAG[state.PersistentError],
        }
        for field, text in status_map.items():
            lbl = self._status_labels.get(field)
            if lbl:
                is_error = text in ("ERROR", "Failed", "FAILED") or "Missing" in text
                lbl.configure(text=text, bg="#FF9999" if is_error else "#90EE90")

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _read_cfg_from_ui(self) -> RadarCfg:
        cfg = RadarCfg()
        for field, vv in self._valid_vars.items():
            setattr(cfg, f"{field}_valid", vv.get())

        bool_fields = {"SendQuality", "SendExtInfo", "CtrlRelay", "StoreInNVM"}
        for field, var in self._value_vars.items():
            w = self._value_widgets[field]
            if isinstance(w, ttk.Combobox):
                idx = w.current()
                raw = w._keys[idx] if idx >= 0 else 0
                if field in bool_fields:
                    setattr(cfg, field, bool(raw))
                else:
                    setattr(cfg, field, int(raw))
            else:
                try:
                    setattr(cfg, field, int(var.get()))
                except Exception:
                    pass

        # RadarPower は常に -3dB 固定で送信
        cfg.RadarPower_valid = True
        cfg.RadarPower = 1

        return cfg

    def _update_hex_preview(self):
        cfg = self._read_cfg_from_ui()
        data = encode_can200(cfg)
        iface = self._iface_var.get().strip() or "can0"
        hex_bytes = " ".join(f"{b:02X}" for b in data)
        hex_nospc = "".join(f"{b:02X}" for b in data)
        cfg_id = self._can.cfg_id
        self._lbl_hex200.configure(
            text=f"{hex_bytes}   cansend: {iface} {cfg_id:03X}#{hex_nospc}")

    def _save_settings(self):
        cfg = self._read_cfg_from_ui()
        d = cfg_to_dict(cfg)
        d["can_interface"]    = self._iface_var.get()
        d["current_sensor_id"] = self._cur_sensor_id_var.get()
        with open(SETTINGS_FILE, "w") as f:
            json.dump(d, f, indent=2)
        messagebox.showinfo("Saved", f"Settings saved to\n{SETTINGS_FILE}")

    def _load_settings(self):
        if not os.path.exists(SETTINGS_FILE):
            return
        try:
            with open(SETTINGS_FILE) as f:
                d = json.load(f)
        except Exception as e:
            messagebox.showerror("Load Error", str(e))
            return

        if "can_interface" in d:
            self._iface_var.set(d["can_interface"])
        if "current_sensor_id" in d:
            self._cur_sensor_id_var.set(int(d["current_sensor_id"]))
            self._on_apply_sensor_id()

        cfg = dict_to_cfg(d)
        self._apply_cfg_to_ui(cfg)

    def _apply_cfg_to_ui(self, cfg: RadarCfg):
        bool_fields = {"SendQuality", "SendExtInfo", "CtrlRelay", "StoreInNVM"}
        for field, vv in self._valid_vars.items():
            valid = getattr(cfg, f"{field}_valid", False)
            vv.set(valid)
            w = self._value_widgets[field]
            val = getattr(cfg, field, None)
            if val is None:
                continue
            if isinstance(w, ttk.Combobox):
                int_val = int(val) if not isinstance(val, bool) else (1 if val else 0)
                keys = w._keys
                if int_val in keys:
                    w.current(keys.index(int_val))
                w.configure(state="readonly" if valid else "disabled")
            else:
                try:
                    self._value_vars[field].set(int(val))
                except Exception:
                    pass
                w.configure(state="normal" if valid else "disabled")
        self._update_hex_preview()


if __name__ == "__main__":
    app = App()
    app.mainloop()
