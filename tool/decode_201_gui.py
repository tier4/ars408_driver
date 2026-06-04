#!/usr/bin/env python3
"""CAN#201 (RadarState) hex → bit layout viewer + field decoder"""
import sys
import pathlib
import tkinter as tk
from tkinter import messagebox

sys.path.insert(0, str(pathlib.Path(__file__).parent / "src"))
from radar_state import parse_can201, state_to_dict  # noqa: E402

# BIT_LAYOUT[byte][col]: col 0 = bit 7 (MSB), col 7 = bit 0 (LSB)
# None = undefined/unused (gray), str = field label (white)
_N = None
BIT_LAYOUT = [
    # byte 0
    ["NVMwrite", "NVMread",  _N,         _N,         _N,         _N,         _N,         _N        ],
    # byte 1  (MaxDistance bits 9-2)
    ["MaxDist",  "MaxDist",  "MaxDist",  "MaxDist",  "MaxDist",  "MaxDist",  "MaxDist",  "MaxDist" ],
    # byte 2  (MaxDistance bits 1-0, error flags)
    ["MaxDist",  "MaxDist",  "PrstErr",  "Interfer", "TempErr",  "TmpErr",   "VoltErr",  _N        ],
    # byte 3  (RadarPower msb at bits 1-0)
    [_N,         _N,         _N,         _N,         _N,         _N,         "RadPow",   "RadPow"  ],
    # byte 4  (RadarPower lsb at bit 7, SortIndex bits 6-4, SensorID bits 2-0)
    ["RadPow",   "SortIdx",  "SortIdx",  "SortIdx",  _N,         "SenID",    "SenID",    "SenID"   ],
    # byte 5
    ["MotionRx", "MotionRx", "ExtInfo",  "Quality",  "OutType",  "OutType",  "CtrlRly",  _N        ],
    # byte 6  (InvalidClusters, full byte)
    ["InvClst",  "InvClst",  "InvClst",  "InvClst",  "InvClst",  "InvClst",  "InvClst",  "InvClst" ],
    # byte 7  (RCS_Threshold bits 4-2)
    [_N,         _N,         _N,         "RCS_Thr",  "RCS_Thr",  "RCS_Thr",  _N,         _N        ],
]

GRAY  = "#CCCCCC"
WHITE = "white"
HDR   = "#D0D0D0"
RED   = "#FF9999"
GREEN = "#90EE90"
ERROR_WORDS = ("ERROR", "Missing", "Failed")


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("CAN#201 RadarState Decoder")
        self.resizable(False, False)
        self._cells: list[list[tk.Label]] = []
        self._build()

    # ------------------------------------------------------------------
    def _build(self):
        p = dict(padx=4, pady=3)

        # --- Hex input row ---
        bar = tk.Frame(self)
        bar.pack(fill="x", padx=8, pady=(10, 2))
        tk.Label(bar, text="CAN#201 hex:").pack(side="left", **p)
        self._hex_var = tk.StringVar(value="40 18 80 00 A0 F4 00 00")
        tk.Entry(bar, textvariable=self._hex_var, width=30,
                 font=("Courier", 10)).pack(side="left", **p)
        tk.Button(bar, text="Decode", width=8,
                  command=self._on_decode).pack(side="left", **p)

        # --- Bit-layout grid ---
        gf = tk.LabelFrame(self, text="Bit Layout  (Byte × Bit)", padx=6, pady=4)
        gf.pack(padx=8, pady=6)

        tk.Label(gf, text="Byte", width=5, bg=HDR, relief="ridge").grid(
            row=0, column=0, padx=1, pady=1)
        for c in range(8):
            tk.Label(gf, text=str(7 - c), width=9, bg=HDR,
                     relief="ridge").grid(row=0, column=c + 1, padx=1, pady=1)

        for r in range(8):
            tk.Label(gf, text=str(r), width=5, bg=HDR,
                     relief="ridge").grid(row=r + 1, column=0, padx=1, pady=1)
            row_cells = []
            for c in range(8):
                field = BIT_LAYOUT[r][c]
                lbl = tk.Label(
                    gf, text=field or "",
                    width=9, height=2,
                    bg=WHITE if field else GRAY,
                    relief="ridge", font=("", 8), anchor="center",
                )
                lbl.grid(row=r + 1, column=c + 1, padx=1, pady=1)
                row_cells.append(lbl)
            self._cells.append(row_cells)

        # --- Decoded-fields panel ---
        self._df = tk.LabelFrame(self, text="Decoded Fields", padx=6, pady=6)
        self._df.pack(fill="x", padx=8, pady=(0, 10))
        tk.Label(self._df, text="Decode a hex string above.", fg="gray").grid(
            row=0, column=0, columnspan=4)

    # ------------------------------------------------------------------
    def _on_decode(self):
        hex_str = self._hex_var.get().strip()
        if "#" in hex_str:                        # candump: "201#XXXXXXXX"
            hex_str = hex_str.split("#", 1)[1]
        try:
            data = bytes.fromhex(hex_str.replace(" ", ""))
            state = parse_can201(data)
        except Exception as e:
            messagebox.showerror("Decode Error", str(e))
            return

        # Update bit cells: show field name + actual bit value
        for r, byte_val in enumerate(data):
            for c in range(8):
                bit = (byte_val >> (7 - c)) & 1
                field = BIT_LAYOUT[r][c]
                self._cells[r][c].configure(
                    text=f"{field}\n{bit}" if field else str(bit),
                )

        # Rebuild decoded-fields panel
        for w in self._df.winfo_children():
            w.destroy()
        for i, (k, (raw, label)) in enumerate(state_to_dict(state).items()):
            r, col = divmod(i, 2)
            tk.Label(self._df, text=f"{k}:", anchor="e", width=22).grid(
                row=r, column=col * 2, sticky="e", padx=(4, 1))
            is_err = any(w in label for w in ERROR_WORDS)
            tk.Label(
                self._df, text=f"{label}  (raw={raw})",
                anchor="w", width=26, relief="sunken", bd=1,
                bg=RED if is_err else GREEN,
            ).grid(row=r, column=col * 2 + 1, sticky="w", padx=(0, 8), pady=1)


if __name__ == "__main__":
    App().mainloop()
