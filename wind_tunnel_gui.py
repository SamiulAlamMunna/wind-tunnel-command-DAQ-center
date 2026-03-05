"""
WIND TUNNEL COMMAND CENTER
Dual HX711 Load Cells (Lift + Drag) + Servo Angle of Attack Control
Arduino serial format (output):
  "Load_cell 1 output val: X    Load_cell 2 output val: Y    AOA: Z"
Arduino serial commands (input):
  "A<float>\n"  -> set AoA    "t\n" -> tare both cells
Baud: 57600

pip install pyqt5 pyqtgraph pyserial numpy scipy
"""

import sys, re, time, csv
from collections import deque
from datetime import datetime

import numpy as np
import serial
import serial.tools.list_ports

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QComboBox, QSlider, QGroupBox,
    QStatusBar, QDoubleSpinBox, QCheckBox, QSizePolicy,
    QScrollArea, QFileDialog, QTabWidget, QTextEdit
)
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal, QObject
from PyQt5.QtGui import QColor
import pyqtgraph as pg

try:
    from scipy.signal import savgol_filter
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False

# ── Colors ────────────────────────────────────────────────────
C_BG     = "#060B13"
C_PANEL  = "#0C1623"
C_BORDER = "#192D44"
C_ACCENT = "#00C8F0"
C_DRAG   = "#FF6B35"
C_GREEN  = "#00E58A"
C_YELLOW = "#FFD060"
C_PURPLE = "#A78BFA"
C_RED    = "#FF4466"
C_RAW_L  = "#1A3A5A"
C_RAW_D  = "#4A2010"
C_TEXT   = "#C8DCF0"
C_DIM    = "#3A5570"
MAX_PTS  = 600
AOA_PTS  = 800
MIN_Y_G  = 15.0
DEFAULT_DEAD_ZONE = 0.0

# ════════════════════════════════════════════════════════════════
# SERIAL WORKER
# ════════════════════════════════════════════════════════════════
class SerialWorker(QObject):
    data_received = pyqtSignal(float, float, float)
    status        = pyqtSignal(str)
    tare_done     = pyqtSignal()
    raw_line      = pyqtSignal(str)
    aoa_confirmed = pyqtSignal(float)

    def __init__(self):
        super().__init__()
        self.ser = None
        self.running = False
        self._last_aoa = 0.0

    def connect(self, port, baud=57600):
        try:
            self.ser = serial.Serial(port, baud, timeout=0.5)
            time.sleep(2)
            self.ser.reset_input_buffer()
            self.running = True
            self.status.emit(f"Connected  {port} @ {baud} baud")
            return True
        except Exception as e:
            self.status.emit(f"Connection failed: {e}")
            return False

    def disconnect(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()

    def send_tare(self):
        if self.ser and self.ser.is_open:
            self.ser.write(b't\n')

    def send_aoa(self, angle):
        if self.ser and self.ser.is_open:
            self.ser.write(f"A{angle:.2f}\n".encode())

    def run(self):
        num = re.compile(r"[-+]?\d+\.?\d*")
        while self.running:
            try:
                if self.ser and self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if not line:
                        continue
                    self.raw_line.emit(line)

                    if "Tare load cell" in line or "Tare complete" in line:
                        self.tare_done.emit()

                    elif line.startswith("AOA_SET:"):
                        m = num.search(line.split("AOA_SET:")[-1])
                        if m:
                            v = float(m.group())
                            self._last_aoa = v
                            self.aoa_confirmed.emit(v)

                    elif "Load_cell 1 output val:" in line:
                        parts = re.split(r'output val:', line)
                        floats = []
                        for p in parts[1:]:
                            m2 = num.search(p)
                            if m2:
                                floats.append(float(m2.group()))
                        aoa_m = re.search(r'AOA:\s*([-+]?\d+\.?\d*)', line)
                        aoa = float(aoa_m.group(1)) if aoa_m else self._last_aoa
                        self._last_aoa = aoa
                        if len(floats) >= 2:
                            self.data_received.emit(floats[0], floats[1], aoa)
                        elif len(floats) == 1:
                            self.data_received.emit(floats[0], 0.0, aoa)
                            self.status.emit("WARNING: Only 1 load cell detected")

            except Exception as e:
                self.status.emit(f"Serial error: {e}")
            time.sleep(0.001)

# ════════════════════════════════════════════════════════════════
# FILTERS
# ════════════════════════════════════════════════════════════════
class Filters:
    @staticmethod
    def moving_average(d, w):
        if len(d) < 2: return np.array(d)
        return np.convolve(d, np.ones(min(w,len(d)))/min(w,len(d)), mode='same')

    @staticmethod
    def median(d, w):
        if len(d) < 2: return np.array(d)
        r = np.array(d, dtype=float); w = min(w, len(d))
        for i in range(len(r)):
            lo, hi = max(0,i-w//2), min(len(r),i+w//2+1)
            r[i] = np.median(r[lo:hi])
        return r

    @staticmethod
    def ema(d, alpha):
        if len(d) < 2: return np.array(d)
        r = np.zeros(len(d)); r[0] = d[0]
        for i in range(1, len(d)):
            r[i] = alpha*d[i] + (1-alpha)*r[i-1]
        return r

    @staticmethod
    def savgol(d, w, poly=3):
        if not HAS_SCIPY or len(d) < w: return np.array(d)
        w = w if w%2==1 else w+1
        return savgol_filter(d, max(w, poly+2), poly)

    @staticmethod
    def apply(d, ftype, window, alpha):
        if ftype == "Moving Average":  return Filters.moving_average(d, window)
        if ftype == "Median":          return Filters.median(d, window)
        if ftype == "EMA":             return Filters.ema(d, alpha)
        if ftype == "Savitzky-Golay":  return Filters.savgol(d, window)
        return np.array(d)

# ════════════════════════════════════════════════════════════════
# STYLE HELPERS
# ════════════════════════════════════════════════════════════════
def make_group(title, color=None):
    c = color or C_ACCENT
    g = QGroupBox(title)
    g.setStyleSheet(f"""
        QGroupBox {{ color:{c}; border:1px solid {C_BORDER}; border-radius:7px;
            margin-top:14px; font-size:9pt; font-weight:bold;
            font-family:monospace; padding:10px; }}
        QGroupBox::title {{ subcontrol-origin:margin; padding:0 8px; }}
    """)
    return g

def lbl(text, size=9, color=C_TEXT, bold=False):
    l = QLabel(text)
    l.setStyleSheet(f"color:{color}; font-size:{size}pt; font-family:monospace;"
                    + (" font-weight:bold;" if bold else ""))
    return l

def sep():
    w = QWidget(); w.setFixedHeight(1)
    w.setStyleSheet(f"background:{C_BORDER};"); return w

SPIN  = f"QSpinBox,QDoubleSpinBox{{background:{C_PANEL};color:{C_ACCENT};border:1px solid {C_BORDER};border-radius:4px;padding:3px 6px;font-size:10pt;font-family:monospace;}}"
COMBO = f"QComboBox{{background:{C_PANEL};color:{C_TEXT};border:1px solid {C_BORDER};border-radius:4px;padding:4px 8px;font-size:9pt;font-family:monospace;}}QComboBox QAbstractItemView{{background:{C_PANEL};color:{C_TEXT};selection-background-color:{C_BORDER};}}"

def pbtn(text, color=C_ACCENT, h=38):
    b = QPushButton(text); b.setFixedHeight(h)
    b.setStyleSheet(
        f"QPushButton{{background:transparent;color:{color};border:1.5px solid {color};"
        f"border-radius:6px;padding:6px;font-size:9pt;font-weight:bold;font-family:monospace;}}"
        f"QPushButton:hover{{background:{color}22;}}QPushButton:checked{{background:{color}33;}}"
        f"QPushButton:disabled{{border-color:{C_BORDER};color:{C_DIM};}}")
    return b

def sbtn(text, color=C_DIM, h=32):
    b = QPushButton(text); b.setFixedHeight(h)
    b.setStyleSheet(
        f"QPushButton{{background:{C_PANEL};color:{color};border:1px solid {C_BORDER};"
        f"border-radius:5px;padding:4px;font-size:8pt;font-family:monospace;}}"
        f"QPushButton:hover{{background:{C_BORDER};color:{C_TEXT};}}")
    return b

def chk_style(color=C_TEXT):
    return (f"QCheckBox{{color:{color};font-family:monospace;font-size:9pt;spacing:8px;}}"
            f"QCheckBox::indicator{{width:16px;height:16px;border:1px solid {C_BORDER};"
            f"border-radius:3px;background:{C_PANEL};}}"
            f"QCheckBox::indicator:checked{{background:{color};border-color:{color};}}")

def sld(color=C_ACCENT):
    return (f"QSlider::groove:horizontal{{background:{C_BORDER};height:4px;border-radius:2px;}}"
            f"QSlider::handle:horizontal{{background:{color};width:14px;height:14px;"
            f"margin:-5px 0;border-radius:7px;}}"
            f"QSlider::sub-page:horizontal{{background:{color}55;border-radius:2px;}}")

def fbtn(active=False, color=C_ACCENT):
    if active:
        return (f"QPushButton{{background:{color}22;color:{color};border:1.5px solid {color};"
                f"border-radius:5px;padding:6px 4px;font-size:8pt;font-weight:bold;font-family:monospace;}}")
    return (f"QPushButton{{background:{C_PANEL};color:{C_DIM};border:1px solid {C_BORDER};"
            f"border-radius:5px;padding:6px 4px;font-size:8pt;font-family:monospace;}}"
            f"QPushButton:hover{{background:{C_BORDER};color:{C_TEXT};}}")

# ════════════════════════════════════════════════════════════════
# CHANNEL STATE
# ════════════════════════════════════════════════════════════════
class ChannelState:
    def __init__(self):
        self.sw_offset       = 0.0
        self.dead_zone       = DEFAULT_DEAD_ZONE
        self.clamp           = True
        self._offset_samples = None
        self.raw_buf         = deque(maxlen=MAX_PTS)
        self.time_buf        = deque(maxlen=MAX_PTS)
        self.t0              = None

    def condition(self, val):
        val -= self.sw_offset
        if abs(val) < self.dead_zone: val = 0.0
        if self.clamp and val < 0:    val = 0.0
        return val

    def push(self, val, t):
        if self.t0 is None: self.t0 = t
        self.time_buf.append(t - self.t0)
        self.raw_buf.append(val)

    def arrays(self):
        return np.array(self.time_buf), np.array(self.raw_buf)

# ════════════════════════════════════════════════════════════════
# MAIN WINDOW
# ════════════════════════════════════════════════════════════════
class WindTunnelCommandCenter(QMainWindow):
    def __init__(self):
        super().__init__()
        self.lift = ChannelState()
        self.drag = ChannelState()
        self.records     = []
        self._recording  = False
        self._current_aoa = 0.0
        self._aoa_updating = False
        self.aoa_polar   = deque(maxlen=AOA_PTS)  # (aoa, lift, drag, ld)

        self.worker = SerialWorker()
        self.thread = QThread()
        self.worker.moveToThread(self.thread)
        self.worker.data_received.connect(self._on_data)
        self.worker.status.connect(self._set_status)
        self.worker.tare_done.connect(lambda: self._set_status("Tare complete ✓"))
        self.worker.raw_line.connect(self._on_raw_line)
        self.worker.aoa_confirmed.connect(self._on_aoa_confirmed)
        self.thread.started.connect(self.worker.run)

        self._active_filter   = "Median"
        self._auto_zero_mode  = False
        self._az_stable_since = None

        self._build_ui()
        self._apply_style()

        self.plot_timer = QTimer()
        self.plot_timer.timeout.connect(self._refresh)
        self.plot_timer.start(50)
        self._refresh_ports()

    # ── UI ────────────────────────────────────────────────────
    def _build_ui(self):
        self.setWindowTitle("WIND TUNNEL COMMAND CENTER")
        self.setMinimumSize(1600, 900)
        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)
        root.setContentsMargins(12,12,12,12); root.setSpacing(12)

        sc = QScrollArea()
        sc.setWidget(self._build_sidebar())
        sc.setWidgetResizable(True); sc.setFixedWidth(410)
        sc.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        sc.setStyleSheet(
            f"QScrollArea{{border:none;background:{C_BG};}}"
            f"QScrollBar:vertical{{background:{C_PANEL};width:6px;border-radius:3px;}}"
            f"QScrollBar::handle:vertical{{background:{C_BORDER};border-radius:3px;}}")
        root.addWidget(sc)
        root.addWidget(self._build_plots(), stretch=1)

        self.status_bar = QStatusBar()
        self.status_bar.setStyleSheet(
            f"background:{C_PANEL};color:{C_DIM};font-family:monospace;font-size:8pt;")
        self.setStatusBar(self.status_bar)
        self._set_status("Disconnected — select COM port and click CONNECT")

    # ── Sidebar ───────────────────────────────────────────────
    def _build_sidebar(self):
        w = QWidget(); w.setMinimumWidth(386)
        lay = QVBoxLayout(w)
        lay.setContentsMargins(4,4,10,12); lay.setSpacing(12)

        # Header
        h = QLabel("WIND TUNNEL\nCOMMAND CENTER")
        h.setAlignment(Qt.AlignCenter)
        h.setStyleSheet(
            f"color:{C_ACCENT};font-size:12pt;font-weight:bold;font-family:monospace;"
            f"letter-spacing:2px;padding:10px 0 8px 0;border-bottom:1px solid {C_BORDER};")
        lay.addWidget(h)
        s = lbl("LIFT: Cell 1  |  DRAG: Cell 2  |  AoA: Servo Pin 9", 7, C_DIM)
        s.setAlignment(Qt.AlignCenter); lay.addWidget(s)

        # ── Connection ──
        cg = make_group("CONNECTION"); cl = QVBoxLayout(cg); cl.setSpacing(8)
        pr = QHBoxLayout()
        self.port_combo = QComboBox()
        self.port_combo.setStyleSheet(COMBO); self.port_combo.setFixedHeight(34)
        br2 = sbtn("⟳"); br2.setFixedSize(34,34); br2.clicked.connect(self._refresh_ports)
        pr.addWidget(self.port_combo, stretch=1); pr.addWidget(br2); cl.addLayout(pr)
        br = QHBoxLayout(); br.addWidget(lbl("Baud:", 9, C_DIM))
        self.baud_combo = QComboBox()
        self.baud_combo.setStyleSheet(COMBO); self.baud_combo.setFixedHeight(34)
        self.baud_combo.addItems(["57600","9600","115200","38400"])
        br.addWidget(self.baud_combo, stretch=1); cl.addLayout(br)
        self.btn_conn = pbtn("CONNECT", C_ACCENT)
        self.btn_conn.setCheckable(True); self.btn_conn.clicked.connect(self._toggle_conn)
        cl.addWidget(self.btn_conn)
        self.lbl_conn = lbl("● OFFLINE", 10, C_DRAG, bold=True)
        self.lbl_conn.setAlignment(Qt.AlignCenter); self.lbl_conn.setFixedHeight(24)
        cl.addWidget(self.lbl_conn)
        lay.addWidget(cg)

        # ── Angle of Attack Control ──
        ag = make_group("ANGLE OF ATTACK CONTROL", C_YELLOW)
        al = QVBoxLayout(ag); al.setSpacing(8)

        self.lbl_aoa_big = QLabel("0.0°")
        self.lbl_aoa_big.setAlignment(Qt.AlignCenter)
        self.lbl_aoa_big.setFixedHeight(58)
        self.lbl_aoa_big.setStyleSheet(
            f"color:{C_YELLOW};font-size:36pt;font-weight:900;font-family:monospace;"
            f"background:{C_YELLOW}0D;border:1px solid {C_YELLOW}44;border-radius:8px;")
        al.addWidget(self.lbl_aoa_big)

        self.lbl_aoa_fb = lbl("Arduino: waiting…", 7, C_DIM)
        self.lbl_aoa_fb.setAlignment(Qt.AlignCenter); al.addWidget(self.lbl_aoa_fb)
        al.addWidget(sep())

        # Slider: -40° to +20°  (×10 integer: -400 to 200)
        sl_row = QHBoxLayout()
        sl_row.addWidget(lbl("-40°", 8, C_DIM))
        self.slider_aoa = QSlider(Qt.Horizontal)
        self.slider_aoa.setRange(-400, 200)
        self.slider_aoa.setValue(0); self.slider_aoa.setFixedHeight(30)
        self.slider_aoa.setStyleSheet(sld(C_YELLOW))
        self.slider_aoa.valueChanged.connect(self._on_aoa_slider)
        sl_row.addWidget(self.slider_aoa, stretch=1)
        sl_row.addWidget(lbl("+20°", 8, C_DIM)); al.addLayout(sl_row)

        ticks = QHBoxLayout()
        for t in ["-40°", "-20°", "0°", "+10°", "+20°"]:
            tl = lbl(t, 7, C_DIM); tl.setAlignment(Qt.AlignCenter); ticks.addWidget(tl, stretch=1)
        al.addLayout(ticks)
        al.addWidget(sep())

        al.addWidget(lbl("FINE STEP", 7, C_DIM, True))
        step_row = QHBoxLayout(); step_row.setSpacing(4)
        for label, delta in [("−5°",-5),("−1°",-1),("−0.5°",-0.5),("0°",None),("+0.5°",+0.5),("+1°",+1),("+5°",+5)]:
            b = sbtn(label, C_YELLOW if delta else C_GREEN, 32)
            if delta is None:
                b.clicked.connect(lambda: self._set_aoa(0.0))
            else:
                b.clicked.connect(lambda _, d=delta: self._step_aoa(d))
            step_row.addWidget(b)
        al.addLayout(step_row)
        al.addWidget(sep())

        # Presets — 3 rows covering -40 → +20
        al.addWidget(lbl("PRESETS", 7, C_DIM, True))
        pre_row1 = QHBoxLayout(); pre_row1.setSpacing(4)
        pre_row2 = QHBoxLayout(); pre_row2.setSpacing(4)
        pre_row3 = QHBoxLayout(); pre_row3.setSpacing(4)
        presets = [-40, -30, -20, -15, -10, -5, 0, 5, 10, 15, 20]
        for i, p in enumerate(presets):
            b = sbtn(f"{p:+d}°", C_YELLOW, 30)
            b.clicked.connect(lambda _, a=p: self._set_aoa(float(a)))
            if i < 4:
                pre_row1.addWidget(b)
            elif i < 8:
                pre_row2.addWidget(b)
            else:
                pre_row3.addWidget(b)
        al.addLayout(pre_row1)
        al.addLayout(pre_row2)
        al.addLayout(pre_row3)
        al.addWidget(sep())

        ex_row = QHBoxLayout()
        ex_row.addWidget(lbl("EXACT:", 8, C_DIM))
        self.spin_aoa = QDoubleSpinBox()
        self.spin_aoa.setRange(-90,90); self.spin_aoa.setDecimals(1)
        self.spin_aoa.setSingleStep(0.5); self.spin_aoa.setValue(0.0)
        self.spin_aoa.setSuffix("°"); self.spin_aoa.setFixedHeight(34); self.spin_aoa.setStyleSheet(SPIN)
        self.spin_aoa.valueChanged.connect(self._on_aoa_spin)
        ex_row.addWidget(self.spin_aoa, stretch=1)
        btn_send = pbtn("SEND", C_YELLOW, 34)
        btn_send.clicked.connect(lambda: self._set_aoa(self.spin_aoa.value()))
        ex_row.addWidget(btn_send); al.addLayout(ex_row)
        lay.addWidget(ag)

        # ── Live Readout ──
        rg = make_group("LIVE READOUT"); rl = QHBoxLayout(rg); rl.setSpacing(8)
        for ch, color, label in [("lift",C_ACCENT,"LIFT  (Cell 1)"),("drag",C_DRAG,"DRAG  (Cell 2)")]:
            col = QVBoxLayout(); col.setSpacing(3)
            tag = QLabel(label); tag.setAlignment(Qt.AlignCenter)
            tag.setStyleSheet(
                f"color:{C_BG};background:{color};font-size:8pt;font-weight:bold;"
                f"font-family:monospace;padding:4px;border-radius:4px;")
            col.addWidget(tag)
            rl2 = QLabel("— g"); rl2.setAlignment(Qt.AlignCenter)
            rl2.setStyleSheet(f"color:{color};font-size:11pt;font-weight:bold;font-family:monospace;")
            rl2.setFixedHeight(26)
            col.addWidget(lbl("RAW",7,C_DIM,True)); col.addWidget(rl2)
            fl2 = QLabel("—"); fl2.setAlignment(Qt.AlignCenter)
            fl2.setStyleSheet(f"color:{color};font-size:24pt;font-weight:900;font-family:monospace;")
            fl2.setFixedHeight(48)
            col.addWidget(lbl("FILTERED",7,C_DIM,True)); col.addWidget(fl2)
            col.addWidget(lbl("grams",7,C_DIM))
            sg = QGridLayout(); sg.setSpacing(3); stat_labels = {}
            for i,(k,t) in enumerate([("min","MIN"),("max","MAX"),("mean","AVG"),("std","σ")]):
                sg.addWidget(lbl(t,7,C_DIM,True), i//2*2, i%2*2)
                v = lbl("—",8,C_YELLOW,True); sg.addWidget(v, i//2*2+1, i%2*2)
                stat_labels[k] = v
            col.addLayout(sg)
            if ch == "drag":
                col.addWidget(sep())
                col.addWidget(lbl("L/D RATIO",7,C_PURPLE,True))
                self.lbl_ld = QLabel("—"); self.lbl_ld.setAlignment(Qt.AlignCenter)
                self.lbl_ld.setStyleSheet(
                    f"color:{C_PURPLE};font-size:18pt;font-weight:900;font-family:monospace;")
                col.addWidget(self.lbl_ld)
            setattr(self, f"lbl_{ch}_raw", rl2)
            setattr(self, f"lbl_{ch}_filt", fl2)
            setattr(self, f"stat_{ch}", stat_labels)
            panel = QWidget()
            panel.setStyleSheet(
                f"background:{C_PANEL};border:1px solid {color}33;border-radius:6px;")
            pl = QVBoxLayout(panel); pl.setContentsMargins(8,8,8,8); pl.addLayout(col)
            rl.addWidget(panel, stretch=1)
        lay.addWidget(rg)

        # ── Filter Settings ──
        fg = make_group("FILTER SETTINGS"); fl = QVBoxLayout(fg); fl.setSpacing(10)
        fl.addWidget(lbl("Applied equally to both channels", 7, C_DIM))
        fl.addWidget(lbl("FILTER TYPE", 8, C_DIM, True))
        self.filter_btns = {}
        fnames = ["Moving Average","Median","EMA","None"]
        if HAS_SCIPY: fnames.insert(3,"Savitzky-Golay")
        short = {"Moving Average":"MOV AVG","Median":"MEDIAN","EMA":"EMA",
                 "Savitzky-Golay":"SAV-GOL","None":"NONE"}
        descs = {"Moving Average":"General smoothing",
                 "Median":"Kills zero-spike dropouts ★",
                 "EMA":"Fast, tuneable alpha",
                 "Savitzky-Golay":"Best curve (scipy)","None":"Raw signal"}
        bg2 = QGridLayout(); bg2.setSpacing(5)
        for i, name in enumerate(fnames):
            b = QPushButton(short[name]); b.setCheckable(True)
            b.setChecked(name=="Median"); b.setFixedHeight(34); b.setToolTip(descs[name])
            b.clicked.connect(lambda _, n=name: self._select_filter(n))
            b.setStyleSheet(fbtn(name=="Median")); self.filter_btns[name] = b
            bg2.addWidget(b, i//2, i%2)
        fl.addLayout(bg2)
        self.lbl_fdesc = QLabel(descs["Median"]); self.lbl_fdesc.setWordWrap(True)
        self.lbl_fdesc.setMinimumHeight(32)
        self.lbl_fdesc.setStyleSheet(
            f"color:{C_ACCENT};font-size:8pt;font-family:monospace;"
            f"background:{C_ACCENT}11;border-left:3px solid {C_ACCENT};padding:5px 8px;border-radius:3px;")
        fl.addWidget(self.lbl_fdesc); fl.addWidget(sep())

        self.win_container = QWidget()
        wl = QVBoxLayout(self.win_container); wl.setContentsMargins(0,0,0,0); wl.setSpacing(4)
        wr = QHBoxLayout(); wr.addWidget(lbl("WINDOW",8,C_DIM,True))
        self.lbl_win = lbl("20",10,C_ACCENT,True); self.lbl_win.setAlignment(Qt.AlignRight)
        wr.addWidget(self.lbl_win); wl.addLayout(wr)
        self.sl_win = QSlider(Qt.Horizontal)
        self.sl_win.setRange(3,80); self.sl_win.setValue(20); self.sl_win.setFixedHeight(26)
        self.sl_win.setStyleSheet(sld(C_ACCENT))
        self.sl_win.valueChanged.connect(lambda v: self.lbl_win.setText(str(v)))
        wl.addWidget(self.sl_win)
        tr = QHBoxLayout()
        for t in ["3","  40  ","80"]:
            tl = lbl(t,7,C_DIM); tl.setAlignment(Qt.AlignCenter); tr.addWidget(tl, stretch=1)
        wl.addLayout(tr); fl.addWidget(self.win_container)

        self.alpha_container = QWidget()
        al3 = QVBoxLayout(self.alpha_container); al3.setContentsMargins(0,0,0,0); al3.setSpacing(4)
        ar = QHBoxLayout(); ar.addWidget(lbl("ALPHA",8,C_DIM,True))
        self.lbl_alpha = lbl("0.15",10,C_YELLOW,True); self.lbl_alpha.setAlignment(Qt.AlignRight)
        ar.addWidget(self.lbl_alpha); al3.addLayout(ar)
        self.sl_alpha = QSlider(Qt.Horizontal)
        self.sl_alpha.setRange(1,100); self.sl_alpha.setValue(15); self.sl_alpha.setFixedHeight(26)
        self.sl_alpha.setStyleSheet(sld(C_YELLOW))
        self.sl_alpha.valueChanged.connect(lambda v: self.lbl_alpha.setText(f"{v/100:.2f}"))
        al3.addWidget(self.sl_alpha); fl.addWidget(self.alpha_container)
        self.alpha_container.setVisible(False)
        fl.addWidget(sep())

        self.chk_raw = QCheckBox("Show raw traces")
        self.chk_raw.setChecked(True); self.chk_raw.setFixedHeight(26)
        self.chk_raw.setStyleSheet(chk_style(C_TEXT)); fl.addWidget(self.chk_raw)
        fl.addWidget(sep())

        self.btn_tare = QPushButton("⟳  TARE BOTH SENSORS")
        self.btn_tare.setFixedHeight(38); self.btn_tare.clicked.connect(self._do_tare)
        self.btn_tare.setStyleSheet(
            f"QPushButton{{background:{C_YELLOW}18;color:{C_YELLOW};"
            f"border:1.5px solid {C_YELLOW}88;border-radius:6px;"
            f"padding:6px;font-size:9pt;font-weight:bold;font-family:monospace;}}"
            f"QPushButton:hover{{background:{C_YELLOW}33;border-color:{C_YELLOW};}}")
        fl.addWidget(self.btn_tare)
        bc = QPushButton("✕  CLEAR ALL DATA"); bc.setFixedHeight(34); bc.clicked.connect(self._clear)
        bc.setStyleSheet(
            f"QPushButton{{background:{C_PANEL};color:{C_DIM};border:1px solid {C_BORDER};"
            f"border-radius:6px;padding:5px;font-size:9pt;font-family:monospace;}}"
            f"QPushButton:hover{{background:{C_BORDER};color:{C_TEXT};}}")
        fl.addWidget(bc); lay.addWidget(fg)

        # ── Zero Calibration ──
        zg = make_group("ZERO CALIBRATION"); zl = QVBoxLayout(zg); zl.setSpacing(8)
        self.chk_az = QCheckBox("Auto-zero when no load detected")
        self.chk_az.setFixedHeight(28); self.chk_az.setStyleSheet(chk_style(C_GREEN))
        self.chk_az.stateChanged.connect(self._toggle_auto_zero); zl.addWidget(self.chk_az)
        azdesc = QLabel("Both channels stable below threshold for 3s → offset auto-set")
        azdesc.setWordWrap(True)
        azdesc.setStyleSheet(
            f"color:{C_DIM};font-size:7pt;font-family:monospace;"
            f"background:{C_GREEN}08;border-left:2px solid {C_GREEN}44;padding:5px 7px;border-radius:3px;")
        zl.addWidget(azdesc)
        tzr = QHBoxLayout(); tzr.addWidget(lbl("Threshold:",9,C_DIM))
        self.spin_azt = QDoubleSpinBox()
        self.spin_azt.setRange(0.1,50); self.spin_azt.setDecimals(1)
        self.spin_azt.setValue(5.0); self.spin_azt.setSuffix(" g")
        self.spin_azt.setFixedHeight(32); self.spin_azt.setStyleSheet(SPIN)
        tzr.addWidget(self.spin_azt); zl.addLayout(tzr)
        self.lbl_azs = QLabel("○  Disabled")
        self.lbl_azs.setStyleSheet(f"color:{C_DIM};font-size:8pt;font-family:monospace;")
        zl.addWidget(self.lbl_azs); zl.addWidget(sep())

        tab = QTabWidget()
        tab.setStyleSheet(
            f"QTabWidget::pane{{border:1px solid {C_BORDER};border-radius:5px;background:{C_PANEL};}}"
            f"QTabBar::tab{{background:{C_PANEL};color:{C_DIM};padding:6px 14px;"
            f"font-family:monospace;font-size:8pt;border:1px solid {C_BORDER};}}"
            f"QTabBar::tab:selected{{color:{C_ACCENT};border-bottom:2px solid {C_ACCENT};}}")
        for ch, color, label in [("lift",C_ACCENT,"LIFT (Cell 1)"),("drag",C_DRAG,"DRAG (Cell 2)")]:
            page = QWidget(); pl = QVBoxLayout(page); pl.setSpacing(7); pl.setContentsMargins(8,10,8,8)
            pl.addWidget(lbl("MANUAL OFFSET",8,C_DIM,True))
            or_ = QHBoxLayout()
            soff = QDoubleSpinBox(); soff.setRange(-500,500); soff.setDecimals(2)
            soff.setSingleStep(0.5); soff.setValue(0.0); soff.setSuffix(" g")
            soff.setFixedHeight(32); soff.setStyleSheet(SPIN)
            soff.valueChanged.connect(lambda v,c=ch: setattr(getattr(self,c),'sw_offset',v))
            setattr(self, f"spin_offset_{ch}", soff)
            bc2 = QPushButton("CAPTURE NOW"); bc2.setFixedHeight(32)
            bc2.clicked.connect(lambda _,c=ch: self._auto_offset(c))
            bc2.setStyleSheet(
                f"QPushButton{{background:{color}18;color:{color};"
                f"border:1.5px solid {color}66;border-radius:5px;"
                f"padding:3px 7px;font-size:8pt;font-weight:bold;font-family:monospace;}}"
                f"QPushButton:hover{{background:{color}33;border-color:{color};}}")
            or_.addWidget(soff, stretch=1); or_.addWidget(bc2); pl.addLayout(or_)
            pl.addWidget(lbl("Remove load → click CAPTURE NOW",7,C_DIM)); pl.addWidget(sep())
            pl.addWidget(lbl("DEAD ZONE",8,C_DIM,True))
            sdz = QDoubleSpinBox(); sdz.setRange(0,50); sdz.setDecimals(1)
            sdz.setSingleStep(0.5); sdz.setValue(0.0); sdz.setSuffix(" g")
            sdz.setFixedHeight(32); sdz.setStyleSheet(SPIN)
            sdz.valueChanged.connect(lambda v,c=ch: setattr(getattr(self,c),'dead_zone',v))
            setattr(self, f"spin_dz_{ch}", sdz); pl.addWidget(sdz)
            pl.addWidget(lbl("Below this value → forced to 0.0 g",7,C_DIM))
            ckc = QCheckBox("Clamp negatives to 0"); ckc.setChecked(True); ckc.setFixedHeight(26)
            ckc.setStyleSheet(chk_style(C_DRAG))
            ckc.stateChanged.connect(lambda v,c=ch: setattr(getattr(self,c),'clamp',bool(v)))
            setattr(self, f"chk_clamp_{ch}", ckc); pl.addWidget(ckc)
            sl2 = lbl("Offset: 0.00g  |  DZ: 0.0g  |  Clamp: ON",7,C_DIM)
            setattr(self, f"lbl_calstat_{ch}", sl2); pl.addWidget(sl2); pl.addStretch()
            tab.addTab(page, label)
        zl.addWidget(tab); lay.addWidget(zg)

        # ── Data Export ──
        eg = make_group("DATA EXPORT"); el = QVBoxLayout(eg); el.setSpacing(7)
        self.lbl_sc = lbl("0 samples",9,C_DIM); self.lbl_sc.setAlignment(Qt.AlignCenter)
        el.addWidget(self.lbl_sc)
        self.btn_rec = pbtn("▶  START RECORDING", C_GREEN)
        self.btn_rec.setCheckable(True); self.btn_rec.clicked.connect(self._toggle_record)
        el.addWidget(self.btn_rec)
        bexp = pbtn("⬇  EXPORT CSV", C_ACCENT, 34); bexp.clicked.connect(self._export_csv)
        el.addWidget(bexp); lay.addWidget(eg)

        # ── Serial Monitor ──
        mg = make_group("SERIAL MONITOR"); ml = QVBoxLayout(mg); ml.setSpacing(5)
        self.serial_mon = QTextEdit(); self.serial_mon.setReadOnly(True)
        self.serial_mon.setFixedHeight(90)
        self.serial_mon.setStyleSheet(
            f"background:{C_BG};color:{C_GREEN};font-family:'Courier New',monospace;"
            f"font-size:8pt;border:1px solid {C_BORDER};border-radius:4px;padding:4px;")
        ml.addWidget(self.serial_mon)
        ml.addWidget(lbl("Blank after connect → wrong baud rate",7,C_DRAG))
        lay.addWidget(mg)

        lay.addStretch()
        return w

    # ── Plots ─────────────────────────────────────────────────
    def _build_plots(self):
        w = QWidget(); g = QGridLayout(w); g.setSpacing(8)

        def mp(title, ylabel, xlabel, color):
            p = pg.PlotWidget(); p.setBackground(C_BG)
            p.showGrid(x=True, y=True, alpha=0.13)
            p.setLabel('left',  ylabel, color=C_DIM, size='9pt')
            p.setLabel('bottom', xlabel, color=C_DIM, size='9pt')
            p.getAxis('left').setTextPen(C_DIM)
            p.getAxis('bottom').setTextPen(C_DIM)
            p.setTitle(title, color=color, size="10pt")
            p.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            return p

        # Row 0 — Time series
        self.plot_lift = mp("LIFT FORCE  vs  TIME",  "Lift (g)",  "Time (s)", C_ACCENT)
        self.curve_lift_raw  = self.plot_lift.plot(pen=pg.mkPen(C_RAW_L,width=1), name="Raw")
        self.curve_lift_filt = self.plot_lift.plot(pen=pg.mkPen(C_ACCENT,width=2), name="Filtered")
        self.plot_lift.addLegend(offset=(10,10)).setLabelTextColor(C_TEXT)

        self.plot_drag = mp("DRAG FORCE  vs  TIME",  "Drag (g)",  "Time (s)", C_DRAG)
        self.curve_drag_raw  = self.plot_drag.plot(pen=pg.mkPen(C_RAW_D,width=1), name="Raw")
        self.curve_drag_filt = self.plot_drag.plot(pen=pg.mkPen(C_DRAG,width=2), name="Filtered")
        self.plot_drag.addLegend(offset=(10,10)).setLabelTextColor(C_TEXT)

        self.plot_ld_time = mp("L/D RATIO  vs  TIME",  "L/D",  "Time (s)", C_PURPLE)
        self.curve_ld_time = self.plot_ld_time.plot(pen=pg.mkPen(C_PURPLE,width=2))

        # Row 1 — AoA sweep results
        self.plot_lift_aoa = mp("LIFT  vs  ANGLE OF ATTACK",  "Lift (g)",  "AoA (°)", C_ACCENT)
        self.curve_lift_aoa_line = self.plot_lift_aoa.plot(pen=pg.mkPen(C_ACCENT,width=2))
        self.curve_lift_aoa_pts  = self.plot_lift_aoa.plot(
            pen=None, symbol='o', symbolSize=5,
            symbolBrush=C_ACCENT+'99', symbolPen=pg.mkPen(C_ACCENT,width=1))
        self.curve_lift_aoa_now  = self.plot_lift_aoa.plot(
            pen=None, symbol='o', symbolSize=13,
            symbolBrush=C_ACCENT, symbolPen=pg.mkPen('#FFFFFF',width=1.5))

        self.plot_drag_aoa = mp("DRAG  vs  ANGLE OF ATTACK",  "Drag (g)",  "AoA (°)", C_DRAG)
        self.curve_drag_aoa_line = self.plot_drag_aoa.plot(pen=pg.mkPen(C_DRAG,width=2))
        self.curve_drag_aoa_pts  = self.plot_drag_aoa.plot(
            pen=None, symbol='o', symbolSize=5,
            symbolBrush=C_DRAG+'99', symbolPen=pg.mkPen(C_DRAG,width=1))
        self.curve_drag_aoa_now  = self.plot_drag_aoa.plot(
            pen=None, symbol='o', symbolSize=13,
            symbolBrush=C_DRAG, symbolPen=pg.mkPen('#FFFFFF',width=1.5))

        self.plot_polar = mp("DRAG POLAR  —  Lift vs Drag",  "Lift (g)",  "Drag (g)", C_GREEN)
        self.curve_polar_pts = self.plot_polar.plot(
            pen=None, symbol='o', symbolSize=5,
            symbolBrush=C_GREEN+'88', symbolPen=pg.mkPen(C_GREEN,width=1))
        self.curve_polar_now = self.plot_polar.plot(
            pen=None, symbol='o', symbolSize=14,
            symbolBrush=C_GREEN, symbolPen=pg.mkPen('#FFFFFF',width=1.5))

        g.addWidget(self.plot_lift,      0, 0)
        g.addWidget(self.plot_drag,      0, 1)
        g.addWidget(self.plot_ld_time,   0, 2)
        g.addWidget(self.plot_lift_aoa,  1, 0)
        g.addWidget(self.plot_drag_aoa,  1, 1)
        g.addWidget(self.plot_polar,     1, 2)
        for r in range(2): g.setRowStretch(r, 1)
        for c in range(3): g.setColumnStretch(c, 1)
        return w

    def _apply_style(self):
        self.setStyleSheet(
            f"QMainWindow,QWidget{{background:{C_BG};color:{C_TEXT};"
            f"font-family:'Segoe UI',sans-serif;}}")

    # ── Slots ─────────────────────────────────────────────────
    def _refresh_ports(self):
        cur = self.port_combo.currentText(); self.port_combo.clear()
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo.addItems(ports if ports else ["No ports"])
        if cur in ports: self.port_combo.setCurrentText(cur)

    def _toggle_conn(self, checked):
        if checked:
            port = self.port_combo.currentText(); baud = int(self.baud_combo.currentText())
            if self.worker.connect(port, baud):
                if not self.thread.isRunning(): self.thread.start()
                self.btn_conn.setText("DISCONNECT")
                self.lbl_conn.setText("● ONLINE")
                self.lbl_conn.setStyleSheet(
                    f"color:{C_GREEN};font-size:10pt;font-weight:bold;font-family:monospace;")
            else:
                self.btn_conn.setChecked(False)
        else:
            self.worker.disconnect(); self.btn_conn.setText("CONNECT")
            self.lbl_conn.setText("● OFFLINE")
            self.lbl_conn.setStyleSheet(
                f"color:{C_DRAG};font-size:10pt;font-weight:bold;font-family:monospace;")

    # ── AoA ───────────────────────────────────────────────────
    def _set_aoa(self, angle):
        angle = max(-90.0, min(90.0, round(angle*10)/10))
        self._current_aoa = angle
        self.worker.send_aoa(angle)
        self._update_aoa_ui(angle)

    def _step_aoa(self, delta):
        self._set_aoa(self._current_aoa + delta)

    def _on_aoa_slider(self, val):
        if self._aoa_updating: return
        self._aoa_updating = True
        angle = val / 10.0
        self.spin_aoa.setValue(angle)
        self._set_aoa(angle)
        self._aoa_updating = False

    def _on_aoa_spin(self, val):
        if self._aoa_updating: return
        self._aoa_updating = True
        self.slider_aoa.setValue(int(val * 10))
        self._aoa_updating = False

    def _update_aoa_ui(self, angle):
        self._aoa_updating = True
        self.lbl_aoa_big.setText(f"{angle:+.1f}°")
        self.slider_aoa.setValue(int(angle * 10))
        self.spin_aoa.setValue(angle)
        col = C_GREEN if abs(angle) < 5 else (C_YELLOW if abs(angle) < 15 else C_DRAG)
        self.lbl_aoa_big.setStyleSheet(
            f"color:{col};font-size:36pt;font-weight:900;font-family:monospace;"
            f"background:{col}0D;border:1px solid {col}44;border-radius:8px;")
        self._aoa_updating = False

    def _on_aoa_confirmed(self, angle):
        self.lbl_aoa_fb.setText(f"✓ Arduino confirmed: {angle:+.2f}°")
        self.lbl_aoa_fb.setStyleSheet(
            f"color:{C_GREEN};font-size:7pt;font-family:monospace;")

    # ── Data ──────────────────────────────────────────────────
    def _on_data(self, raw_lift, raw_drag, aoa):
        t_now = time.time()
        self._current_aoa = aoa
        self._update_aoa_ui(aoa)

        for ch_name, raw_val in [("lift",raw_lift),("drag",raw_drag)]:
            ch = getattr(self, ch_name)
            if ch._offset_samples is not None:
                ch._offset_samples.append(raw_val)
                if len(ch._offset_samples) >= 50:
                    new_off = float(np.mean(ch._offset_samples))
                    ch.sw_offset = new_off
                    getattr(self, f"spin_offset_{ch_name}").setValue(new_off)
                    ch._offset_samples = None
                    self._set_status(f"{ch_name.upper()} auto-zero → {new_off:.2f} g")
                else:
                    self._set_status(f"Auto-zeroing {ch_name.upper()}… {len(ch._offset_samples)}/50")
                continue
            ch.push(ch.condition(raw_val), t_now)

        self.lbl_lift_raw.setText(f"{raw_lift:.2f} g")
        self.lbl_drag_raw.setText(f"{raw_drag:.2f} g")

        if self._recording:
            t_rel = t_now - (self.lift.t0 or t_now)
            self.records.append((t_rel, aoa, raw_lift, raw_drag,
                                  self.lift.condition(raw_lift), self.drag.condition(raw_drag)))
            self.lbl_sc.setText(f"{len(self.records)} samples recorded")

    def _do_tare(self):
        self.worker.send_tare(); self._set_status("Tare sent to both sensors…")

    def _clear(self):
        for ch in (self.lift, self.drag):
            ch.raw_buf.clear(); ch.time_buf.clear(); ch.t0 = None
        self.aoa_polar.clear()
        for c in [self.curve_lift_raw, self.curve_lift_filt,
                  self.curve_drag_raw, self.curve_drag_filt,
                  self.curve_ld_time,
                  self.curve_lift_aoa_line, self.curve_lift_aoa_pts,
                  self.curve_drag_aoa_line, self.curve_drag_aoa_pts,
                  self.curve_polar_pts]:
            c.setData([], [])
        self._set_status("All data cleared")

    def _select_filter(self, name):
        self._active_filter = name
        descs = {"Moving Average":"General smoothing — good default",
                 "Median":"Kills zero-spike dropouts — recommended for HX711",
                 "EMA":"Fast response with tuneable alpha",
                 "Savitzky-Golay":"Best curve shape — preserves peaks (scipy)",
                 "None":"Raw unfiltered signal"}
        for n, b in self.filter_btns.items():
            b.setStyleSheet(fbtn(n==name)); b.setChecked(n==name)
        self.lbl_fdesc.setText(descs.get(name,""))
        self.alpha_container.setVisible("EMA" in name)
        self.win_container.setVisible(name not in ("None","EMA"))

    def _toggle_auto_zero(self, state):
        self._auto_zero_mode = bool(state); self._az_stable_since = None
        self.lbl_azs.setText("◉  Watching both channels…" if self._auto_zero_mode else "○  Disabled")
        self.lbl_azs.setStyleSheet(
            f"color:{C_GREEN if self._auto_zero_mode else C_DIM};font-size:8pt;font-family:monospace;")

    def _check_auto_zero(self, lf, df):
        if not self._auto_zero_mode: return
        thresh = self.spin_azt.value(); now = time.time()
        if abs(lf) <= thresh and abs(df) <= thresh:
            if self._az_stable_since is None: self._az_stable_since = now
            rem = max(0, 3.0-(now-self._az_stable_since))
            self.lbl_azs.setText(f"◉  No load — zeroing in {rem:.1f}s…")
            self.lbl_azs.setStyleSheet(f"color:{C_YELLOW};font-size:8pt;font-family:monospace;")
            if now - self._az_stable_since >= 3.0:
                for cn in ("lift","drag"):
                    ch = getattr(self, cn)
                    if len(ch.raw_buf) > 5:
                        noff = ch.sw_offset + float(np.mean(list(ch.raw_buf)[-20:]))
                        ch.sw_offset = noff
                        getattr(self, f"spin_offset_{cn}").setValue(noff)
                self._az_stable_since = None
                self.lbl_azs.setText("✓  Both channels auto-zeroed")
                self.lbl_azs.setStyleSheet(f"color:{C_GREEN};font-size:8pt;font-family:monospace;")
        else:
            self._az_stable_since = None
            self.lbl_azs.setText("◉  Load detected — watching…")
            self.lbl_azs.setStyleSheet(f"color:{C_DIM};font-size:8pt;font-family:monospace;")

    def _auto_offset(self, ch_name):
        ch = getattr(self, ch_name); ch._offset_samples = []
        self._set_status(f"Auto-zeroing {ch_name.upper()} — keep UNLOADED…")

    def _toggle_record(self, checked):
        self._recording = checked
        self.btn_rec.setText("■  STOP RECORDING" if checked else "▶  START RECORDING")
        self._set_status("Recording started…" if checked else f"Stopped — {len(self.records)} samples")

    def _export_csv(self):
        if not self.records: self._set_status("No data to export"); return
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        path, _ = QFileDialog.getSaveFileName(
            self, "Export CSV", f"wind_tunnel_{ts}.csv", "CSV Files (*.csv)")
        if path:
            with open(path,'w',newline='') as f:
                cw = csv.writer(f)
                cw.writerow(["Time (s)","AoA (deg)","Lift Raw (g)","Drag Raw (g)",
                              "Lift Filt (g)","Drag Filt (g)"])
                cw.writerows(self.records)
            self._set_status(f"Exported {len(self.records)} records → {path}")

    def _set_status(self, msg):
        self.status_bar.showMessage(f"  {msg}")

    def _on_raw_line(self, line):
        self.serial_mon.append(line)
        doc = self.serial_mon.document()
        while doc.blockCount() > 25:
            cur = self.serial_mon.textCursor()
            cur.movePosition(cur.Start); cur.select(cur.BlockUnderCursor)
            cur.removeSelectedText(); cur.deleteChar()

    # ── Plot refresh (50ms) ───────────────────────────────────
    def _refresh(self):
        if len(self.lift.raw_buf) < 2 or len(self.drag.raw_buf) < 2: return

        lt, l_raw = self.lift.arrays()
        dt, d_raw = self.drag.arrays()
        ftype  = self._active_filter
        window = self.sl_win.value()
        alpha  = self.sl_alpha.value() / 100.0

        l_filt = Filters.apply(l_raw, ftype, window, alpha)
        d_filt = Filters.apply(d_raw, ftype, window, alpha)
        last_l = float(l_filt[-1])
        last_d = float(d_filt[-1])

        show = self.chk_raw.isChecked()
        self.curve_lift_raw.setData(lt, l_raw)  if show else self.curve_lift_raw.setData([],[])
        self.curve_drag_raw.setData(dt, d_raw)  if show else self.curve_drag_raw.setData([],[])
        self.curve_lift_filt.setData(lt, l_filt)
        self.curve_drag_filt.setData(dt, d_filt)

        for plot, arr in [(self.plot_lift, np.concatenate([l_raw,l_filt])),
                          (self.plot_drag, np.concatenate([d_raw,d_filt]))]:
            y_max = max(float(arr.max()), MIN_Y_G)
            y_min = min(float(arr.min()), 0.0)
            plot.setYRange(y_min, y_max, padding=0.08)

        n   = min(len(l_filt), len(d_filt))
        ld  = np.where(np.abs(d_filt[:n]) > 0.5, l_filt[:n] / d_filt[:n], 0.0)
        self.curve_ld_time.setData(lt[:n], ld)
        last_ld = float(ld[-1]) if len(ld) else 0.0

        self.lbl_lift_filt.setText(f"{last_l:.2f}")
        self.lbl_drag_filt.setText(f"{last_d:.2f}")
        self.lbl_ld.setText(f"{last_ld:.2f}")

        for cn, ra, sd in [("lift",l_raw,self.stat_lift),("drag",d_raw,self.stat_drag)]:
            sd["min"].setText(f"{ra.min():.1f}g"); sd["max"].setText(f"{ra.max():.1f}g")
            sd["mean"].setText(f"{ra.mean():.1f}g"); sd["std"].setText(f"{ra.std():.2f}g")
            ch = getattr(self, cn)
            getattr(self, f"lbl_calstat_{cn}").setText(
                f"Offset:{ch.sw_offset:.2f}g  DZ:{ch.dead_zone:.1f}g  "
                f"Clamp:{'ON' if ch.clamp else 'OFF'}")

        self.aoa_polar.append((self._current_aoa, last_l, last_d, last_ld))

        if len(self.aoa_polar) > 1 and len(self.lift.raw_buf) % 5 == 0:
            pol = np.array(self.aoa_polar)
            aoa_a  = pol[:,0]
            lift_a = pol[:,1]
            drag_a = pol[:,2]

            idx = np.argsort(aoa_a)
            aoa_s  = aoa_a[idx]
            lift_s = lift_a[idx]
            drag_s = drag_a[idx]

            self.curve_lift_aoa_pts.setData(aoa_a, lift_a)
            self.curve_lift_aoa_line.setData(aoa_s, lift_s)
            self.curve_lift_aoa_now.setData([self._current_aoa], [last_l])

            self.curve_drag_aoa_pts.setData(aoa_a, drag_a)
            self.curve_drag_aoa_line.setData(aoa_s, drag_s)
            self.curve_drag_aoa_now.setData([self._current_aoa], [last_d])

            self.curve_polar_pts.setData(drag_a, lift_a)
            self.curve_polar_now.setData([last_d], [last_l])

        self._check_auto_zero(last_l, last_d)

    def closeEvent(self, event):
        self.worker.disconnect(); self.thread.quit(); self.thread.wait(1000); event.accept()

# ── Entry ─────────────────────────────────────────────────────
if __name__ == "__main__":
    app = QApplication(sys.argv); app.setStyle("Fusion")
    pal = app.palette()
    for role, col in [(pal.Window,C_BG),(pal.WindowText,C_TEXT),(pal.Base,C_PANEL),
                      (pal.Text,C_TEXT),(pal.Button,C_PANEL),(pal.ButtonText,C_TEXT),
                      (pal.Highlight,C_ACCENT)]:
        pal.setColor(role, QColor(col))
    app.setPalette(pal)
    win = WindTunnelCommandCenter(); win.show(); sys.exit(app.exec_())
