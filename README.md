# 🌬️ Wind Tunnel Command Center — Web Edition

A browser-based port of the PyQt5 Wind Tunnel GUI. Connect your Arduino + dual HX711 load cells directly from Chrome/Edge using the **Web Serial API** — no Python, no installation required.

![Status](https://img.shields.io/badge/status-stable-00E58A) ![License](https://img.shields.io/badge/license-MIT-00C8F0)

---

## ✨ Features

| Feature | Details |
|---|---|
| **Dual load cell readout** | Live lift (Cell 1) and drag (Cell 2) in grams |
| **Angle of Attack control** | Slider, step buttons (±0.5°, ±1°, ±5°), presets, exact entry |
| **5 signal filters** | Moving Average, Median ★, EMA, Savitzky-Golay (approx), None |
| **6 live plots** | Lift/Drag/L·D vs time, Lift/Drag vs AoA, Drag Polar |
| **Zero calibration** | Manual offset, dead-zone, clamp, auto-zero on no-load |
| **CSV export** | One-click download of all recorded samples |
| **Demo mode** | Simulated physics data — works offline, no hardware needed |
| **Web Serial** | Real Arduino connection in Chrome/Edge (desktop) |

---

## 🚀 Quick Start

### Option A — GitHub Pages (recommended)

1. Fork this repo
2. Go to **Settings → Pages → Source: main / root**
3. Your app is live at `https://<you>.github.io/<repo>/`

### Option B — Local file

```bash
git clone https://github.com/<you>/<repo>.git
# Open index.html in Chrome or Edge
```

> ⚠️ Web Serial requires a **secure context** (HTTPS or localhost).  
> Opening `index.html` directly as a `file://` URL disables Serial — use a local server:
> ```bash
> python3 -m http.server 8080
> # then open http://localhost:8080
> ```

---

## 🔌 Arduino Connection

### Requirements
- **Browser**: Chrome ≥ 89 or Edge ≥ 89 (desktop only — not mobile, not Firefox/Safari)
- **Arduino sketch**: must output the same serial format as the PyQt5 version:

```
Load_cell 1 output val: X    Load_cell 2 output val: Y    AOA: Z
```

### Connecting
1. Plug in your Arduino
2. Click **CONNECT** — the browser will show a port picker
3. Select your COM port / `/dev/ttyUSB*`
4. Default baud: **57600** (change in the dropdown to match your sketch)

### Arduino Commands Sent by the Web App
| Action | Serial command |
|---|---|
| Tare both sensors | `t\n` |
| Set angle of attack | `A<float>\n` e.g. `A12.50\n` |

---

## 🧪 Demo Mode

No hardware? Click **◈ DEMO MODE** to see simulated lift/drag data with a sweeping AoA from −10° → +15°, including realistic sensor noise and occasional spike artifacts (to test the filters).

---

## 📊 Filters

| Filter | Best for |
|---|---|
| **Median** ★ | HX711 zero-spike dropout removal (default) |
| **Moving Average** | General smoothing |
| **EMA** | Fast response, tunable with alpha slider |
| **Savitzky-Golay** | Preserving peak shape (browser approx) |
| **None** | Raw signal |

---

## 📁 File Structure

```
index.html   ← entire app, single self-contained file
README.md    ← this file
```

No build step. No dependencies to install. Everything is loaded from CDNs.

---

## ⚡ vs Original PyQt5 Version

| | PyQt5 (`wind_tunnel_gui.py`) | Web (`index.html`) |
|---|---|---|
| Installation | `pip install pyqt5 pyqtgraph pyserial numpy scipy` | None |
| Serial | `pyserial` | Web Serial API (Chrome/Edge) |
| Savitzky-Golay | Full `scipy` implementation | Polynomial approximation |
| Offline use | ✅ Full | ✅ (Demo mode; Serial needs HTTPS) |
| Shareable URL | ❌ | ✅ GitHub Pages |

---

## 📜 License

MIT — do whatever you want, attribution appreciated.
