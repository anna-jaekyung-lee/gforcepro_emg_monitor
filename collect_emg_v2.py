#!/usr/bin/env python3
# collect_emg_v2.py — gForcePRO EMG Collector with Real-Time Graph
# Requirements: pip install bleak scipy numpy pyqtgraph PyQt5

import asyncio
import struct
import time
import os
import threading
import numpy as np
from datetime import datetime
from collections import deque

from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg

try:
    from scipy.signal import butter, sosfilt, sosfilt_zi, iirnotch, tf2sos
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False

from gforce import GForceProfile, DataNotifFlags, NotifDataType, ResponseResult

# ── Settings ──────────────────────────────────────────────────────────────
SAVE_DIR         = "."
N_CH             = 8
GRAPH_WINDOW_SEC = 5      # seconds shown in graph
GRAPH_UPDATE_MS  = 50     # graph refresh interval (ms) → 20fps
GRAPH_DOWNSAMPLE = 2      # downsample for display (1000Hz → 500Hz)
# ──────────────────────────────────────────────────────────────────────────

pg.setConfigOptions(antialias=False, useOpenGL=False)

COLORS = [
    '#4af0a0', '#4a9fff', '#f09060', '#f06060',
    '#a0d8f0', '#f0c060', '#c0a0f0', '#60d0a0'
]

class DataRecorder:
    def __init__(self, save_dir=SAVE_DIR, auto_save_sec=0, sample_rate=1000):
        self.save_dir      = save_dir
        self.auto_save_sec = auto_save_sec
        self.split_samples = int(auto_save_sec * sample_rate) if auto_save_sec > 0 else 0
        self.file          = None
        self.lock          = threading.Lock()
        self.row_count     = 0
        self.current_path  = None

    def _new_file(self):
        if self.file:
            self.file.close()
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.current_path = os.path.join(self.save_dir, f"emg_{ts}.csv")
        self.file = open(self.current_path, "w", newline="")
        header = "timestamp_s," + ",".join(f"CH{i+1}" for i in range(N_CH)) + "\n"
        self.file.write(header)
        self.row_count = 0

    def start(self):
        self._new_file()

    def write(self, t, row):
        if not self.file:
            return
        if self.split_samples > 0 and self.row_count > 0 and self.row_count % self.split_samples == 0:
            self._new_file()
        with self.lock:
            self.file.write(f"{t:.6f}," + ",".join(str(v) for v in row) + "\n")
            self.row_count += 1

    def stop(self):
        if self.file:
            self.file.close()
            self.file = None


class MainWindow(QtWidgets.QMainWindow):
    log_signal    = QtCore.pyqtSignal(str)
    status_signal = QtCore.pyqtSignal(str, str)  # text, color

    def __init__(self):
        super().__init__()
        self.setWindowTitle("gForcePRO EMG Collector v2")
        self.setMinimumSize(1200, 800)
        self._apply_theme()

        self.gforce  = GForceProfile()
        self.loop    = asyncio.new_event_loop()
        threading.Thread(
            target=lambda: (asyncio.set_event_loop(self.loop), self.loop.run_forever()),
            daemon=True).start()

        self.recording      = False
        self.connected      = False
        self.scan_results   = []
        self.start_time     = None
        self.sample_counter = [0]
        self.recorder       = None
        self.fs             = 1000
        self.resolution     = 8

        # Graph ring buffers
        maxlen = GRAPH_WINDOW_SEC * 1000 // GRAPH_DOWNSAMPLE
        self.graph_t   = deque(maxlen=maxlen)
        self.graph_buf = [deque(maxlen=maxlen) for _ in range(N_CH)]
        self.buf_lock  = threading.Lock()

        self.log_signal.connect(self._append_log)
        self.status_signal.connect(self._set_status)

        self._build_ui()

        # Graph timer
        self.graph_timer = QtCore.QTimer()
        self.graph_timer.timeout.connect(self._update_graph)
        self.graph_timer.start(GRAPH_UPDATE_MS)

        # Clock timer
        self.clock_timer = QtCore.QTimer()
        self.clock_timer.timeout.connect(self._update_clock)
        self.clock_timer.start(100)

    def _apply_theme(self):
        self.setStyleSheet("""
            QMainWindow, QWidget { background: #0d1117; color: #e0e0e0; }
            QLabel { color: #e0e0e0; }
            QLineEdit, QComboBox {
                background: #161b22; border: 1px solid #30363d;
                border-radius: 4px; padding: 4px 8px; color: #e0e0e0;
            }
            QComboBox::drop-down { border: none; }
            QComboBox::down-arrow { image: none; }
            QPushButton {
                background: #21262d; border: 1px solid #30363d;
                border-radius: 4px; padding: 6px 16px; color: #e0e0e0;
                font-weight: bold;
            }
            QPushButton:hover { background: #30363d; border-color: #58a6ff; }
            QPushButton:disabled { color: #484f58; border-color: #21262d; }
            QGroupBox {
                border: 1px solid #30363d; border-radius: 6px;
                margin-top: 8px; padding-top: 8px;
                font-weight: bold; color: #58a6ff;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }
            QListWidget {
                background: #161b22; border: 1px solid #30363d;
                border-radius: 4px; color: #e0e0e0;
            }
            QListWidget::item:selected { background: #1f6feb; }
            QTextEdit {
                background: #0d1117; border: 1px solid #30363d;
                border-radius: 4px; color: #7d8590; font-family: monospace;
                font-size: 11px;
            }
            QCheckBox { color: #e0e0e0; spacing: 6px; }
            QCheckBox::indicator {
                width: 14px; height: 14px;
                border: 1px solid #30363d; border-radius: 3px;
                background: #161b22;
            }
            QCheckBox::indicator:checked { background: #238636; border-color: #2ea043; }
            QSplitter::handle { background: #30363d; }
        """)

    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        main_layout = QtWidgets.QHBoxLayout(central)
        main_layout.setContentsMargins(8, 8, 8, 8)
        main_layout.setSpacing(8)

        # ── LEFT PANEL ─────────────────────────────────────────────────
        left = QtWidgets.QWidget()
        left.setFixedWidth(320)
        lv = QtWidgets.QVBoxLayout(left)
        lv.setSpacing(8)

        # Status
        status_box = QtWidgets.QGroupBox("Status")
        sv = QtWidgets.QVBoxLayout(status_box)
        self.lbl_conn  = QtWidgets.QLabel("● Not connected")
        self.lbl_conn.setStyleSheet("color: #484f58; font-weight: bold; font-size: 13px;")
        self.lbl_timer = QtWidgets.QLabel("⏱  00:00:00.000")
        self.lbl_timer.setStyleSheet("font-size: 18px; font-weight: bold; font-family: monospace; color: #58a6ff;")
        self.lbl_rate  = QtWidgets.QLabel("Sample rate: —")
        self.lbl_rate.setStyleSheet("color: #7d8590; font-size: 11px;")
        self.lbl_file  = QtWidgets.QLabel("Output file: —")
        self.lbl_file.setStyleSheet("color: #7d8590; font-size: 10px;")
        self.lbl_file.setWordWrap(True)
        sv.addWidget(self.lbl_conn)
        sv.addWidget(self.lbl_timer)
        sv.addWidget(self.lbl_rate)
        sv.addWidget(self.lbl_file)
        lv.addWidget(status_box)

        # Settings
        cfg_box = QtWidgets.QGroupBox("Recording Settings")
        cv = QtWidgets.QFormLayout(cfg_box)
        cv.setSpacing(6)

        self.var_samprate   = QtWidgets.QComboBox()
        self.var_samprate.addItems(["500", "650", "1000"])
        self.var_samprate.setCurrentText("1000")

        self.var_resolution = QtWidgets.QComboBox()
        self.var_resolution.addItems(["8", "12"])

        self.var_autosave   = QtWidgets.QLineEdit("0")
        self.var_autosave.setPlaceholderText("0 = off")

        self.var_savedir    = QtWidgets.QLineEdit(os.path.abspath(SAVE_DIR))
        browse_btn = QtWidgets.QPushButton("Browse")
        browse_btn.clicked.connect(self._browse)
        dir_row = QtWidgets.QHBoxLayout()
        dir_row.addWidget(self.var_savedir)
        dir_row.addWidget(browse_btn)

        cv.addRow("Sample Rate (Hz)", self.var_samprate)
        cv.addRow("Resolution (bit)", self.var_resolution)
        cv.addRow("Auto-save (sec)", self.var_autosave)
        cv.addRow("Save Folder", dir_row)

        note = QtWidgets.QLabel("※ 1000Hz+12bit is experimental")
        note.setStyleSheet("color: #484f58; font-size: 10px;")
        cv.addRow(note)
        lv.addWidget(cfg_box)

        # Graph settings
        graph_box = QtWidgets.QGroupBox("Graph Settings")
        gv = QtWidgets.QFormLayout(graph_box)
        gv.setSpacing(6)
        self.var_win_sec = QtWidgets.QLineEdit(str(GRAPH_WINDOW_SEC))
        self.var_win_sec.setPlaceholderText("seconds")
        gv.addRow("Window (sec)", self.var_win_sec)
        lv.addWidget(graph_box)

        # Buttons
        btn_box = QtWidgets.QGroupBox("Control")
        bv = QtWidgets.QGridLayout(btn_box)

        self.btn_scan    = QtWidgets.QPushButton("🔍  Scan")
        self.btn_connect = QtWidgets.QPushButton("🔗  Connect")
        self.btn_start   = QtWidgets.QPushButton("▶  Start")
        self.btn_stop    = QtWidgets.QPushButton("■  Stop")

        self.btn_connect.setEnabled(False)
        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(False)

        self.btn_start.setStyleSheet("background: #1a4a1a; border-color: #2ea043; color: #2ea043;")
        self.btn_stop.setStyleSheet("background: #4a1a1a; border-color: #f85149; color: #f85149;")

        self.btn_scan.clicked.connect(self._on_scan)
        self.btn_connect.clicked.connect(self._on_connect)
        self.btn_start.clicked.connect(self._on_start)
        self.btn_stop.clicked.connect(self._on_stop)

        bv.addWidget(self.btn_scan,    0, 0)
        bv.addWidget(self.btn_connect, 0, 1)
        bv.addWidget(self.btn_start,   1, 0)
        bv.addWidget(self.btn_stop,    1, 1)
        lv.addWidget(btn_box)

        # Device list
        dev_box = QtWidgets.QGroupBox("Detected Devices")
        dv = QtWidgets.QVBoxLayout(dev_box)
        self.listbox = QtWidgets.QListWidget()
        self.listbox.setMaximumHeight(80)
        dv.addWidget(self.listbox)
        lv.addWidget(dev_box)

        # Log
        log_box = QtWidgets.QGroupBox("Log")
        logv = QtWidgets.QVBoxLayout(log_box)
        self.log_text = QtWidgets.QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(150)
        logv.addWidget(self.log_text)
        lv.addWidget(log_box)
        lv.addStretch()

        main_layout.addWidget(left)

        # ── RIGHT PANEL: GRAPHS ────────────────────────────────────────
        graph_widget = pg.GraphicsLayoutWidget()
        graph_widget.setBackground('#0d1117')

        self.plots = []
        self.curves = []

        for i in range(N_CH):
            p = graph_widget.addPlot(row=i, col=0)
            p.setLabel('left', f'CH{i+1}', color='#7d8590', size='9pt')
            p.getAxis('left').setStyle(tickFont=QtGui.QFont('monospace', 7))
            p.getAxis('left').setTextPen(pg.mkPen('#7d8590'))
            p.getAxis('bottom').setTextPen(pg.mkPen('#7d8590'))
            p.setMenuEnabled(False)
            p.hideButtons()
            p.showGrid(x=False, y=True, alpha=0.15)
            p.getAxis('bottom').setStyle(showValues=(i == N_CH - 1))
            if i < N_CH - 1:
                p.getAxis('bottom').setHeight(0)
            else:
                p.setLabel('bottom', 'Time (s)', color='#7d8590', size='9pt')

            # zero line
            zero = pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen('#30363d', width=1, style=QtCore.Qt.DashLine))
            p.addItem(zero)

            color = COLORS[i]
            curve = p.plot(pen=pg.mkPen(color, width=1))
            self.plots.append(p)
            self.curves.append(curve)

        main_layout.addWidget(graph_widget, stretch=1)

    # ── Helpers ───────────────────────────────────────────────────────
    def _log(self, msg):
        ts = datetime.now().strftime('%H:%M:%S')
        self.log_signal.emit(f"[{ts}] {msg}")

    @QtCore.pyqtSlot(str)
    def _append_log(self, msg):
        self.log_text.append(msg)
        self.log_text.verticalScrollBar().setValue(
            self.log_text.verticalScrollBar().maximum())

    @QtCore.pyqtSlot(str, str)
    def _set_status(self, text, color):
        self.lbl_conn.setText(text)
        self.lbl_conn.setStyleSheet(f"color: {color}; font-weight: bold; font-size: 13px;")

    def _browse(self):
        d = QtWidgets.QFileDialog.getExistingDirectory(self, "Select Save Folder")
        if d:
            self.var_savedir.setText(d)

    def _submit(self, coro):
        return asyncio.run_coroutine_threadsafe(coro, self.loop)

    # ── Clock ─────────────────────────────────────────────────────────
    def _update_clock(self):
        if self.recording and self.start_time is not None:
            elapsed = time.perf_counter() - self.start_time
            h = int(elapsed // 3600)
            m = int((elapsed % 3600) // 60)
            s = int(elapsed % 60)
            ms = int((elapsed % 1) * 1000)
            self.lbl_timer.setText(f"⏱  {h:02d}:{m:02d}:{s:02d}.{ms:03d}")

    # ── Graph ─────────────────────────────────────────────────────────
    def _update_graph(self):
        if not self.recording:
            return
        with self.buf_lock:
            if len(self.graph_t) < 2:
                return
            t_arr = np.array(self.graph_t)
            bufs  = [np.array(b) for b in self.graph_buf]

        t_rel = t_arr - t_arr[-1] + float(self.var_win_sec.text() or GRAPH_WINDOW_SEC)

        for i, (plot, curve) in enumerate(zip(self.plots, self.curves)):
            y = bufs[i]
            curve.setData(t_rel, y)
            if len(y) > 0:
                ymax = max(abs(y).max(), 10) * 1.2
                plot.setYRange(-ymax, ymax, padding=0)
            plot.setXRange(0, float(self.var_win_sec.text() or GRAPH_WINDOW_SEC), padding=0)

    # ── Scan ──────────────────────────────────────────────────────────
    def _on_scan(self):
        self.btn_scan.setEnabled(False)
        self.btn_scan.setText("Scanning...")
        self.listbox.clear()
        self._log("Scanning BLE (5s)...")
        self._submit(self._do_scan())

    async def _do_scan(self):
        try:
            results = await self.gforce.scan(5, "gForce")
            self.scan_results = results
            def _u():
                self.listbox.clear()
                if not results:
                    self.listbox.addItem("No devices found.")
                    self._log("No devices found.")
                else:
                    for d in results:
                        self.listbox.addItem(f"{d['name']}  |  {d['address']}  |  RSSI={d['rssi']}dB")
                    self._log(f"{len(results)} device(s) found.")
                self.btn_scan.setEnabled(True)
                self.btn_scan.setText("🔍  Scan")
                self.btn_connect.setEnabled(bool(results))
            QtCore.QMetaObject.invokeMethod(self, "_run_in_main", QtCore.Qt.QueuedConnection,
                                            QtCore.Q_ARG(object, _u))
        except Exception as e:
            self._log(f"Scan error: {e}")
            QtCore.QMetaObject.invokeMethod(self, "_run_in_main", QtCore.Qt.QueuedConnection,
                QtCore.Q_ARG(object, lambda: (self.btn_scan.setEnabled(True),
                                               self.btn_scan.setText("🔍  Scan"))))

    # ── Connect ───────────────────────────────────────────────────────
    def _on_connect(self):
        sel = self.listbox.currentRow()
        if sel < 0 or sel >= len(self.scan_results):
            return
        addr = self.scan_results[sel]["address"]
        self._log(f"Connecting to: {addr}")
        self.btn_connect.setEnabled(False)
        self.btn_connect.setText("Connecting...")
        self._submit(self._do_connect(addr))

    async def _do_connect(self, addr):
        try:
            await self.gforce.connect(addr)
            self.connected = True
            def _u():
                self.status_signal.emit("● Connected", "#2ea043")
                self.btn_connect.setText("🔗  Connected")
                self.btn_start.setEnabled(True)
            QtCore.QMetaObject.invokeMethod(self, "_run_in_main", QtCore.Qt.QueuedConnection,
                                            QtCore.Q_ARG(object, _u))
            self._log("Connected!")
        except Exception as e:
            self._log(f"Connection failed: {e}")
            QtCore.QMetaObject.invokeMethod(self, "_run_in_main", QtCore.Qt.QueuedConnection,
                QtCore.Q_ARG(object, lambda: (self.btn_connect.setEnabled(True),
                                               self.btn_connect.setText("🔗  Connect"))))

    # ── Start ─────────────────────────────────────────────────────────
    def _on_start(self):
        self.fs         = int(self.var_samprate.currentText())
        self.resolution = int(self.var_resolution.currentText())
        auto_sec        = int(self.var_autosave.text() or 0)

        self.recorder = DataRecorder(
            save_dir=self.var_savedir.text(),
            auto_save_sec=auto_sec,
            sample_rate=self.fs)
        self.recorder.start()

        self.recording      = True
        self.start_time     = None
        self.sample_counter = [0]
        self._rate_count    = 0
        self._rate_time     = None

        # Clear graph buffers
        with self.buf_lock:
            self.graph_t.clear()
            for b in self.graph_buf:
                b.clear()
            # Resize if window changed
            try:
                win = int(float(self.var_win_sec.text()) * 1000 // GRAPH_DOWNSAMPLE)
                self.graph_t = deque(maxlen=win)
                self.graph_buf = [deque(maxlen=win) for _ in range(N_CH)]
            except:
                pass

        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self._log(f"Recording — {self.fs}Hz / {self.resolution}bit / auto-save={auto_sec}s")
        self._submit(self._do_start())

    async def _do_start(self):
        def emg_cb(resp):
            codes = {0:"SUCCESS",1:"NOT_SUPPORT",2:"BAD_PARAM",3:"FAILED",4:"TIMEOUT"}
            self._log(f"EMG config: {codes.get(resp, resp)}")

        await self.gforce.setEmgRawDataConfig(
            sampRate=self.fs, channelMask=0xFF, dataLen=128,
            resolution=self.resolution, cb=emg_cb, timeout=1000)
        await asyncio.sleep(0.3)

        await self.gforce.setDataNotifSwitch(
            DataNotifFlags.DNF_EMG_RAW,
            lambda r: self._log(f"DataNotifSwitch: {r}"), 1000)
        await asyncio.sleep(0.3)
        await self.gforce.startDataNotification(self._on_data)

    # ── Data callback ─────────────────────────────────────────────────
    def _on_data(self, data):
        if not self.recording:
            return
        if data[0] != NotifDataType.NTF_EMG_ADC_DATA:
            return

        now = time.perf_counter()
        if self.start_time is None:
            self.start_time  = now
            self._rate_time  = now
            self._rate_count = 0

        elapsed = now - self.start_time

        if self.resolution == 8:
            if len(data) != 129:
                return
            raw = np.array(data[1:], dtype=np.uint8).reshape(-1, N_CH).astype(np.float32)
            n = 16
        else:
            if len(data) != 129:
                return
            rb = data[1:]
            n  = len(rb) // (2 * N_CH)
            raw = np.zeros((n, N_CH), dtype=np.float32)
            for s in range(n):
                for ch in range(N_CH):
                    idx = (s * N_CH + ch) * 2
                    u = (rb[idx] | (rb[idx+1] << 8)) & 0x0FFF
                    raw[s, ch] = u - 2048

        dt = 1.0 / self.fs

        # Save
        for s in range(n):
            t = (self.sample_counter[0] + s) * dt
            self.recorder.write(t, [int(v) for v in raw[s]])
        self.sample_counter[0] += n

        # Graph buffer (downsampled)
        with self.buf_lock:
            for s in range(0, n, GRAPH_DOWNSAMPLE):
                t_s = elapsed - (n - 1 - s) * dt
                self.graph_t.append(t_s)
                for ch in range(N_CH):
                    self.graph_buf[ch].append(raw[s, ch])

        # Sample rate display
        self._rate_count += n
        if self._rate_time is not None and (now - self._rate_time) >= 1.0:
            rate = self._rate_count / (now - self._rate_time)
            fname = os.path.basename(self.recorder.current_path) if self.recorder and self.recorder.current_path else "—"
            def _u(r=rate, f=fname):
                self.lbl_rate.setText(f"Sample rate: {r:.1f} Hz")
                self.lbl_file.setText(f"Output: {f}")
            QtCore.QMetaObject.invokeMethod(self, "_run_in_main", QtCore.Qt.QueuedConnection,
                                            QtCore.Q_ARG(object, _u))
            self._rate_count = 0
            self._rate_time  = now

    # ── Stop ──────────────────────────────────────────────────────────
    def _on_stop(self):
        self.recording = False
        self.btn_stop.setEnabled(False)
        self.btn_start.setEnabled(True)
        self._log("Stopping...")
        self._submit(self._do_stop())

    async def _do_stop(self):
        await self.gforce.stopDataNotification()
        await asyncio.sleep(0.3)
        await self.gforce.setDataNotifSwitch(DataNotifFlags.DNF_OFF, lambda r: None, 1000)
        if self.recorder:
            self.recorder.stop()
            self._log(f"Saved: {self.recorder.current_path} ({self.recorder.row_count:,} rows)")
        self._log("Stopped.")

    # ── Thread-safe UI call ───────────────────────────────────────────
    @QtCore.pyqtSlot(object)
    def _run_in_main(self, fn):
        fn()


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName("gForcePRO EMG v2")
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())
