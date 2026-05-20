#!/usr/bin/env python3
import sys
import json
import os
import threading
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtCore import QObject, pyqtSignal
from .timerGUI import Ui_MainWindow

import rclpy
from rclpy.node import Node
from auv_interfaces.msg import MultiPID, SetPoint, PID
from std_msgs.msg import String, Float32


# ─────────────────────────────────────────────────────────────────────────────
# Path file sesi — disimpan di folder yang sama dengan script ini
# ─────────────────────────────────────────────────────────────────────────────
SESSION_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "last_session_timer.json"
)


# ─────────────────────────────────────────────────────────────────────────────
# Helper: Simpan semua input UI ke JSON
# ─────────────────────────────────────────────────────────────────────────────
def save_session(ui):
    """
    Menyimpan delay + 18 baris (status, timer, yaw, depth) ke file JSON.
    Dipanggil saat tombol START ditekan.
    """
    data = {
        "delay": ui.delayInput.text().strip(),
        "rows": []
    }

    for i in range(1, 19):
        cb_name = 'comboBox' if i == 1 else f'comboBox_{i}'
        cb  = getattr(ui, cb_name, None)
        tmr = getattr(ui, f'timer{i}', None)
        yaw = getattr(ui, f'setPointYaw{i}', None)
        dep = getattr(ui, f'setPointDepth{i}', None)

        data["rows"].append({
            "status": cb.currentText().strip()  if cb  else "",
            "timer":  tmr.text().strip()        if tmr else "",
            "yaw":    yaw.text().strip()        if yaw else "",
            "depth":  dep.text().strip()        if dep else "",
        })

    try:
        with open(SESSION_FILE, "w") as f:
            json.dump(data, f, indent=2)
        print(f"[TimerGUI] 💾 Sesi disimpan ke: {SESSION_FILE}")
    except Exception as e:
        print(f"[TimerGUI] Gagal menyimpan sesi: {e}")


# ─────────────────────────────────────────────────────────────────────────────
# Helper: Load sesi dari JSON ke UI
# ─────────────────────────────────────────────────────────────────────────────
def load_session(ui):
    """
    Memuat data sesi dari file JSON dan mengisi kembali seluruh input UI.
    Dipanggil otomatis saat GUI pertama kali dibuka.
    Jika file belum ada, diam saja (UI tetap kosong/default).
    """
    if not os.path.exists(SESSION_FILE):
        print("[TimerGUI] Belum ada sesi tersimpan, mulai dari kosong.")
        return

    try:
        with open(SESSION_FILE, "r") as f:
            data = json.load(f)
    except Exception as e:
        print(f"[TimerGUI] Gagal memuat sesi: {e}")
        return

    # Isi delay
    ui.delayInput.setText(data.get("delay", ""))

    # Isi 18 baris
    for i, row in enumerate(data.get("rows", [])[:18], start=1):
        cb_name = 'comboBox' if i == 1 else f'comboBox_{i}'
        cb  = getattr(ui, cb_name, None)
        tmr = getattr(ui, f'timer{i}', None)
        yaw = getattr(ui, f'setPointYaw{i}', None)
        dep = getattr(ui, f'setPointDepth{i}', None)

        if cb is not None:
            status_text = row.get("status", "")
            idx = cb.findText(status_text)
            if idx >= 0:
                cb.setCurrentIndex(idx)
            else:
                cb.setCurrentText(status_text)  # custom text (editable combobox)

        if tmr is not None:
            tmr.setText(row.get("timer", ""))
        if yaw is not None:
            yaw.setText(row.get("yaw", ""))
        if dep is not None:
            dep.setText(row.get("depth", ""))

    print(f"[TimerGUI] ✅ Sesi berhasil dimuat dari: {SESSION_FILE}")


# ─────────────────────────────────────────────────────────────────────────────
# Signal bridge: dipakai untuk mengirim perintah update GUI dari ROS thread
# ke Qt main thread dengan aman.
# ─────────────────────────────────────────────────────────────────────────────
class GuiBridge(QObject):
    """
    Semua signal di sini di-emit dari ROS thread,
    tapi di-connect ke slot yang berjalan di main thread Qt.
    """
    update_display_signal  = pyqtSignal(int, bool)   # (active_idx, all_done)
    reset_display_signal   = pyqtSignal()


# ─────────────────────────────────────────────────────────────────────────────
# Helper: parse semua row dari UI → list of steps
# ─────────────────────────────────────────────────────────────────────────────
def parse_steps(ui):
    rows = []
    for i in range(1, 19):
        cb_name = 'comboBox' if i == 1 else f'comboBox_{i}'
        cb  = getattr(ui, cb_name, None)
        tmr = getattr(ui, f'timer{i}', None)
        yaw = getattr(ui, f'setPointYaw{i}', None)
        dep = getattr(ui, f'setPointDepth{i}', None)

        rows.append({
            'status': cb.currentText().strip()  if cb  else "",
            'timer':  tmr.text().strip()        if tmr else "",
            'yaw':    yaw.text().strip()        if yaw else "",
            'depth':  dep.text().strip()        if dep else "",
        })

    steps      = []
    cumulative = 0.0
    prev_yaw   = 0.0
    prev_depth = -0.11

    for r in rows:
        if r['yaw']:
            try:   prev_yaw   = float(r['yaw'])
            except ValueError: pass
        if r['depth']:
            try:   prev_depth = float(r['depth'])
            except ValueError: pass

        if not r['timer']:
            continue
        try:
            duration = float(r['timer'])
        except ValueError:
            continue
        if duration <= 0:
            continue

        steps.append({
            'start':  cumulative,
            'end':    cumulative + duration,
            'status': r['status'] if r['status'] else 'stop',
            'yaw':    prev_yaw,
            'depth':  prev_depth,
        })

        cumulative += duration

    return steps, cumulative


# ─────────────────────────────────────────────────────────────────────────────
# ROS2 Node
# ─────────────────────────────────────────────────────────────────────────────
class GuidanceGUI(Node):

    def __init__(self, ui, bridge: GuiBridge):
        super().__init__('gui_timer')
        self.ui     = ui
        self.bridge = bridge

        # ── State ─────────────────────────────────────────────────────────
        self.is_running         = False
        self.is_started         = False
        self.start_time         = 0.0
        self.boot_time          = 0.0
        self.param_delay        = 3.0
        self.steps              = []
        self.total_duration     = 0.0
        self.has_published_stop = False
        self.last_active_idx    = -2

        self.multi_pid_msg     = None
        self.default_roll      = 0.0
        self.default_pitch     = 0.0
        self._drop_stop_timer  = None

        # ── Publishers ────────────────────────────────────────────────────
        self.pub_set_point = self.create_publisher(SetPoint, 'set_point', 10)
        self.pub_status    = self.create_publisher(String,   'status',    10)
        self.pub_boost     = self.create_publisher(Float32,  'boost',     10)

        # ── Subscribers ───────────────────────────────────────────────────
        self.sub_pid = self.create_subscription(
            MultiPID, 'pid', self.pid_callback, 10
        )
        self.sub_drop_ball = self.create_subscription(
            Float32, 'drop_ball_msg', self.drop_ball_callback, 10
        )

        # ── ROS Timer 10 Hz ───────────────────────────────────────────────
        self.ros_timer = self.create_timer(0.1, self.loop)

        # ── Tombol START ──────────────────────────────────────────────────
        self.ui.startButton.clicked.connect(
            self.on_start_clicked, QtCore.Qt.QueuedConnection
        )

        # ── Auto-load sesi terakhir saat GUI dibuka ───────────────────────
        load_session(self.ui)

        self.get_logger().info("GuidanceGUI Node Started")

    # ─────────────────────────────────────────────────────────────────────────
    def pid_callback(self, msg):
        self.multi_pid_msg = msg

    def drop_ball_callback(self, msg):
        if msg.data >= 1.0 and self.is_running and not self.has_published_stop:
            current_yaw = (
                self.steps[self.last_active_idx]['yaw']
                if 0 <= self.last_active_idx < len(self.steps)
                else 0.0
            )
            self._publish_setpoint({'yaw': current_yaw, 'depth': -1})
            self._publish_status('dpr_ssy')
            self.has_published_stop = True
            self.is_running = False
            self.bridge.update_display_signal.emit(-1, True)
            self._drop_stop_timer = self.create_timer(5.0, self._drop_ball_then_stop)

    def _drop_ball_then_stop(self):
        self._publish_status('dpr_ssy')
        self.get_logger().info("drop ball → STOP (permanent)")
        if self._drop_stop_timer is not None:
            self._drop_stop_timer.cancel()
            self._drop_stop_timer = None

    # ─────────────────────────────────────────────────────────────────────────
    # on_start_clicked berjalan di main thread (dipanggil via clicked signal).
    # ─────────────────────────────────────────────────────────────────────────
    def on_start_clicked(self):
        # ── Simpan sesi sebelum memproses apapun ─────────────────────────
        save_session(self.ui)

        delay_text = self.ui.delayInput.text().strip()
        try:
            self.param_delay = float(delay_text) if delay_text else 3.0
        except ValueError:
            self.param_delay = 3.0

        self.steps, self.total_duration = parse_steps(self.ui)

        if not self.steps:
            self.get_logger().warn("Tidak ada step valid! Pastikan kolom timer terisi.")
            return

        self.is_started         = False
        self.is_running         = True
        self.has_published_stop = False
        self.last_active_idx    = -2
        self.start_time         = 0.0

        self._reset_status_display()

        self.get_logger().info(
            f"▶ START | delay={self.param_delay}s | "
            f"steps={len(self.steps)} | total={self.total_duration}s"
        )
        for i, s in enumerate(self.steps):
            self.get_logger().info(
                f"  Step {i+1}: [{s['start']:.1f}s – {s['end']:.1f}s] "
                f"status={s['status']} yaw={s['yaw']} depth={s['depth']}"
            )

    # ─────────────────────────────────────────────────────────────────────────
    def loop(self):
        """Dipanggil dari ROS timer thread. TIDAK boleh sentuh widget Qt langsung."""
        if not self.is_running:
            return

        if not self.is_started:
            self.start_time = self.get_clock().now().nanoseconds / 1e9
            self.is_started = True

        self.boot_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time

        # ── Fase delay ────────────────────────────────────────────────────
        if self.boot_time < self.param_delay:
            remaining = self.param_delay - self.boot_time
            self.get_logger().info(f'DELAY... sisa {remaining:.1f}s')
            return

        effective_time = self.boot_time - self.param_delay

        # ── Semua step selesai ────────────────────────────────────────────
        if effective_time >= self.total_duration:
            if not self.has_published_stop:
                self._publish_status('stop')
                self.has_published_stop = True
                self.bridge.update_display_signal.emit(-1, True)
                self.get_logger().info(
                    f"✓ Semua step selesai (t_eff={effective_time:.1f}s) → STOP"
                )
            return

        # ── Cari step aktif ───────────────────────────────────────────────
        active_step = None
        active_idx  = -1
        for i, step in enumerate(self.steps):
            if step['start'] <= effective_time < step['end']:
                active_step = step
                active_idx  = i
                break

        if active_step is None:
            return

        self._publish_setpoint(active_step)
        self._publish_status(active_step['status'])

        if active_idx != self.last_active_idx:
            self.last_active_idx = active_idx
            self.bridge.update_display_signal.emit(active_idx, False)
            sisa = active_step['end'] - effective_time
            self.get_logger().info(
                f"▶ Step {active_idx+1}/{len(self.steps)}: "
                f"status='{active_step['status']}' | "
                f"range=[{active_step['start']:.1f}–{active_step['end']:.1f}s] | "
                f"sisa≈{sisa:.1f}s | "
                f"yaw={active_step['yaw']} depth={active_step['depth']}"
            )

    # ─────────────────────────────────────────────────────────────────────────
    def _publish_setpoint(self, step):
        sp       = SetPoint()
        sp.yaw   = float(step['yaw'])
        sp.depth = float(step['depth'])
        sp.pitch = self.default_pitch
        sp.roll  = self.default_roll
        self.pub_set_point.publish(sp)

    def _publish_status(self, status_str: str):
        sm      = String()
        sm.data = status_str
        self.pub_status.publish(sm)

    # ─────────────────────────────────────────────────────────────────────────
    # Fungsi di bawah dipanggil HANYA dari main thread Qt (via signal/slot).
    # ─────────────────────────────────────────────────────────────────────────
    def _update_display(self, active_idx: int, all_done: bool = False):
        n_steps = len(self.steps)
        for i in range(1, 19):
            w = getattr(self.ui, f'status{i}', None)
            if w is None:
                continue

            row_idx = i - 1

            if row_idx >= n_steps:
                w.setStyleSheet("color: #555555;")
                w.setText("--")
                continue

            if all_done:
                w.setStyleSheet(
                    "background: #1A3A5C; color: #7AB3E0; font-weight: bold;"
                )
                w.setText("✓ DONE")

            elif row_idx < active_idx:
                w.setStyleSheet(
                    "background: #1A3A5C; color: #7AB3E0; font-weight: bold;"
                )
                w.setText("✓ DONE")

            elif row_idx == active_idx:
                w.setStyleSheet(
                    "background: #00AA44; color: white; font-weight: bold;"
                )
                w.setText("▶ ACTIVE")

            else:
                w.setStyleSheet(
                    "background: #2E2E2E; color: #888888; font-weight: bold;"
                )
                w.setText("● PENDING")

    def _reset_status_display(self):
        """Dipanggil langsung dari on_start_clicked (main thread) → aman."""
        n_steps = len(self.steps)
        for i in range(1, 19):
            w = getattr(self.ui, f'status{i}', None)
            if w is None:
                continue
            if i - 1 < n_steps:
                w.setStyleSheet(
                    "background: #2E2E2E; color: #888888; font-weight: bold;"
                )
                w.setText("● PENDING")
            else:
                w.setStyleSheet("color: #555555;")
                w.setText("--")


# ─────────────────────────────────────────────────────────────────────────────
def ros_spin(node):
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    app        = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui         = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()

    bridge = GuiBridge()
    node   = GuidanceGUI(ui, bridge)

    bridge.update_display_signal.connect(
        node._update_display, QtCore.Qt.QueuedConnection
    )

    ros_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    ros_thread.start()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()