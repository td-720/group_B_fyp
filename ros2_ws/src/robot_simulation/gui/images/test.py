#!/usr/bin/env python3
import sys
import os
import subprocess
import signal
import json
import pandas as pd
import numpy as np

# --- ROS 2 IMPORTS ---
from ament_index_python.packages import get_package_share_directory

# --- GUI IMPORTS ---
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                             QComboBox, QPushButton, QTextEdit, QLabel, QMessageBox, 
                             QFileDialog, QDoubleSpinBox, QGroupBox, QLineEdit, 
                             QTableWidgetItem, QTableWidget, QHeaderView, QScrollArea,
                             QDialog, QDialogButtonBox, QFrame, QTextBrowser, QGraphicsOpacityEffect)
from PyQt5.QtGui import QFont, QPalette, QColor, QPixmap
from PyQt5.QtCore import Qt, pyqtSignal, QTimer, QPropertyAnimation, QEasingCurve

# --- MATPLOTLIB IMPORTS ---
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
from matplotlib.patches import Rectangle


# =====================================================================
# --- CONFIGURATION & CONSTANTS ---
# =====================================================================
try:
    PACKAGE_SHARE = get_package_share_directory("robot_simulation")
    IMAGE_DIR = os.path.join(PACKAGE_SHARE, "gui", "images")
except Exception as e:
    print(f"CRITICAL: Failed to locate package share directory. Error: {e}")
    IMAGE_DIR = ""

ROS2_SETUP_CMD = (
    "source /opt/ros/humble/setup.bash && "
    "source $(dirname $(dirname $(which ros2)))/../setup.bash"
)

DEFAULT_LOG_DIR = os.path.join(
    os.path.expanduser("~"),
    "simulation_logs"
)

# --- ROBOT STARTING POSITION (WEBOTS ABSOLUTE) ---
ROBOT_START_X = -2.5
ROBOT_START_Y = -2.5

# =====================================================================
# --- GLOBAL VISUAL THEME (clean modern light) ---
# =====================================================================
THEME_QSS = """
    QWidget { background-color: #F5F6F8; color: #1F2A37; font-family: 'Segoe UI', 'Helvetica Neue', Arial, sans-serif; font-size: 13px; }
    QLabel { font-weight: 600; color: #1F2A37; }
    QLabel#sectionTitle { font-size: 17px; font-weight: 700; color: #1F2A37; }
    QLabel#bannerTitle { font-size: 15px; font-weight: 700; background-color: #22304A; color: #FFFFFF; padding: 12px 14px; border-radius: 8px; }
    QLabel#groupSubtitle { font-size: 16px; font-weight: 700; color: #1F2A37; margin-bottom: 6px; }
    QLabel#mutedText { font-weight: 400; color: #6B7684; }
    QLabel#helpText { font-weight: 400; font-style: italic; color: #8A93A1; }

    QGroupBox { font-weight: 700; color: #1F2A37; background-color: #FFFFFF; border: 1px solid #DDE1E6; border-radius: 10px; margin-top: 14px; padding: 14px 10px 10px 10px; }
    QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; left: 10px; padding: 0 6px; color: #1F2A37; }

    QComboBox, QLineEdit, QDoubleSpinBox {
        background-color: #FFFFFF; color: #1F2A37; border: 1px solid #DDE1E6;
        border-radius: 6px; padding: 5px 8px; selection-background-color: #2F6FED;
    }
    QComboBox:focus, QLineEdit:focus, QDoubleSpinBox:focus { border: 1px solid #2F6FED; }
    QComboBox::drop-down { border: none; width: 22px; }
    QComboBox QAbstractItemView { background-color: #FFFFFF; color: #1F2A37; selection-background-color: #DCE7FC; border: 1px solid #DDE1E6; }

    QDoubleSpinBox { padding-right: 2px; }
    QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {
        width: 20px; background-color: #EEF3FE; border: 1px solid #C7D2E0; border-radius: 3px;
    }
    QDoubleSpinBox::up-button { subcontrol-position: top right; margin: 2px 2px 1px 0px; }
    QDoubleSpinBox::down-button { subcontrol-position: bottom right; margin: 1px 2px 2px 0px; }
    QDoubleSpinBox::up-button:hover, QDoubleSpinBox::down-button:hover { background-color: #DCE7FC; border: 1px solid #2F6FED; }
    QDoubleSpinBox::up-button:pressed, QDoubleSpinBox::down-button:pressed { background-color: #C7D9F8; }
    QDoubleSpinBox::up-arrow {
        width: 0; height: 0; border-left: 4px solid transparent; border-right: 4px solid transparent;
        border-bottom: 5px solid #2F6FED; margin-top: 1px;
    }
    QDoubleSpinBox::down-arrow {
        width: 0; height: 0; border-left: 4px solid transparent; border-right: 4px solid transparent;
        border-top: 5px solid #2F6FED; margin-bottom: 1px;
    }
    QDoubleSpinBox::up-arrow:disabled, QDoubleSpinBox::down-arrow:disabled { border-bottom-color: #A6ADB4; border-top-color: #A6ADB4; }

    QTextBrowser, QTextEdit#logBrowser { background-color: #FAFBFC; font-style: normal; color: #3B4552; border: 1px solid #DDE1E6; border-radius: 6px; }

    QScrollArea { border: none; background-color: transparent; }
    QScrollBar:vertical { background-color: #F5F6F8; width: 12px; margin: 0px; }
    QScrollBar::handle:vertical { background-color: #C7D2E0; border-radius: 5px; min-height: 24px; }
    QScrollBar::handle:vertical:hover { background-color: #AFC0D8; }
    QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0px; }

    QPushButton {
        background-color: #FFFFFF; color: #2F6FED; border: 1px solid #C7D2E0;
        border-radius: 6px; padding: 6px 12px; font-weight: 600;
    }
    QPushButton:hover { background-color: #EEF3FE; }
    QPushButton:pressed { background-color: #E1EAFB; }
    QPushButton:disabled { background-color: #F0F1F3; color: #A6ADB4; border: 1px solid #E4E7EB; }

    QPushButton#primaryAction {
        background-color: #2F6FED; color: #FFFFFF; border: none; font-weight: 700; min-height: 30px;
    }
    QPushButton#primaryAction:hover { background-color: #2A62D3; }
    QPushButton#primaryAction:pressed { background-color: #2555BA; }

    QPushButton#dangerAction {
        background-color: #D64545; color: #FFFFFF; border: none; font-weight: 700; min-height: 30px;
    }
    QPushButton#dangerAction:hover { background-color: #C13B3B; }
    QPushButton#dangerAction:pressed { background-color: #AC3333; }

    QPushButton#accentAction {
        background-color: #7C4DBB; color: #FFFFFF; border: none; font-weight: 700; min-height: 34px;
    }
    QPushButton#accentAction:hover { background-color: #6E43A8; }
    QPushButton#accentAction:disabled { background-color: #C9CDD3; color: #F2F3F5; border: none; }

    QTableWidget { background-color: #FAFBFC; color: #1F2A37; gridline-color: #E4E7EB; border: 1px solid #DDE1E6; border-radius: 6px; }
    QTableWidget::item { color: #1F2A37; }
    QHeaderView::section { background-color: #EEF0F3; color: #1F2A37; font-weight: 700; border: none; padding: 4px; }
"""


# =====================================================================
# --- CORE LOGIC MODULES ---
# =====================================================================

class ROS2ProcessManager:
    """Handles OS-level subprocess execution for ROS 2 commands."""
    
    def __init__(self):
        self.active_process = None
        self.background_processes = [] 

    def _sweep_background_processes(self):
        """Cleanly purge ONLY the background processes spawned by this GUI."""
        print("SWEEP: Terminating tracked background processes...")
        
        for proc in self.background_processes:
            if proc.poll() is None: 
                try:
                    proc.terminate()
                    proc.wait(timeout=1)
                except Exception as e:
                    print(f"WARNING: Failed to terminate background process: {e}")
                    proc.kill()
        
        self.background_processes.clear()

        try:
            global ROS2_SETUP_CMD
            subprocess.run(f"{ROS2_SETUP_CMD} && ros2 daemon stop", shell=True, executable='/bin/bash', stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print("SUCCESS: ROS 2 background discovery channels purged clean.\n")
        except Exception as e:
            print(f"WARNING: Daemon sweep encountered an issue: {e}")

    def is_running(self):
        if self.active_process is None:
            return False
            
        poll_status = self.active_process.poll()
        if poll_status is not None:
            print(f"PROCESS MONITOR: Core process terminated independently (exit code {poll_status}).")
            self.active_process = None
            self._sweep_background_processes()
            return False
            
        return True

    def launch(self, target_world, node, robot_name, waypoints_str="", kp=None, ki=None, kd=None):
        robot_arg = f"robot_name:={robot_name}" if robot_name.strip() else ""
        waypoint_arg = f" relative_waypoints:=\"{waypoints_str}\"" if waypoints_str else ""
        
        pid_args = f" kp:={kp} ki:={ki} kd:={kd}" if kp is not None else ""
        
        launch_args = f"world:={target_world} scenario:={node} {robot_arg}{waypoint_arg}{pid_args}".strip()
        full_cmd = f"{ROS2_SETUP_CMD} && ros2 launch robot_simulation simulation.launch.py {launch_args}"
        
        print(f"IGNITION SEQUENCE INITIATED:\n{full_cmd}")
        self.active_process = subprocess.Popen(
            full_cmd, shell=True, executable='/bin/bash', preexec_fn=os.setsid
        )

    def kill_all(self):
        if not self.is_running():
            return False
            
        print("\nTERMINATE: Initiating complete system sweep...")
        try:
            pgid = os.getpgid(self.active_process.pid)
            os.killpg(pgid, signal.SIGTERM)
            print(f"SUCCESS: Terminated core simulation group {pgid}")
        except ProcessLookupError:
            print("WARNING: Core simulation already down.")
        finally:
            self.active_process = None

        self._sweep_background_processes()
        return True

    def set_pid_params(self, node, robot_name, kp, ki, kd):
        global ROS2_SETUP_CMD
        target_node = f"{robot_name}/{node}" if robot_name.strip() else node
        target_node = target_node.strip('/')
        
        cmd = (f"({ROS2_SETUP_CMD} && ros2 param set /{target_node} kp {kp}) & "
               f"({ROS2_SETUP_CMD} && ros2 param set /{target_node} ki {ki}) & "
               f"({ROS2_SETUP_CMD} && ros2 param set /{target_node} kd {kd})")
               
        print(f"\n--- DISPATCHING ASYNC PID GAINS FOR /{target_node} ---")
        proc = subprocess.Popen(cmd, shell=True, executable='/bin/bash', stdout=subprocess.DEVNULL)
        self.background_processes.append(proc) 
        return proc

    def publish_polynomial(self, poly_type):
        global ROS2_SETUP_CMD
        raw_cmd = f"{ROS2_SETUP_CMD} && ros2 topic pub --once /set_polynomial std_msgs/msg/String \"{{data: '{poly_type}'}}\""
        proc = subprocess.Popen(raw_cmd, shell=True, executable='/bin/bash', stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.background_processes.append(proc) 

    def publish_waypoints(self, points):
        if not self.is_running(): return False
            
        global ROS2_SETUP_CMD
        
        relative_points = [[pt[0] - ROBOT_START_X, pt[1] - ROBOT_START_Y] for pt in points]
        rotated_points = [[round(pt[1], 3), round(-pt[0], 3)] for pt in relative_points]
        
        points_str = json.dumps(rotated_points)
        raw_cmd = f"{ROS2_SETUP_CMD} && ros2 topic pub --keep-alive 2 /gcs/via_points std_msgs/msg/String \"{{data: '{points_str}'}}\""
        proc = subprocess.Popen(raw_cmd, shell=True, executable='/bin/bash', stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.background_processes.append(proc) 
        return True
        
class SimulationAnalyzer:
    @staticmethod
    def calculate_wall_following_metrics(df):
        time, errors = df['Time'].values, df['Error'].values
        dt = np.mean(np.diff(time)) if len(time) > 1 else 0.1
        
        rmse = np.sqrt(np.mean(errors**2))
        iae = np.trapz(np.abs(errors), dx=dt)
        ise = np.trapz(errors**2, dx=dt)
        peak_err = np.max(np.abs(errors))
        initial_err = abs(errors[0]) if len(errors) > 0 else 0.4
        os_pct = ((peak_err - initial_err) / initial_err) * 100 if peak_err > initial_err else 0.0
        return [iae, ise, os_pct, 0.0, rmse]


# =====================================================================
# --- GUI MODULES ---
# =====================================================================

class ClickableLabel(QLabel):
    """Custom Label that emits a signal when clicked."""
    clicked = pyqtSignal()
    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.clicked.emit()
        super().mousePressEvent(event)

class StatusPill(QLabel):
    """Rounded status chip: colored dot + text, swaps palette by state."""
    STATES = {
        "ready":   ("#DFF3E7", "#1E8E5A", "● Ready"),
        "running": ("#FCEBD8", "#B36B1D", "● Running"),
        "stopped": ("#E7EAEF", "#6B7684", "● Stopped"),
        "sending": ("#DCE7FC", "#2F6FED", "◐ Sending..."),
    }
    def __init__(self, state="ready", parent=None, text=None):
        super().__init__(parent)
        self.setAlignment(Qt.AlignCenter)
        self._opacity_effect = QGraphicsOpacityEffect(self)
        self.setGraphicsEffect(self._opacity_effect)
        self._opacity_effect.setOpacity(1.0)
        self._anim = None
        self.set_state(state, text, animate=False)

    def set_state(self, state, text=None, animate=True):
        bg, fg, default_text = self.STATES.get(state, self.STATES["ready"])
        self.setText(text if text else default_text)
        self.setStyleSheet(f"""
            background-color: {bg}; color: {fg}; font-weight: 700;
            border-radius: 10px; padding: 3px 12px; font-family: 'Consolas', monospace; font-size: 11px;
        """)
        if animate:
            self._fade_pulse()

    def _fade_pulse(self):
        self._anim = QPropertyAnimation(self._opacity_effect, b"opacity")
        self._anim.setDuration(280)
        self._anim.setKeyValueAt(0.0, 1.0)
        self._anim.setKeyValueAt(0.4, 0.35)
        self._anim.setKeyValueAt(1.0, 1.0)
        self._anim.setEasingCurve(QEasingCurve.InOutQuad)
        self._anim.start()

class DiagramViewer(QDialog):
    """Popup window for high-resolution diagram inspection."""
    def __init__(self, image_path, parent=None):
        super().__init__(parent)
        self.setStyleSheet(THEME_QSS)
        self.setWindowTitle("System Diagram - Full Resolution")
        self.resize(1000, 800)
        
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        self.label = QLabel()
        pixmap = QPixmap(image_path)
        self.label.setPixmap(pixmap)
        
        self.label.setFixedSize(pixmap.width(), pixmap.height())
        
        scroll = QScrollArea()
        scroll.setStyleSheet("background-color: #F5F6F8;")
        scroll.setWidgetResizable(False) 
        scroll.setWidget(self.label)
        layout.addWidget(scroll)


class InteractiveMapDialog(QDialog):
    def __init__(self, existing_points, parent=None):
        super().__init__(parent)
        self.setStyleSheet(THEME_QSS)
        self.setWindowTitle("Map Waypoint Selector")
        self.resize(650, 650)
        self.via_points = existing_points.copy()
        
        layout = QVBoxLayout(self)
        self.figure = Figure(figsize=(6, 6), dpi=100)
        self.canvas = FigureCanvas(self.figure)
        layout.addWidget(self.canvas)
        
        self.coord_label = QLabel(f"Absolute Points: {self.via_points}" if self.via_points else "Click on the grid to add Webots absolute coordinates.")
        self.coord_label.setAlignment(Qt.AlignCenter)
        self.coord_label.setStyleSheet("font-weight: 700; font-size: 14px; padding: 10px; color: #1F2A37;")
        layout.addWidget(self.coord_label)
        
        self.btn_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        self.clear_btn = QPushButton("Clear Points")
        self.btn_box.addButton(self.clear_btn, QDialogButtonBox.ActionRole)
        layout.addWidget(self.btn_box)
        
        self.btn_box.accepted.connect(self.accept)
        self.btn_box.rejected.connect(self.reject)
        self.clear_btn.clicked.connect(self.clear_plot)
        self.canvas.mpl_connect('button_press_event', self.on_click)
        self.setup_plot()

    def setup_plot(self):
        self.figure.clear()
        self.ax = self.figure.add_subplot(111)
        self.ax.set_title("Interactive Arena Map (Webots 10x10 Arena)")
        
        self.ax.set_xlim(-5.5, 5.5) 
        self.ax.set_ylim(-5.5, 5.5)
        self.ax.set_xlabel("Webots X (meters)")
        self.ax.set_ylabel("Webots Y (meters)")
        self.ax.grid(True, linestyle='--', alpha=0.5)
        self.ax.set_aspect('equal')
        
        self.ax.plot(0, 0, 'k+', markersize=15, markeredgewidth=2, label="Arena Center (0,0)")
        
        walls = Rectangle((-5.0, -5.0), 10.0, 10.0, fill=False, edgecolor='black', linewidth=3)
        self.ax.add_patch(walls)

        self.ax.plot(ROBOT_START_X, ROBOT_START_Y, 'gX', markersize=12, label=f"Robot Start ({ROBOT_START_X}, {ROBOT_START_Y})")
        self.ax.legend(loc="upper right", fontsize='small')
        
        for pt in self.via_points:
            self.ax.plot(pt[0], pt[1], 'ro', markersize=8)
        if len(self.via_points) > 1:
            pts = np.array(self.via_points)
            self.ax.plot(pts[:,0], pts[:,1], 'b-', alpha=0.5)
            
        self.canvas.draw()

    def on_click(self, event):
        if event.inaxes != self.ax: return
        x, y = round(event.xdata, 2), round(event.ydata, 2)
        
        if x < -5.0 or x > 5.0 or y < -5.0 or y > 5.0:
            return
            
        self.via_points.append([x, y])
        self.coord_label.setText(f"Absolute Points: {self.via_points}")
        self.ax.plot(x, y, 'ro', markersize=8)
        if len(self.via_points) > 1:
            prev_point = self.via_points[-2]
            self.ax.plot([prev_point[0], x], [prev_point[1], y], 'b-', alpha=0.5)
        self.canvas.draw()

    def clear_plot(self):
        self.via_points.clear()
        self.coord_label.setText("Click on the grid to add Webots absolute coordinates.")
        self.setup_plot()
        
    def get_points(self): return self.via_points


class GlossaryDialog(QDialog):
    """Non-modal reference panel explaining GCS/control-theory terms for students."""
    GLOSSARY_HTML = """
        <h2 style="margin-bottom:4px;">GCS Glossary</h2>
        <p style="color:#6B7684; margin-top:0;">Quick reference for terms used throughout this dashboard.</p>

        <h3>PID Control</h3>
        <p><b>Kp (Proportional gain):</b> Scales the correction in direct proportion to the current error.
        Higher Kp means a more aggressive, immediate steering response toward the target, but too high a value
        can cause overshoot or oscillation.</p>
        <p><b>Ki (Integral gain):</b> Accumulates past error over time and corrects for it. Useful for
        eliminating persistent steady-state error, but excessive Ki can cause slow oscillations ("integral windup").</p>
        <p><b>Kd (Derivative gain):</b> Reacts to the rate of change of the error, effectively predicting where
        the error is heading. Used to dampen the response and reduce overshoot.</p>

        <h3>Performance Metrics</h3>
        <p><b>Settling Time:</b> The time it takes for the system's response to enter and stay within a small
        tolerance band around its final (target) value. A shorter settling time generally means faster, more
        responsive control &mdash; provided it isn't achieved at the cost of instability.</p>
        <p><b>RMSE (Root Mean Squared Error):</b> A single number summarizing how far, on average, the robot's
        actual path deviated from the intended path or trajectory. Lower RMSE means the robot tracked the
        reference path more closely.</p>
        <p><b>Steady-State Error:</b> The remaining, persistent gap between the target value and the actual
        value once the system has settled and stopped changing. Ideally this approaches zero; integral gain
        (Ki) is often used specifically to drive it down.</p>
        <p><b>Overshoot:</b> How far the system's response exceeds the target value before settling back down.
        Common with high proportional or low derivative gains.</p>

        <h3>Navigation</h3>
        <p><b>Path Planning:</b> The process of computing a geometric route from the robot's current position to
        a goal, typically before motion begins, while avoiding known obstacles. The result is usually a series
        of waypoints or a continuous curve the robot should follow.</p>
        <p><b>Path Tracking / Trajectory Tracking:</b> The control problem of steering the robot so it follows
        a previously planned path or trajectory as closely as possible over time. Where path planning decides
        <i>where to go</i>, path/trajectory tracking is about <i>how well the robot actually follows that route</i>
        &mdash; this is what PID gains and RMSE are ultimately measuring.</p>

        <h3>System / ROS 2</h3>
        <p><b>Node:</b> An independent process in ROS 2 that performs a specific task (e.g. a controller or
        sensor driver) and communicates with other nodes over topics and parameters.</p>
        <p><b>Parameter:</b> A named, runtime-configurable value on a ROS 2 node (like <i>kp</i>, <i>ki</i>,
        <i>kd</i>) that can be read or updated without restarting the node.</p>
        <p><b>Waypoint:</b> A single coordinate the robot is instructed to pass through or reach, typically as
        part of a longer planned trajectory.</p>
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet(THEME_QSS)
        self.setWindowTitle("Glossary & Help")
        self.setModal(False)
        self.resize(480, 560)

        layout = QVBoxLayout(self)
        browser = QTextBrowser()
        browser.setOpenExternalLinks(False)
        browser.setHtml(self.GLOSSARY_HTML)
        layout.addWidget(browser)

        close_btn = QPushButton("Close")
        close_btn.setObjectName("primaryAction")
        close_btn.clicked.connect(self.close)
        layout.addWidget(close_btn)


class AnalyticsWindow(QWidget):
    def __init__(self, scenario_name):
        super().__init__()
        self.scenario_name = scenario_name
        self.setStyleSheet(THEME_QSS)
        self.setWindowTitle(f"Performance Evaluation Dashboard - {self.scenario_name}")
        self.resize(1050, 750) 
        self.init_ui()

    def init_ui(self):
        self.outer_layout = QVBoxLayout(self)
        self.outer_layout.setContentsMargins(0, 0, 0, 0)
        
        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        
        self.content_container = QWidget()
        self.page_layout = QVBoxLayout(self.content_container)
        self.page_layout.setSpacing(20)
        
        header_row = QHBoxLayout()
        self.extract_btn = QPushButton(f"⬆ Load {self.scenario_name} Experiment Data")
        self.extract_btn.setObjectName("primaryAction")
        self.extract_btn.setStyleSheet("padding: 12px; font-size: 14px;")
        self.extract_btn.clicked.connect(self.load_and_plot_file)

        self.glossary_btn = QPushButton("Glossary")
        self.glossary_btn.setFixedHeight(32)
        self.glossary_btn.setToolTip("Glossary & Help: definitions for settling time, RMSE, PID gains, and more.")
        self.glossary_btn.setStyleSheet("border-radius: 16px; font-weight: 700; font-size: 13px; padding: 0 14px;")
        self.glossary_btn.clicked.connect(self.open_glossary)

        header_row.addWidget(self.extract_btn, stretch=1)
        header_row.addWidget(self.glossary_btn)
        self.page_layout.addLayout(header_row)

        self.stats_label = QLabel("Experiment Log: Pending Selection")
        self.stats_label.setFont(QFont("Consolas", 11))
        self.stats_label.setAlignment(Qt.AlignCenter)
        self.stats_label.setStyleSheet("background-color: #FAFBFC; padding: 10px; border: 1px solid #DDE1E6; border-radius: 6px;")
        self.page_layout.addWidget(self.stats_label)

        metrics_lbl = QLabel("▣ ENGINEERING PERFORMANCE INDICES:")
        metrics_lbl.setObjectName("groupSubtitle")
        self.page_layout.addWidget(metrics_lbl)
        self.metrics_table = QTableWidget(1, 2) 
        self.metrics_table.setHorizontalHeaderLabels(["Settling (s)", "RMSE (m)"])
        self.metrics_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.metrics_table.setFixedHeight(85)
        self.page_layout.addWidget(self.metrics_table)

        plots_lbl = QLabel("◱ EXPERIMENT RESPONSE PLOTS:")
        plots_lbl.setObjectName("groupSubtitle")
        self.page_layout.addWidget(plots_lbl)
        self.figure = Figure(figsize=(10, 10), dpi=100)
        
        self.ax1 = self.figure.add_subplot(211)
        self.ax2 = self.figure.add_subplot(223)
        self.ax3 = self.figure.add_subplot(224)
        self.figure.subplots_adjust(hspace=0.4, wspace=0.3)

        self.canvas = FigureCanvas(self.figure)
        self.toolbar = NavigationToolbar(self.canvas, self)
        self.canvas.setMinimumHeight(800) 
        
        self.page_layout.addWidget(self.toolbar)
        self.page_layout.addWidget(self.canvas)
        self.scroll.setWidget(self.content_container)
        self.outer_layout.addWidget(self.scroll)

    def open_glossary(self):
        self.glossary_window = GlossaryDialog(self)
        self.glossary_window.show()

    def load_and_plot_file(self):
        csv_filename, _ = QFileDialog.getOpenFileName(self, f"Open {self.scenario_name} Log", DEFAULT_LOG_DIR, "CSV Files (*.csv)")
        if not csv_filename: return

        try:
            df = pd.read_csv(csv_filename)
            df.columns = df.columns.str.strip() 
        except Exception as e:
            QMessageBox.critical(self, "Read Error", f"Failed to parse CSV:\n{e}")
            return
            
        if self.scenario_name == "Wall Following":
            self.plot_wall_following(df, csv_filename)
        elif self.scenario_name == "Obstacle Avoidance":
            self.plot_obstacle_avoidance(df, csv_filename)
        elif self.scenario_name == "Trajectory Planning":
            self.plot_trajectory_planning(df, csv_filename)

        self.figure.tight_layout(pad=3.0)
        self.canvas.draw()

    def update_table(self, values):
        for i, val in enumerate(values):
            self.metrics_table.setItem(0, i, QTableWidgetItem(f"{val}"))
            
    def plot_wall_following(self, df, path):
        self.ax1.clear()
        self.ax2.clear()
        self.ax3.clear()
        
        if df.empty:
            QMessageBox.critical(self, "Data Error", "The selected CSV file is completely empty.")
            return
            
        required = ['Time', 'Desired_Distance', 'Measured_Distance', 'Angular_Vel', 'X', 'Y']
        if not all(col in df.columns for col in required):
            QMessageBox.critical(self, "Data Error", f"CSV missing required columns.\nRequires: {required}")
            return

        self.stats_label.setText(f"Wall Following Analysis | File: {os.path.basename(path)}")

        times = df['Time'].values
        measured = df['Measured_Distance'].values
        setpoint = df['Desired_Distance'].iloc[0]

        initial_val = measured[0]
        diff = abs(initial_val - setpoint)
        
        try:
            t_10 = times[np.where(abs(measured - initial_val) >= 0.10 * diff)[0][0]]
            t_90 = times[np.where(abs(measured - initial_val) >= 0.90 * diff)[0][0]]
            rise_time = t_90 - t_10
        except IndexError:
            t_10, t_90, rise_time = None, None, 0.0

        rmse = np.sqrt(np.mean((measured - setpoint)**2))
        self.update_table([f"Rise (10-90): {rise_time:.2f}s", f"RMSE: {rmse:.4f}m"])

        tolerance = 0.05 * setpoint
        self.ax1.fill_between(times, setpoint - tolerance, setpoint + tolerance, 
                              color='gray', alpha=0.2, label='5% Tolerance Band')
        self.ax1.plot(df['Time'], measured, color='tab:blue', linewidth=2, label='Measured Distance')
        self.ax1.axhline(setpoint, color='tab:red', linestyle='--', linewidth=2, label='Setpoint')

        if t_10 and t_90:
            self.ax1.axvline(t_10, color='green', linestyle=':', label='Rise Time Start (10%)')
            self.ax1.axvline(t_90, color='green', linestyle=':', label='Rise Time End (90%)')

        self.ax1.set_title('Figure 1 - Wall Distance Response')
        self.ax1.set_xlabel('Time (s)')
        self.ax1.set_ylabel('Distance to wall (m)')
        self.ax1.legend(loc='upper right', fontsize=9)
        self.ax1.grid(True, alpha=0.5)

        self.ax2.plot(df['Time'], df['Angular_Vel'], color='tab:orange', linewidth=2, label='Angular Velocity')
        self.ax2.axhline(0, color='black', linestyle='--', linewidth=1.5)
        self.ax2.set_title('Figure 2 - Angular Velocity vs Time')
        self.ax2.set_xlabel('Time (s)')
        self.ax2.set_ylabel('Angular Velocity (rad/s)')
        self.ax2.legend(loc='upper right')
        self.ax2.grid(True, alpha=0.5)

        self.ax3.plot(df['X'], df['Y'], color='blue', linewidth=2, label='Robot Trajectory')
        self.ax3.scatter(df['X'].iloc[0], df['Y'].iloc[0], color='green', marker='o', s=100, label='Start', zorder=5)
        self.ax3.scatter(df['X'].iloc[-1], df['Y'].iloc[-1], color='red', marker='X', s=100, label='End', zorder=5)
        self.ax3.set_title('Figure 5 - Robot Trajectory')
        self.ax3.set_xlabel('X Position (m)')
        self.ax3.set_ylabel('Y Position (m)')
        self.ax3.axis('equal')
        self.ax3.legend(loc='upper right')
        self.ax3.grid(True, alpha=0.5)

    def plot_trajectory_planning(self, df, path):
        self.ax1.clear()
        self.ax2.clear()
        self.ax3.clear()
        
        if df.empty:
            QMessageBox.critical(self, "Data Error", "The selected CSV file is completely empty.")
            return

        required = ['Time', 'Ref_X', 'Ref_Y', 'Robot_X', 'Robot_Y', 'Cross_Track_Error', 'Heading_Error']
        if not all(col in df.columns for col in required):
            QMessageBox.critical(self, "Data Error", f"CSV missing required columns.\nRequires: {required}")
            return
            
        self.stats_label.setText(f"📐 Pure Pursuit Analysis | File: {os.path.basename(path)}")
        time = df['Time'].values
        cte = df['Cross_Track_Error'].values

        rmse_cte = np.sqrt(np.mean(cte**2))
        max_cte = np.max(np.abs(cte))
        self.update_table([f"Max: {max_cte:.4f}m", f"RMSE: {rmse_cte:.4f}m"])

        self.ax1.plot(df['Ref_X'], df['Ref_Y'], color='black', linestyle='--', linewidth=2, label='Reference Path')
        self.ax1.plot(df['Robot_X'], df['Robot_Y'], color='tab:blue', linewidth=2, label='Actual Trajectory')
        self.ax1.scatter(df['Ref_X'].iloc[0], df['Ref_Y'].iloc[0], color='green', marker='o', s=120, label='Start Point', zorder=5)
        self.ax1.scatter(df['Ref_X'].iloc[-1], df['Ref_Y'].iloc[-1], color='red', marker='X', s=120, label='Goal Point', zorder=5)
        self.ax1.set_title('Figure 1 - Reference Path vs Actual Robot Trajectory')
        self.ax1.set_xlabel('X Position (m)')
        self.ax1.set_ylabel('Y Position (m)')
        self.ax1.axis('equal') 
        self.ax1.grid(True, alpha=0.5)
        self.ax1.legend()

        self.ax2.plot(time, cte, color='tab:red', linewidth=2, label='Cross-Track Error')
        self.ax2.axhline(0, color='black', linestyle='-', linewidth=1)
        self.ax2.set_title('Figure 2 - Cross-Track Error vs Time')
        self.ax2.set_xlabel('Time (s)')
        self.ax2.set_ylabel('Cross-track error (m)')
        self.ax2.grid(True, alpha=0.5)
        self.ax2.legend()

        self.ax3.plot(time, df['Heading_Error'], color='tab:purple', linewidth=2, label='Heading Error')
        self.ax3.axhline(0, color='black', linestyle='-', linewidth=1)
        self.ax3.set_title('Figure 3 - Heading Error vs Time')
        self.ax3.set_xlabel('Time (s)')
        self.ax3.set_ylabel('Heading error (degrees)')
        self.ax3.grid(True, alpha=0.5)
        self.ax3.legend()
    
    def plot_obstacle_avoidance(self, df, path):
        self.ax1.clear()
        self.ax2.clear()
        self.ax3.clear() 
        
        if df.empty:
            QMessageBox.critical(self, "Data Error", "The selected CSV file is completely empty.")
            return
            
        required = ['Time', 'Clearance', 'Angular_Vel']
        if not all(col in df.columns for col in required):
            QMessageBox.critical(self, "Data Error", f"CSV missing required columns.\nRequires: {required}")
            return
            
        time, clearance, ang_vel = df['Time'].values, df['Clearance'].values, df['Angular_Vel'].values
        min_clear = np.min(clearance)
        self.stats_label.setText(f"▲ OBSTACLE AVOIDANCE ▲ | File: {os.path.basename(path)}")
        self.update_table([f"{min_clear:.4f}", f"{time[-1]:.1f}"])

        self.ax1.plot(time, clearance, color='#e67e22', linewidth=2, label='Clearance')
        self.ax1.axhline(0.2, color='red', linestyle='--', label='Safety Limit')
        self.ax1.set(title="Proximity Metrics", ylabel="Dist (m)")
        self.ax1.grid(True, linestyle=':', alpha=0.7)
        
        self.ax2.plot(time, ang_vel, color='#9b59b6', label='Yaw Rate')
        self.ax2.set(title="Maneuver Intensity", xlabel="Time (s)", ylabel="Rad/s")
        self.ax2.grid(True, linestyle=':', alpha=0.7)


class AFIT_GCS(QWidget):
    def __init__(self):
        super().__init__()
        self.ros_manager = ROS2ProcessManager()
        self.custom_node = ""
        self.current_waypoints = []
        self.current_diagram_path = ""
        
        self.logic_matrix = {
            "Teleop": {
                "worlds": ["obstacle_course.wbt", "garden.wbt", "wall_following_1.wbt"], 
                "node": "teleop_node", 
                "controller": "Manual Control",
                "has_analytics": False,
                "objective": "Understand how manual keyboard commands are translated into robot motion through the ROS2 communication framework and the differential-drive controller.",
                "how_it_works": "• Press the W, A, S or D keys to control the robot.\n• The teleoperation node converts the selected key into linear and angular velocity values.\n• A Twist velocity command is published through the ROS2 topic (/diffdrive_controller/cmd_vel_unstamped)\n• The differential drive controller converts the velocity command into wheel velocities.\n• The robot moves in Webots.",
                "diagram_path": os.path.join(IMAGE_DIR, "diagram_teleop.png") if IMAGE_DIR else "diagram_teleop.png"
            },
            "Trajectory Planning": {
                "worlds": ["obstacle_course.wbt"], 
                "node": "niku_controller_viapoints", 
                "controller": "Pure Pursuit Spline",
                "has_analytics": True,
                "objective": "Understand how a robot generates a smooth trajectory through user-defined waypoints and follows it using the Pure Pursuit controller.",
                "how_it_works": "• Select waypoints.\n• A Piecewise Quintic Polynomial trajectory is generated.\n• Pure Pursuit selects a look-ahead point.\n• The controller computes the steering command.\n• The robot follows the planned trajectory.",
                "diagram_path": os.path.join(IMAGE_DIR, "diagram_trajectory.png") if IMAGE_DIR else "diagram_trajectory.png"
            },
            "Wall Following": {
                "worlds": ["wall_following_1.wbt", "wall_following_2.wbt"], 
                "node": "wall_follower", 
                "controller": "PID Control",
                "has_analytics": True,
                "objective": "Investigate how feedback control enables a robot to maintain a desired distance from a wall by adjusting the PID controller gains.",
                "how_it_works": "• The laser scanner measures the wall distance.\n• The measured distance is compared with the desired distance.\n• The PID controller computes a steering correction.\n• The steering command is sent to the robot.\n• The process repeats continuously.",
                "diagram_path": os.path.join(IMAGE_DIR, "diagram_wall_following.jpeg") if IMAGE_DIR else "diagram_wall_following.jpeg"
            }
        }
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("Integrated Mobile Robot Experimentation Platform")
        self.resize(750, 950) 
        
        self.setStyleSheet(THEME_QSS)

        self.scroll_main = QScrollArea()
        self.scroll_main.setWidgetResizable(True)
        self.main_container = QWidget()
        main_layout = QVBoxLayout(self.main_container)

        self.logo_label = QLabel()
        self.logo_label.setAlignment(Qt.AlignCenter)
        
        logo_path = os.path.join(IMAGE_DIR, "afit_logo.png") if IMAGE_DIR else "afit_logo.png"
        
        if os.path.exists(logo_path):
            pix = QPixmap(logo_path)
            self.logo_label.setPixmap(pix.scaledToHeight(120, Qt.SmoothTransformation))
        else:
            self.logo_label.setText(f"[Missing: {logo_path}]")
            
        main_layout.addWidget(self.logo_label)

        group_lbl = QLabel("MCT GROUP B")
        group_lbl.setAlignment(Qt.AlignCenter)
        group_lbl.setObjectName("sectionTitle")
        group_lbl.setStyleSheet("margin-bottom: 8px;")
        main_layout.addWidget(group_lbl)
        
        title_row = QHBoxLayout()
        title_lbl = QLabel("INTEGRATED MOBILE ROBOT EXPERIMENTATION PLATFORM")
        title_lbl.setAlignment(Qt.AlignCenter)
        title_lbl.setObjectName("bannerTitle")

        self.glossary_btn = QPushButton("Glossary")
        self.glossary_btn.setFixedHeight(32)
        self.glossary_btn.setToolTip("Glossary & Help: definitions for settling time, RMSE, PID gains, and more.")
        self.glossary_btn.setStyleSheet("border-radius: 16px; font-weight: 700; font-size: 13px; padding: 0 14px;")
        self.glossary_btn.clicked.connect(self.open_glossary)

        title_row.addWidget(title_lbl, stretch=1)
        title_row.addWidget(self.glossary_btn)
        main_layout.addLayout(title_row)

        main_layout.addWidget(self.create_summary_panel())
        main_layout.addWidget(self.create_configuration_panel())
        main_layout.addWidget(self.create_execution_panel())
        main_layout.addWidget(self.create_evaluation_panel())
        
        term_label = QLabel("Raw ROS 2 Terminal Execution:")
        term_label.setObjectName("mutedText")
        term_label.setStyleSheet("margin-top: 10px;")
        main_layout.addWidget(term_label)
        
        self.terminal_display = QTextEdit()
        self.terminal_display.setReadOnly(True)
        self.terminal_display.setPlaceholderText("Awaiting ROS 2 subsystem execution...")
        self.terminal_display.setMaximumHeight(100)
        self.terminal_display.setStyleSheet("""
            background-color: #14171D; 
            color: #4AF67A; 
            font-family: 'Consolas', 'Courier New', monospace;
            font-size: 12px;
            border: 1px solid #DDE1E6;
            border-radius: 6px;
            padding: 6px;
        """)
        main_layout.addWidget(self.terminal_display)
        
        self.scroll_main.setWidget(self.main_container)
        
        central_layout = QVBoxLayout(self)
        central_layout.setContentsMargins(0, 0, 0, 0)
        central_layout.addWidget(self.scroll_main)

        self.sync_dropdowns()

    def log_terminal(self, command):
        self.terminal_display.append(f"$ {command}")
        scrollbar = self.terminal_display.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def create_summary_panel(self):
        panel = QGroupBox("CURRENT EXPERIMENT SUMMARY")
        layout = QVBoxLayout(panel)
        
        self.sum_exp_lbl = QLabel("Experiment : ")
        self.sum_world_lbl = QLabel("World      : ")
        self.sum_ctrl_lbl = QLabel("Controller : ")
        
        for lbl in [self.sum_exp_lbl, self.sum_world_lbl, self.sum_ctrl_lbl]:
            lbl.setFont(QFont("Consolas", 10))
            layout.addWidget(lbl)

        status_row = QHBoxLayout()
        status_caption = QLabel("Status     :")
        status_caption.setFont(QFont("Consolas", 10))
        self.sum_status_lbl = StatusPill("ready")
        status_row.addWidget(status_caption)
        status_row.addWidget(self.sum_status_lbl)
        status_row.addStretch()
        layout.addLayout(status_row)
            
        return panel

    def _fade_widget(self, widget):
        effect = widget.graphicsEffect()
        if not isinstance(effect, QGraphicsOpacityEffect):
            effect = QGraphicsOpacityEffect(widget)
            widget.setGraphicsEffect(effect)
        effect.setOpacity(1.0)
        anim = QPropertyAnimation(effect, b"opacity", widget)
        anim.setDuration(280)
        anim.setKeyValueAt(0.0, 1.0)
        anim.setKeyValueAt(0.4, 0.35)
        anim.setKeyValueAt(1.0, 1.0)
        anim.setEasingCurve(QEasingCurve.InOutQuad)
        widget._fade_anim = anim 
        anim.start()

    def make_step_header(self, number, title):
        row = QHBoxLayout()
        row.setContentsMargins(0, 0, 0, 4)
        num_lbl = QLabel(str(number))
        num_lbl.setFixedSize(24, 24)
        num_lbl.setAlignment(Qt.AlignCenter)
        num_lbl.setStyleSheet("""
            background-color: #2F6FED; color: #FFFFFF; font-weight: 700;
            border-radius: 12px; font-size: 12px;
        """)
        title_lbl = QLabel(title)
        title_lbl.setObjectName("groupSubtitle")
        title_lbl.setStyleSheet("margin-bottom: 0px;")
        row.addWidget(num_lbl)
        row.addWidget(title_lbl)
        row.addStretch()
        return row

    def create_configuration_panel(self):
        panel = QGroupBox()
        layout = QVBoxLayout(panel)
        layout.addLayout(self.make_step_header(1, "EXPERIMENT CONFIGURATION"))
        
        row_selectors = QHBoxLayout()
        self.scenario_combo = QComboBox()
        self.scenario_combo.addItems(self.logic_matrix.keys())
        self.world_combo = QComboBox()
        
        row_selectors.addWidget(QLabel("Experiment Type:"))
        row_selectors.addWidget(self.scenario_combo, stretch=2)
        row_selectors.addWidget(QLabel("Webots World:"))
        row_selectors.addWidget(self.world_combo, stretch=2)
        layout.addLayout(row_selectors)

        self.learning_panel = QGroupBox("Learning Panel")
        lp_layout = QVBoxLayout(self.learning_panel)
        
        lp_layout.addWidget(QLabel("Experiment Objective"))
        self.obj_browser = QTextBrowser()
        self.obj_browser.setMaximumHeight(50)
        lp_layout.addWidget(self.obj_browser)
        
        lp_layout.addWidget(QLabel("How It Works"))
        self.hw_browser = QTextBrowser()
        self.hw_browser.setMaximumHeight(130)
        lp_layout.addWidget(self.hw_browser)
        
        lp_layout.addWidget(QLabel("System Diagram (Click to enlarge)"))
        
        self.diagram_label = ClickableLabel("Loading Diagram...")
        self.diagram_label.setAlignment(Qt.AlignCenter)
        self.diagram_label.setMinimumHeight(280)
        self.diagram_label.setStyleSheet("background-color: #FAFBFC; border: 1px dashed #C7D2E0; border-radius: 6px;")
        self.diagram_label.clicked.connect(self.open_diagram_popup)
        
        lp_layout.addWidget(self.diagram_label)
        
        layout.addWidget(self.learning_panel)

        row_robot = QHBoxLayout()
        self.robot_name_input = QLineEdit()
        self.robot_name_input.setPlaceholderText("Robot Name in Webots (Default: robot_a)")
        row_robot.addWidget(QLabel("Robot Target:"))
        row_robot.addWidget(self.robot_name_input)
        layout.addLayout(row_robot)

        layout.addWidget(QLabel("Experiment Parameters:"))
        self.pid_panel = self.create_pid_panel()
        self.traj_panel = self.create_traj_panel()
        self.empty_param_panel = QLabel("No runtime parameters required for this experiment.")
        self.empty_param_panel.setObjectName("helpText")
        
        layout.addWidget(self.pid_panel)
        layout.addWidget(self.traj_panel)
        layout.addWidget(self.empty_param_panel)

        self.scenario_combo.currentTextChanged.connect(self.sync_dropdowns)
        self.world_combo.currentTextChanged.connect(self.update_summary)
        
        return panel

    def create_execution_panel(self):
        panel = QGroupBox()
        outer = QVBoxLayout(panel)
        outer.addLayout(self.make_step_header(2, "EXPERIMENT EXECUTION"))
        
        layout = QHBoxLayout()
        
        self.launch_btn = QPushButton("▶ START EXPERIMENT")
        self.launch_btn.setObjectName("primaryAction")
        
        self.kill_btn = QPushButton("■ STOP EXPERIMENT")
        self.kill_btn.setObjectName("dangerAction")
        
        layout.addWidget(self.launch_btn)
        layout.addWidget(self.kill_btn)
        outer.addLayout(layout)
        
        self.launch_btn.clicked.connect(self.handle_launch)
        self.kill_btn.clicked.connect(self.handle_kill)
        
        return panel

    def create_evaluation_panel(self):
        panel = QGroupBox()
        layout = QVBoxLayout(panel)
        layout.addLayout(self.make_step_header(3, "PERFORMANCE EVALUATION"))
        
        desc = QLabel("(Plots and engineering performance metrics are computed in the dashboard)")
        desc.setObjectName("mutedText")
        layout.addWidget(desc)
        
        self.open_dashboard_btn = QPushButton("▣ OPEN ANALYSIS DASHBOARD")
        self.open_dashboard_btn.setObjectName("accentAction")
        layout.addWidget(self.open_dashboard_btn)
        
        self.open_dashboard_btn.clicked.connect(self.launch_analytics_window)
        
        return panel

    def create_pid_panel(self):
        panel = QWidget()
        layout = QHBoxLayout(panel)
        layout.setContentsMargins(0,0,0,0)
        self.kp_box, self.ki_box, self.kd_box = QDoubleSpinBox(), QDoubleSpinBox(), QDoubleSpinBox()
        
        for box in (self.kp_box, self.ki_box, self.kd_box):
            box.setDecimals(3)
            box.setRange(0.0, 10.0)
            box.setSingleStep(0.05)
            box.setValue(1.0)
            box.setMinimumWidth(90)
            box.setMinimumHeight(30)
            
        self.kp_box.setToolTip("<b>kp (Proportional):</b> Drives the robot towards the target. Higher values mean a more aggressive steering response.")
        self.ki_box.setToolTip("<b>ki (Integral):</b> Eliminates steady-state error by accumulating past errors. Use sparingly to avoid oscillation.")
        self.kd_box.setToolTip("<b>kd (Derivative):</b> Dampens the response. Predicts future error to prevent the robot from overshooting the target.")
            
        self.update_pid_btn = QPushButton("UPDATE GAINS")
        self.update_pid_btn.setObjectName("primaryAction")
        self.update_pid_btn.clicked.connect(self.send_pid_update)

        self.pid_status_pill = StatusPill("stopped", text="Idle")
        
        layout.addWidget(QLabel("Kp:")); layout.addWidget(self.kp_box)
        layout.addWidget(QLabel("Ki:")); layout.addWidget(self.ki_box)
        layout.addWidget(QLabel("Kd:")); layout.addWidget(self.kd_box)
        layout.addWidget(self.update_pid_btn)
        layout.addWidget(self.pid_status_pill)
        return panel

    def create_traj_panel(self):
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0,0,0,0)
        
        row1 = QHBoxLayout()
        self.open_map_btn = QPushButton("◈ Open Waypoint Selector")
        self.open_map_btn.clicked.connect(self.open_waypoint_map)
        row1.addWidget(self.open_map_btn)
        layout.addLayout(row1)

        row2 = QHBoxLayout()
        self.wp_x = QDoubleSpinBox()
        self.wp_x.setRange(-5.0, 5.0) 
        self.wp_x.setSingleStep(0.25)
        
        self.wp_y = QDoubleSpinBox()
        self.wp_y.setRange(-5.0, 5.0) 
        self.wp_y.setSingleStep(0.25)

        self.add_wp_btn = QPushButton("+ Add Pt")
        self.clear_wp_btn = QPushButton("Clear")
        self.add_wp_btn.clicked.connect(self.manual_add_wp)
        self.clear_wp_btn.clicked.connect(self.clear_wps)

        row2.addWidget(QLabel("X:")); row2.addWidget(self.wp_x)
        row2.addWidget(QLabel("Y:")); row2.addWidget(self.wp_y)
        row2.addWidget(self.add_wp_btn)
        row2.addWidget(self.clear_wp_btn)
        layout.addLayout(row2)

        self.waypoints_display = QLineEdit()
        self.waypoints_display.setReadOnly(True)
        self.waypoints_display.setPlaceholderText("No absolute points set...")
        layout.addWidget(self.waypoints_display)

        return panel

    # --- BEHAVIOR & LOGIC ---

    def load_diagram_image(self, file_path):
        self.current_diagram_path = file_path
        if file_path and os.path.exists(file_path):
            pixmap = QPixmap(file_path)
            scaled_pixmap = pixmap.scaled(600, 280, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.diagram_label.setPixmap(scaled_pixmap)
            self.diagram_label.setCursor(Qt.PointingHandCursor) 
        else:
            self.diagram_label.clear()
            self.diagram_label.setText(f"Missing File: {file_path}")
            self.diagram_label.setCursor(Qt.ArrowCursor)

    def open_diagram_popup(self):
        if self.current_diagram_path and os.path.exists(self.current_diagram_path):
            self.diagram_viewer = DiagramViewer(self.current_diagram_path, self)
            self.diagram_viewer.show()

    def sync_dropdowns(self):
        scenario = self.scenario_combo.currentText()
        data = self.logic_matrix[scenario]
        
        self.world_combo.blockSignals(True)
        self.world_combo.clear()
        self.world_combo.addItems(data["worlds"])
        self.world_combo.blockSignals(False)
        
        self.obj_browser.setText(data["objective"])
        self.hw_browser.setText(data["how_it_works"])
        self.load_diagram_image(data.get("diagram_path", ""))

        is_traj = (scenario == "Trajectory Planning")
        is_pid = (scenario == "Wall Following")
        
        self.traj_panel.setVisible(is_traj)
        self.pid_panel.setVisible(is_pid)
        self.empty_param_panel.setVisible(not is_traj and not is_pid)

        if data["has_analytics"]:
            self.open_dashboard_btn.setEnabled(True)
            self.open_dashboard_btn.setText("▣ OPEN ANALYSIS DASHBOARD")
        else:
            self.open_dashboard_btn.setEnabled(False)
            self.open_dashboard_btn.setText("Analytics Disabled for this Experiment")
        self._fade_widget(self.open_dashboard_btn)
            
        self.update_summary()

    def update_summary(self, status_override=None):
        scenario = self.scenario_combo.currentText()
        data = self.logic_matrix[scenario]
        
        self.sum_exp_lbl.setText(f"Experiment : {scenario}")
        self.sum_world_lbl.setText(f"World      : {self.world_combo.currentText()}")
        self.sum_ctrl_lbl.setText(f"Controller : {data['controller']}")
        
        if status_override:
            if "Running" in status_override:
                self.sum_status_lbl.set_state("running", status_override)
            elif "Stopped" in status_override or "Completed" in status_override:
                self.sum_status_lbl.set_state("stopped", status_override)
            else:
                self.sum_status_lbl.set_state("ready", status_override)

    def manual_add_wp(self):
        x = round(self.wp_x.value(), 2)
        y = round(self.wp_y.value(), 2)
        self.current_waypoints.append([x, y])
        self.waypoints_display.setText(f"Absolute: {str(self.current_waypoints)}")

    def clear_wps(self):
        self.current_waypoints.clear()
        self.waypoints_display.setText("")

    def launch_analytics_window(self):
        self.analytics_window = AnalyticsWindow(self.scenario_combo.currentText())
        self.analytics_window.show()

    def open_glossary(self):
        self.glossary_window = GlossaryDialog(self)
        self.glossary_window.show()

    def handle_launch(self):
        if self.ros_manager.is_running(): 
            QMessageBox.warning(self, 'Error', 'Experiment is already running.')
            return
        scenario = self.scenario_combo.currentText()
        node = self.custom_node if self.custom_node else self.logic_matrix[scenario]["node"]
        
        waypoints_str = ""
        if scenario == "Trajectory Planning" and self.current_waypoints:
            try:
                flat_list = []
                for pt in self.current_waypoints:
                    rel_x = round(pt[0] - ROBOT_START_X, 3)
                    rel_y = round(pt[1] - ROBOT_START_Y, 3)
                    flat_list.extend([rel_x, rel_y])
                
                waypoints_str = str(flat_list).replace(" ", "")
                print(f"[GUI MATH] Translated Absolute Webots points to Relative Node Points: {waypoints_str}")
            except Exception as e:
                print(f"Error parsing waypoints: {e}")

        if scenario == "Wall Following":
            kp, ki, kd = self.kp_box.value(), self.ki_box.value(), self.kd_box.value()
            pid_args_str = f" kp:={kp} ki:={ki} kd:={kd}"
        else:
            kp, ki, kd = None, None, None
            pid_args_str = ""

        robot_arg = f"robot_name:={self.robot_name_input.text()} " if self.robot_name_input.text().strip() else ""
        wp_arg = f"relative_waypoints:=\"{waypoints_str}\"" if waypoints_str else ""
        
        self.log_terminal(f"ros2 launch robot_simulation simulation.launch.py world:={self.world_combo.currentText()} scenario:={node} {robot_arg}{wp_arg}{pid_args_str}".strip())

        self.update_summary("● Running")
        self.ros_manager.launch(self.world_combo.currentText(), node, self.robot_name_input.text(), waypoints_str, kp, ki, kd)

    def handle_kill(self):
        if not self.ros_manager.kill_all():
            QMessageBox.warning(self, 'Error', 'No experiment currently running.')
        else:
            self.log_terminal("ros2 daemon stop")
            self.update_summary("● Stopped")

    def send_pid_update(self):
        if not self.ros_manager.is_running():
            self.pid_status_pill.set_state("stopped", "Queued (not running)")
            QMessageBox.information(self, "Gains Queued", "Experiment is not running yet. These gains are saved in the UI and will inject automatically when you hit Start.")
            return
        
        node = self.logic_matrix[self.scenario_combo.currentText()]["node"]
        robot = self.robot_name_input.text()
        
        target_node = f"{robot}/{node}" if robot.strip() else node
        target_node = target_node.strip('/')

        self.log_terminal(f"ros2 param set /{target_node} kp {self.kp_box.value()}")
        self.log_terminal(f"ros2 param set /{target_node} ki {self.ki_box.value()}")
        self.log_terminal(f"ros2 param set /{target_node} kd {self.kd_box.value()}")

        self.pid_status_pill.set_state("sending")
        proc = self.ros_manager.set_pid_params(node, robot, self.kp_box.value(), self.ki_box.value(), self.kd_box.value())
        self._watch_pid_send(proc)

    def _watch_pid_send(self, proc):
        """Polls the dispatched PID-set subprocess and updates the indicator once it completes."""
        if proc is None:
            self.pid_status_pill.set_state("ready", "Sent")
            return
        if proc.poll() is None:
            QTimer.singleShot(150, lambda: self._watch_pid_send(proc))
        else:
            if proc.returncode == 0:
                self.pid_status_pill.set_state("ready", "● Gains Sent")
            else:
                self.pid_status_pill.set_state("stopped", "Send Failed")

    def open_waypoint_map(self):
        map_dialog = InteractiveMapDialog(self.current_waypoints, self)
        if map_dialog.exec_() == QDialog.Accepted:
            self.current_waypoints = map_dialog.get_points()
            if self.current_waypoints:
                self.waypoints_display.setText(f"Absolute: {str(self.current_waypoints)}")
            else:
                self.waypoints_display.setText("")


# --- At the bottom of gui.py ---

def main(args=None):
    app = QApplication(sys.argv)
    app.setStyle("Fusion") 
    window = AFIT_GCS()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()