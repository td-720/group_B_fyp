# #!/usr/bin/env python3
# import sys
# import os
# import subprocess
# import signal
# import json
# import pandas as pd
# import numpy as np

# # --- GUI IMPORTS ---
# from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
#                              QComboBox, QPushButton, QTextEdit, QLabel, QMessageBox, 
#                              QFileDialog, QDoubleSpinBox, QGroupBox, QLineEdit, 
#                              QTableWidgetItem, QTableWidget, QHeaderView, QScrollArea,
#                              QDialog, QDialogButtonBox)
# from PyQt5.QtGui import QFont
# from PyQt5.QtCore import Qt

# # --- MATPLOTLIB IMPORTS ---
# from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
# from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
# from matplotlib.figure import Figure
# from matplotlib.patches import Rectangle


# # =====================================================================
# # --- CONFIGURATION & CONSTANTS ---
# # =====================================================================
# ROS2_SETUP_CMD = "source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash"
# DEFAULT_LOG_DIR = os.path.expanduser('~/ros2_ws/simulation_logs')

# # --- ROBOT STARTING POSITION (WEBOTS ABSOLUTE) ---
# ROBOT_START_X = -2.5
# ROBOT_START_Y = -2.5


# # =====================================================================
# # --- CORE LOGIC MODULES ---
# # =====================================================================

# class ROS2ProcessManager:
#     """Handles OS-level subprocess execution for ROS 2 commands."""
    
#     def __init__(self):
#         self.active_process = None

#     def is_running(self):
#         return self.active_process is not None

#     def launch(self, target_world, node, robot_name, waypoints_str=""):
#         robot_arg = f"robot_name:={robot_name}" if robot_name.strip() else ""
#         waypoint_arg = f" relative_waypoints:=\"{waypoints_str}\"" if waypoints_str else ""
#         launch_args = f"world:={target_world} scenario:={node} {robot_arg}{waypoint_arg}".strip()
#         full_cmd = f"{ROS2_SETUP_CMD} && ros2 launch robot_simulation simulation.launch.py {launch_args}"
        
#         print(f"IGNITION SEQUENCE INITIATED:\n{full_cmd}")
#         self.active_process = subprocess.Popen(
#             full_cmd, shell=True, executable='/bin/bash', preexec_fn=os.setsid
#         )

#     def kill_all(self):
#         if not self.is_running():
#             return False
            
#         print("\nTERMINATE: Initiating complete system sweep...")
#         try:
#             pgid = os.getpgid(self.active_process.pid)
#             os.killpg(pgid, signal.SIGTERM)
#             print(f"SUCCESS: Terminated core simulation group {pgid}")
#         except ProcessLookupError:
#             print("WARNING: Core simulation already down.")
#         finally:
#             self.active_process = None

#         print("SWEEP: Purging background terminal processes...")
#         try:
#             subprocess.run("pkill -f 'ros2 topic pub'", shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
#             global ROS2_SETUP_CMD
#             subprocess.run(f"{ROS2_SETUP_CMD} && ros2 daemon stop", shell=True, executable='/bin/bash', stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
#             print("SUCCESS: ROS 2 background discovery channels purged clean.\n")
#         except Exception as e:
#             print(f"WARNING: Automated sweep encountered an issue: {e}")

#         return True

#     def set_pid_params(self, node, robot_name, kp, ki, kd):
#         global ROS2_SETUP_CMD
#         target_node = f"{robot_name}/{node}" if robot_name.strip() else node
#         target_node = target_node.strip('/')
        
#         cmd = (f"({ROS2_SETUP_CMD} && ros2 param set /{target_node} Kp {kp}) & "
#                f"({ROS2_SETUP_CMD} && ros2 param set /{target_node} Ki {ki}) & "
#                f"({ROS2_SETUP_CMD} && ros2 param set /{target_node} Kd {kd})")
               
#         print(f"\n--- DISPATCHING ASYNC PID GAINS FOR /{target_node} ---")
#         subprocess.Popen(cmd, shell=True, executable='/bin/bash', stdout=subprocess.DEVNULL)

#     def publish_polynomial(self, poly_type):
#         global ROS2_SETUP_CMD
#         raw_cmd = f"{ROS2_SETUP_CMD} && ros2 topic pub --once /set_polynomial std_msgs/msg/String \"{{data: '{poly_type}'}}\""
#         subprocess.Popen(raw_cmd, shell=True, executable='/bin/bash', stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

#     def publish_waypoints(self, points):
#         if not self.is_running(): return False
            
#         global ROS2_SETUP_CMD
        
#         # Convert Absolute (Webots) back to Relative before publishing via topic
#         relative_points = [[pt[0] - ROBOT_START_X, pt[1] - ROBOT_START_Y] for pt in points]
#         rotated_points = [[round(pt[1], 3), round(-pt[0], 3)] for pt in relative_points]
        
#         points_str = json.dumps(rotated_points)
#         raw_cmd = f"{ROS2_SETUP_CMD} && ros2 topic pub --keep-alive 2 /gcs/via_points std_msgs/msg/String \"{{data: '{points_str}'}}\""
#         subprocess.Popen(raw_cmd, shell=True, executable='/bin/bash', stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
#         return True

# class SimulationAnalyzer:
#     @staticmethod
#     def calculate_wall_following_metrics(df):
#         time, errors = df['Time'].values, df['Error'].values
#         dt = np.mean(np.diff(time)) if len(time) > 1 else 0.1
        
#         rmse = np.sqrt(np.mean(errors**2))
#         iae = np.trapz(np.abs(errors), dx=dt)
#         ise = np.trapz(errors**2, dx=dt)
#         peak_err = np.max(np.abs(errors))
#         initial_err = abs(errors[0]) if len(errors) > 0 else 0.4
#         os_pct = ((peak_err - initial_err) / initial_err) * 100 if peak_err > initial_err else 0.0
#         return [iae, ise, os_pct, 0.0, rmse]


# # =====================================================================
# # --- GUI MODULES ---
# # =====================================================================

# class InteractiveMapDialog(QDialog):
#     def __init__(self, existing_points, parent=None):
#         super().__init__(parent)
#         self.setWindowTitle("Map Waypoint Selector")
#         self.resize(650, 650)
#         self.via_points = existing_points.copy()
        
#         layout = QVBoxLayout(self)
#         self.figure = Figure(figsize=(6, 6), dpi=100)
#         self.canvas = FigureCanvas(self.figure)
#         layout.addWidget(self.canvas)
        
#         self.coord_label = QLabel(f"Absolute Points: {self.via_points}" if self.via_points else "Click on the grid to add Webots absolute coordinates.")
#         self.coord_label.setAlignment(Qt.AlignCenter)
#         self.coord_label.setStyleSheet("font-weight: bold; font-size: 14px; padding: 10px;")
#         layout.addWidget(self.coord_label)
        
#         self.btn_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
#         self.clear_btn = QPushButton("Clear Points")
#         self.btn_box.addButton(self.clear_btn, QDialogButtonBox.ActionRole)
#         layout.addWidget(self.btn_box)
        
#         self.btn_box.accepted.connect(self.accept)
#         self.btn_box.rejected.connect(self.reject)
#         self.clear_btn.clicked.connect(self.clear_plot)
#         self.canvas.mpl_connect('button_press_event', self.on_click)
#         self.setup_plot()

#     def setup_plot(self):
#         self.figure.clear()
#         self.ax = self.figure.add_subplot(111)
#         self.ax.set_title("Interactive Arena Map (Webots 10x10 Arena)")
        
#         # Set bounds to encompass the new 10x10 area with a small buffer for visibility
#         self.ax.set_xlim(-5.5, 5.5) 
#         self.ax.set_ylim(-5.5, 5.5)
#         self.ax.set_xlabel("Webots X (meters)")
#         self.ax.set_ylabel("Webots Y (meters)")
#         self.ax.grid(True, linestyle='--', alpha=0.5)
#         self.ax.set_aspect('equal')
        
#         # 1. Draw the exact center of the Webots world (0,0)
#         self.ax.plot(0, 0, 'k+', markersize=15, markeredgewidth=2, label="Arena Center (0,0)")
        
#         # 2. Draw the solid physical walls of the 10x10 arena (from -5.0 to 5.0)
#         walls = Rectangle((-5.0, -5.0), 10.0, 10.0, fill=False, edgecolor='black', linewidth=3)
#         self.ax.add_patch(walls)

#         # Draw the robot physically where it starts in Webots
#         self.ax.plot(ROBOT_START_X, ROBOT_START_Y, 'gX', markersize=12, label=f"Robot Start ({ROBOT_START_X}, {ROBOT_START_Y})")
#         self.ax.legend(loc="upper right", fontsize='small')
        
#         # Draw existing points
#         for pt in self.via_points:
#             self.ax.plot(pt[0], pt[1], 'ro', markersize=8)
#         if len(self.via_points) > 1:
#             pts = np.array(self.via_points)
#             self.ax.plot(pts[:,0], pts[:,1], 'b-', alpha=0.5)
            
#         self.canvas.draw()

#     def on_click(self, event):
#         if event.inaxes != self.ax: return
#         x, y = round(event.xdata, 2), round(event.ydata, 2)
        
#         # Optional: Prevent clicks completely outside the physical 10x10 wall bounds
#         if x < -5.0 or x > 5.0 or y < -5.0 or y > 5.0:
#             return
            
#         self.via_points.append([x, y])
#         self.coord_label.setText(f"Absolute Points: {self.via_points}")
#         self.ax.plot(x, y, 'ro', markersize=8)
#         if len(self.via_points) > 1:
#             prev_point = self.via_points[-2]
#             self.ax.plot([prev_point[0], x], [prev_point[1], y], 'b-', alpha=0.5)
#         self.canvas.draw()

#     def clear_plot(self):
#         self.via_points.clear()
#         self.coord_label.setText("Click on the grid to add Webots absolute coordinates.")
#         self.setup_plot()
        
#     def get_points(self): return self.via_points


# class AnalyticsWindow(QWidget):
#     def __init__(self, scenario_name):
#         super().__init__()
#         self.scenario_name = scenario_name
#         self.setWindowTitle(f"Engineering Analysis - {self.scenario_name}")
#         self.resize(1050, 750) 
#         self.init_ui()

#     def init_ui(self):
#         self.outer_layout = QVBoxLayout(self)
#         self.outer_layout.setContentsMargins(0, 0, 0, 0)
        
#         self.scroll = QScrollArea()
#         self.scroll.setWidgetResizable(True)
#         self.scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        
#         self.content_container = QWidget()
#         self.page_layout = QVBoxLayout(self.content_container)
#         self.page_layout.setSpacing(20)
        
#         self.extract_btn = QPushButton(f"📂 Load {self.scenario_name} CSV Data")
#         self.extract_btn.setStyleSheet("background-color: #2980b9; color: white; font-weight: bold; padding: 15px; font-size: 14px;")
#         self.extract_btn.clicked.connect(self.load_and_plot_file)
#         self.page_layout.addWidget(self.extract_btn)

#         self.stats_label = QLabel("Simulation Log: Pending Selection")
#         self.stats_label.setFont(QFont("Courier", 11))
#         self.stats_label.setAlignment(Qt.AlignCenter)
#         self.stats_label.setStyleSheet("background-color: #ecf0f1; padding: 10px; border: 1px solid #bdc3c7;")
#         self.page_layout.addWidget(self.stats_label)

#         self.page_layout.addWidget(QLabel("📊 ACADEMIC PERFORMANCE INDICES:"))
#         self.metrics_table = QTableWidget(1, 5) 
#         self.metrics_table.setHorizontalHeaderLabels(["IAE (Area)", "ISE (Penalty)", "OS (%)", "Settling (s)", "RMSE (m)"])
#         self.metrics_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
#         self.metrics_table.setFixedHeight(85)
#         self.metrics_table.setStyleSheet("background-color: #f8f9fa; color: black; font-weight: bold;")
#         self.page_layout.addWidget(self.metrics_table)

#         self.page_layout.addWidget(QLabel("📈 VISUAL TRANSIENT RESPONSE:"))
#         self.figure = Figure(figsize=(10, 10), dpi=100)
#         self.canvas = FigureCanvas(self.figure)
#         self.toolbar = NavigationToolbar(self.canvas, self)
#         self.canvas.setMinimumHeight(800) 
        
#         self.page_layout.addWidget(self.toolbar)
#         self.page_layout.addWidget(self.canvas)
#         self.scroll.setWidget(self.content_container)
#         self.outer_layout.addWidget(self.scroll)

#     def load_and_plot_file(self):
#         csv_filename, _ = QFileDialog.getOpenFileName(self, f"Open {self.scenario_name} Log", DEFAULT_LOG_DIR, "CSV Files (*.csv)")
#         if not csv_filename: return

#         try:
#             df = pd.read_csv(csv_filename)
#             df.columns = df.columns.str.strip() # Fixes hidden spaces in CSV headers
#         except Exception as e:
#             QMessageBox.critical(self, "Read Error", f"Failed to parse CSV:\n{e}")
#             return
            
#         if self.scenario_name == "Wall Following":
#             self.plot_wall_following(df, csv_filename)
#         elif self.scenario_name == "Obstacle Avoidance":
#             self.plot_obstacle_avoidance(df, csv_filename)
#         elif self.scenario_name == "Trajectory Planning":
#             self.plot_trajectory_planning(df, csv_filename)

#         self.figure.tight_layout(pad=3.0)
#         self.canvas.draw()

#     def update_table(self, values):
#         for i, val in enumerate(values):
#             self.metrics_table.setItem(0, i, QTableWidgetItem(f"{val}"))

#     def plot_wall_following(self, df, path):
#         self.figure.clear()
#         metrics = SimulationAnalyzer.calculate_wall_following_metrics(df)
#         self.update_table([f"{m:.4f}" for m in metrics])
#         self.stats_label.setText(f"Wall Following Analysis | File: {os.path.basename(path)}")

#         time = df['Time'].values
#         errors = df['Error'].values
#         effort = df['Control_Effort'].values
        
#         ax_spatial = self.figure.add_subplot(221)
#         ax_err     = self.figure.add_subplot(222)
#         ax_effort  = self.figure.add_subplot(212)

#         if 'X' in df.columns and 'Y' in df.columns:
#             ax_spatial.plot(df['X'], df['Y'], color='black', linewidth=1.5, label='Robot Path')
#             ax_spatial.set(title="2D Spatial Cornering", xlabel="X (m)", ylabel="Y (m)")
#             ax_spatial.axis('equal')
#         else:
#             ax_spatial.text(0.5, 0.5, 'Missing X, Y in CSV', ha='center', va='center', color='red')
#             ax_spatial.set(title="2D Spatial Cornering")

#         ax_err.plot(time, errors, color='#1f77b4', linewidth=2)
#         ax_err.axhline(0, color='red', linestyle='--', alpha=0.5)
#         ax_err.set(title="Tracking Accuracy (e(t))", xlabel="Time (s)", ylabel="Error (m)")
#         ax_err.grid(True, linestyle=':', alpha=0.6)

#         ax_effort.step(time, effort, color='#2ca02c', linewidth=1.5)
#         ax_effort.set(title="Steering Actuation (Control Effort)", xlabel="Time (s)", ylabel="Steer (rad/s)")
#         ax_effort.grid(True, linestyle=':', alpha=0.6)

#     def plot_obstacle_avoidance(self, df, path):
#         self.figure.clear()
#         time, clearance, ang_vel = df['Time'].values, df['Clearance'].values, df['Angular_Vel'].values
#         min_clear = np.min(clearance)
#         self.stats_label.setText(f"🚧 OBSTACLE AVOIDANCE 🚧 | File: {os.path.basename(path)}")
#         self.update_table(["N/A", "N/A", "N/A", f"{min_clear:.4f}", f"{time[-1]:.1f}"])

#         ax1 = self.figure.add_subplot(211)
#         ax2 = self.figure.add_subplot(212)

#         ax1.plot(time, clearance, color='#e67e22', linewidth=2, label='Clearance')
#         ax1.axhline(0.2, color='red', linestyle='--', label='Safety Limit')
#         ax1.set(title="Proximity Metrics", ylabel="Dist (m)")
#         ax1.grid(True, linestyle=':', alpha=0.7)
        
#         ax2.plot(time, ang_vel, color='#9b59b6', label='Yaw Rate')
#         ax2.set(title="Maneuver Intensity", xlabel="Time (s)", ylabel="Rad/s")
#         ax2.grid(True, linestyle=':', alpha=0.7)

#     def plot_trajectory_planning(self, df, path):
#         self.figure.clear()
#         self.stats_label.setText(f"📐 Quintic Spline & Dynamic Pure Pursuit | File: {os.path.basename(path)}")

#         if df.empty:
#             QMessageBox.critical(self, "Empty File", "The CSV has headers but no data.")
#             return

#         required = ['Time', 'X', 'Y', 'Target_X', 'Target_Y', 'V', 'W', 'Ld', 'CTE']
#         if not all(col in df.columns for col in required):
#             QMessageBox.critical(self, "Data Error", f"CSV missing columns!\nRequires: {required}")
#             return

#         time = df['Time'].values
#         cte = df['CTE'].values

#         rmse_cte = np.sqrt(np.mean(cte**2))
#         max_cte = np.max(np.abs(cte))
#         self.update_table(["N/A", "N/A", "N/A", f"Max: {max_cte:.4f}m", f"RMSE: {rmse_cte:.4f}m"])

#         ax1 = self.figure.add_subplot(221)
#         ax2 = self.figure.add_subplot(222)
#         ax3 = self.figure.add_subplot(223)
#         ax4 = self.figure.add_subplot(224)

#         ax1.plot(df['Target_X'], df['Target_Y'], 'b--', alpha=0.6, label='Ideal')
#         ax1.plot(df['X'], df['Y'], 'g-', linewidth=2, label='Actual')
#         ax1.scatter(df['X'].iloc[0], df['Y'].iloc[0], color='green', s=80, label='Start', zorder=5)
#         ax1.scatter(df['X'].iloc[-1], df['Y'].iloc[-1], color='red', marker='X', s=80, label='Goal', zorder=5)
#         ax1.set_title("2D Spatial Path Tracking")
#         ax1.set_xlabel("X (m)"); ax1.set_ylabel("Y (m)")
#         ax1.axis('equal'); ax1.grid(True, linestyle=':', alpha=0.7); ax1.legend(loc='best', fontsize='small')

#         ax2.plot(time, cte, color='#e74c3c', linewidth=2, label='CTE')
#         ax2.axhline(0, color='black', linestyle='--', alpha=0.6)
#         ax2.fill_between(time, cte, 0, color='#e74c3c', alpha=0.1)
#         ax2.set_title("Kinematic Tracking Error (CTE)")
#         ax2.set_xlabel("Time (s)"); ax2.set_ylabel("Error (m)")
#         ax2.grid(True, linestyle=':', alpha=0.7); ax2.legend(loc='best')

#         ax3.plot(time, df['Ld'], color='purple', linewidth=2, label='Ld')
#         ax3.set_xlabel("Time (s)"); ax3.set_ylabel("Look-Ahead (m)", color='purple')
#         ax3.tick_params(axis='y', labelcolor='purple')
        
#         ax3_twin = ax3.twinx()
#         ax3_twin.plot(time, df['V'], color='orange', linestyle='--', linewidth=2, label='Velocity')
#         ax3_twin.set_ylabel("Velocity (m/s)", color='orange')
#         ax3_twin.tick_params(axis='y', labelcolor='orange')
#         ax3.set_title("Dynamic Look-Ahead Validation")
#         ax3.grid(True, linestyle=':', alpha=0.5)

#         ax4.plot(time, df['W'], color='#2ca02c', linewidth=1.5, label='Angular Vel (W)')
#         ax4.set_title("Steering Actuation Smoothness")
#         ax4.set_xlabel("Time (s)"); ax4.set_ylabel("Commanded rad/s")
#         ax4.grid(True, linestyle=':', alpha=0.7); ax4.legend(loc='best')


# class AFIT_GCS(QWidget):
#     def __init__(self):
#         super().__init__()
#         self.ros_manager = ROS2ProcessManager()
#         self.custom_node = ""
#         self.current_waypoints = [] # Keeps track of Absolute waypoints locally
        
#         self.logic_matrix = {
#             "Teleop": {
#                 "worlds": ["obstacle_course.wbt", "garden.wbt", "wall_following.wbt"], 
#                 "node": "teleop_node", 
#                 "desc": "MANUAL CONTROL", 
#                 "has_analytics": False
#             },
#             "Trajectory Planning": {
#                 "worlds": ["obstacle_course.wbt", "garden.wbt", "wall_following.wbt"], 
#                 "node": "niku_controller_viapoints", 
#                 "desc": "AUTONOMOUS: Niku Spline", 
#                 "has_analytics": True
#             },
#             "Wall Following": {
#                 "worlds": ["wall_following_1.wbt", "wall_following_2.wbt", "garden.wbt"], 
#                 "node": "wall_follower", 
#                 "desc": "AUTONOMOUS: IR Wall Hugging", 
#                 "has_analytics": True
#             },
#             "Obstacle Avoidance": {
#                 "worlds": ["boxes_dense.wbt", "obstacle_course.wbt", "garden.wbt", "wall_following.wbt"], 
#                 "node": "obstacle_avoidance", 
#                 "desc": "AUTONOMOUS: LiDAR Avoidance", 
#                 "has_analytics": True
#             }
#         }
#         self.init_ui()

#     def init_ui(self):
#         self.setStyleSheet("""
#             QWidget { background-color: #87CEEB; } 
#             QLabel { font-weight: bold; color: #2c3e50; }
#             QComboBox, QTextEdit, QLineEdit, QDoubleSpinBox { background-color: white; color: black; border-radius: 5px; }
#         """)

#         main_layout = QVBoxLayout(self)
#         self.scenario_combo = QComboBox()
#         self.scenario_combo.addItems(self.logic_matrix.keys())
#         self.world_combo = QComboBox()
        
#         self.desc_box = QTextEdit()
#         self.desc_box.setReadOnly(True)
#         self.desc_box.setMaximumHeight(80)

#         main_layout.addWidget(QLabel("1. SELECT SCENARIO:"))
#         main_layout.addWidget(self.scenario_combo)
#         main_layout.addWidget(QLabel("2. SELECT WORLD:"))
#         main_layout.addWidget(self.world_combo)
#         main_layout.addWidget(self.desc_box)

#         self.launch_btn = QPushButton("LAUNCH SIMULATION")
#         self.launch_btn.setStyleSheet("background-color: #27ae60; color: white; font-weight: bold; height: 40px;")
        
#         self.kill_btn = QPushButton("TERMINATE ALL PROCESSES")
#         self.kill_btn.setStyleSheet("background-color: #c0392b; color: white; font-weight: bold; height: 40px;")
        
#         main_layout.addWidget(self.launch_btn)
#         main_layout.addWidget(self.kill_btn)

#         self.upload_node_btn = QPushButton("Upload Custom Node")
#         self.clear_node_btn = QPushButton("Clear Custom Node")
#         main_layout.addWidget(self.upload_node_btn)
#         main_layout.addWidget(self.clear_node_btn)

#         self.robot_name_input = QLineEdit()
#         self.robot_name_input.setPlaceholderText("Robot Name in Webots (Default: robot_a)")
#         main_layout.addWidget(self.robot_name_input)

#         self.pid_panel = self.create_pid_panel()
#         self.traj_panel = self.create_traj_panel()
#         main_layout.addWidget(self.pid_panel)
#         main_layout.addWidget(self.traj_panel)

#         self.custom_plot_btn = QPushButton("Run Custom Analytics Script (.py)")
#         self.custom_plot_btn.setStyleSheet("background-color: #f39c12; color: white; font-weight: bold;")
#         main_layout.addWidget(self.custom_plot_btn)

#         self.open_dashboard_btn = QPushButton("OPEN ANALYTICS DASHBOARD")
#         main_layout.addWidget(self.open_dashboard_btn)

#         self.scenario_combo.currentTextChanged.connect(self.sync_dropdowns)
#         self.launch_btn.clicked.connect(self.handle_launch)
#         self.kill_btn.clicked.connect(self.handle_kill)
#         self.update_pid_btn.clicked.connect(self.send_pid_update)
#         self.upload_node_btn.clicked.connect(self.upload_node)
#         self.clear_node_btn.clicked.connect(self.clear_custom_node)
#         self.custom_plot_btn.clicked.connect(self.launch_custom_script)
#         self.open_dashboard_btn.clicked.connect(self.launch_analytics_window)
        
#         self.setWindowTitle("Robot Simulation and Control Tool")
#         self.resize(600, 550) 
#         self.sync_dropdowns()

#     def create_pid_panel(self):
#         panel = QGroupBox("Live PID Tuning")
#         layout = QHBoxLayout(panel)
#         self.kp_box, self.ki_box, self.kd_box = QDoubleSpinBox(), QDoubleSpinBox(), QDoubleSpinBox()
        
#         for box in (self.kp_box, self.ki_box, self.kd_box):
#             box.setDecimals(3)
#             box.setRange(0.0, 10.0)
#             box.setSingleStep(0.05)
#             box.setValue(1.0)
            
#         self.update_pid_btn = QPushButton("UPDATE GAINS")
        
#         layout.addWidget(QLabel("Kp:")); layout.addWidget(self.kp_box)
#         layout.addWidget(QLabel("Ki:")); layout.addWidget(self.ki_box)
#         layout.addWidget(QLabel("Kd:")); layout.addWidget(self.kd_box)
#         layout.addWidget(self.update_pid_btn)
#         return panel

#     def create_traj_panel(self):
#         panel = QGroupBox("Trajectory Math & Webots Coordinates")
#         layout = QVBoxLayout(panel)
        
#         # Row 1: Interactive Map Option
#         row1 = QHBoxLayout()
#         self.open_map_btn = QPushButton("🗺️ OPEN WEBOTS MAP SELECTOR")
#         self.open_map_btn.setStyleSheet("background-color: #e67e22; color: white; font-weight: bold;")
#         self.open_map_btn.clicked.connect(self.open_waypoint_map)
#         row1.addWidget(self.open_map_btn)
#         layout.addLayout(row1)

#         # Row 2: Manual Data Entry Input (Now mapped to 10x10 Webots bounds)
#         row2 = QHBoxLayout()
#         self.wp_x = QDoubleSpinBox()
#         self.wp_x.setRange(-5.0, 5.0) # Matches 10x10 Webots boundaries
#         self.wp_x.setSingleStep(0.25)
        
#         self.wp_y = QDoubleSpinBox()
#         self.wp_y.setRange(-5.0, 5.0) # Matches 10x10 Webots boundaries
#         self.wp_y.setSingleStep(0.25)

#         self.add_wp_btn = QPushButton("+ Add Pt")
#         self.add_wp_btn.setStyleSheet("background-color: #27ae60; color: white; font-weight: bold;")
#         self.add_wp_btn.clicked.connect(self.manual_add_wp)

#         self.clear_wp_btn = QPushButton("Clear All")
#         self.clear_wp_btn.setStyleSheet("background-color: #c0392b; color: white; font-weight: bold;")
#         self.clear_wp_btn.clicked.connect(self.clear_wps)

#         row2.addWidget(QLabel("Webots X:"))
#         row2.addWidget(self.wp_x)
#         row2.addWidget(QLabel("Webots Y:"))
#         row2.addWidget(self.wp_y)
#         row2.addWidget(self.add_wp_btn)
#         row2.addWidget(self.clear_wp_btn)
#         layout.addLayout(row2)

#         # Row 3: Output Display
#         self.waypoints_display = QLineEdit()
#         self.waypoints_display.setReadOnly(True)
#         self.waypoints_display.setPlaceholderText("No Webots absolute points added yet...")
#         layout.addWidget(self.waypoints_display)

#         return panel

#     def manual_add_wp(self):
#         x = round(self.wp_x.value(), 2)
#         y = round(self.wp_y.value(), 2)
#         self.current_waypoints.append([x, y])
#         self.waypoints_display.setText(f"Absolute: {str(self.current_waypoints)}")

#     def clear_wps(self):
#         self.current_waypoints.clear()
#         self.waypoints_display.setText("")

#     def sync_dropdowns(self):
#         scenario = self.scenario_combo.currentText()
#         data = self.logic_matrix[scenario]
        
#         self.world_combo.clear()
#         self.world_combo.addItems(data["worlds"])
#         self.desc_box.setText(data["desc"])

#         is_traj = (scenario == "Trajectory Planning")
#         self.traj_panel.setVisible(is_traj)
#         self.pid_panel.setVisible(not is_traj)

#         if data["has_analytics"]:
#             self.open_dashboard_btn.setEnabled(True)
#             self.open_dashboard_btn.setStyleSheet("background-color: #8e44ad; color: white; font-weight: bold; height: 50px;")
#             self.open_dashboard_btn.setText("OPEN ANALYTICS DASHBOARD")
#         else:
#             self.open_dashboard_btn.setEnabled(False)
#             self.open_dashboard_btn.setStyleSheet("background-color: #95a5a6; color: #ecf0f1; font-weight: bold; height: 50px;")
#             self.open_dashboard_btn.setText("Analytics Disabled")

#     def upload_node(self):
#         file_path, _ = QFileDialog.getOpenFileName(self, "Select Node", "", "Python File (*.py)")
#         if file_path:
#             self.custom_node = file_path
#             self.upload_node_btn.setText(f"CUSTOM NODE: {os.path.basename(file_path)}")
#             self.upload_node_btn.setStyleSheet("background-color: #2ecc71; color: white;")
#             QMessageBox.information(self, "Custom Node Active", "Script overriding standard node.")

#     def clear_custom_node(self):
#         self.custom_node = ""
#         self.upload_node_btn.setText("Upload Custom Node")
#         self.upload_node_btn.setStyleSheet("")

#     def launch_analytics_window(self):
#         self.analytics_window = AnalyticsWindow(self.scenario_combo.currentText())
#         self.analytics_window.show()

#     def handle_launch(self):
#         if self.ros_manager.is_running(): 
#             QMessageBox.warning(self, 'Error', 'Simulation already running.')
#             return
#         scenario = self.scenario_combo.currentText()
#         node = self.custom_node if self.custom_node else self.logic_matrix[scenario]["node"]
        
#         waypoints_str = ""
#         if scenario == "Trajectory Planning" and self.current_waypoints:
#             try:
#                 # The node expects RELATIVE points from the start position.
#                 # We calculate: relative_point = absolute_webots_point - robot_start_point
#                 flat_list = []
#                 for pt in self.current_waypoints:
#                     rel_x = round(pt[0] - ROBOT_START_X, 3)
#                     rel_y = round(pt[1] - ROBOT_START_Y, 3)
#                     flat_list.extend([rel_x, rel_y])
                
#                 waypoints_str = str(flat_list).replace(" ", "")
#                 print(f"[GUI MATH] Translated Absolute Webots points to Relative Node Points: {waypoints_str}")
#             except Exception as e:
#                 print(f"Error parsing waypoints: {e}")

#         self.ros_manager.launch(self.world_combo.currentText(), node, self.robot_name_input.text(), waypoints_str)

#     def handle_kill(self):
#         if not self.ros_manager.kill_all():
#             QMessageBox.warning(self, 'Error', 'No process running.')

#     def send_pid_update(self):
#         if not self.ros_manager.is_running():
#             QMessageBox.warning(self, "Error", "Start a simulation first!")
#             return
#         node = self.logic_matrix[self.scenario_combo.currentText()]["node"]
#         self.ros_manager.set_pid_params(node, self.robot_name_input.text(), self.kp_box.value(), self.ki_box.value(), self.kd_box.value())

#     def open_waypoint_map(self):
#         map_dialog = InteractiveMapDialog(self.current_waypoints, self)
#         if map_dialog.exec_() == QDialog.Accepted:
#             self.current_waypoints = map_dialog.get_points()
#             if self.current_waypoints:
#                 self.waypoints_display.setText(f"Absolute: {str(self.current_waypoints)}")
#             else:
#                 self.waypoints_display.setText("")

#     def launch_custom_script(self):
#         file_path, _ = QFileDialog.getOpenFileName(self, "Select Script", "", "Python Files (*.py)")
#         if file_path:
#             subprocess.Popen(['python3', file_path], cwd=os.path.dirname(file_path))


# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     window = AFIT_GCS()
#     window.show()
#     sys.exit(app.exec_())






























#upgrade
# #!/usr/bin/env python3
# import sys
# import os
# import subprocess
# import signal
# import json
# import pandas as pd
# import numpy as np

# # --- GUI IMPORTS ---
# from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
#                              QComboBox, QPushButton, QTextEdit, QLabel, QMessageBox, 
#                              QFileDialog, QDoubleSpinBox, QGroupBox, QLineEdit, 
#                              QTableWidgetItem, QTableWidget, QHeaderView, QScrollArea,
#                              QDialog, QDialogButtonBox, QFrame)
# from PyQt5.QtGui import QFont, QPalette, QColor
# from PyQt5.QtCore import Qt

# # --- MATPLOTLIB IMPORTS ---
# from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
# from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
# from matplotlib.figure import Figure
# from matplotlib.patches import Rectangle


# # =====================================================================
# # --- CONFIGURATION & CONSTANTS ---
# # =====================================================================
# ROS2_SETUP_CMD = "source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash"
# DEFAULT_LOG_DIR = os.path.expanduser('~/ros2_ws/simulation_logs')

# # --- ROBOT STARTING POSITION (WEBOTS ABSOLUTE) ---
# ROBOT_START_X = -2.5
# ROBOT_START_Y = -2.5


# # =====================================================================
# # --- CORE LOGIC MODULES ---
# # =====================================================================

# class ROS2ProcessManager:
#     """Handles OS-level subprocess execution for ROS 2 commands."""
    
#     def __init__(self):
#         self.active_process = None

#     def is_running(self):
#         return self.active_process is not None

#     def launch(self, target_world, node, robot_name, waypoints_str=""):
#         robot_arg = f"robot_name:={robot_name}" if robot_name.strip() else ""
#         waypoint_arg = f" relative_waypoints:=\"{waypoints_str}\"" if waypoints_str else ""
#         launch_args = f"world:={target_world} scenario:={node} {robot_arg}{waypoint_arg}".strip()
#         full_cmd = f"{ROS2_SETUP_CMD} && ros2 launch robot_simulation simulation.launch.py {launch_args}"
        
#         print(f"IGNITION SEQUENCE INITIATED:\n{full_cmd}")
#         self.active_process = subprocess.Popen(
#             full_cmd, shell=True, executable='/bin/bash', preexec_fn=os.setsid
#         )

#     def kill_all(self):
#         if not self.is_running():
#             return False
            
#         print("\nTERMINATE: Initiating complete system sweep...")
#         try:
#             pgid = os.getpgid(self.active_process.pid)
#             os.killpg(pgid, signal.SIGTERM)
#             print(f"SUCCESS: Terminated core simulation group {pgid}")
#         except ProcessLookupError:
#             print("WARNING: Core simulation already down.")
#         finally:
#             self.active_process = None

#         print("SWEEP: Purging background terminal processes...")
#         try:
#             subprocess.run("pkill -f 'ros2 topic pub'", shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
#             global ROS2_SETUP_CMD
#             subprocess.run(f"{ROS2_SETUP_CMD} && ros2 daemon stop", shell=True, executable='/bin/bash', stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
#             print("SUCCESS: ROS 2 background discovery channels purged clean.\n")
#         except Exception as e:
#             print(f"WARNING: Automated sweep encountered an issue: {e}")

#         return True

#     def set_pid_params(self, node, robot_name, kp, ki, kd):
#         global ROS2_SETUP_CMD
#         target_node = f"{robot_name}/{node}" if robot_name.strip() else node
#         target_node = target_node.strip('/')
        
#         cmd = (f"({ROS2_SETUP_CMD} && ros2 param set /{target_node} Kp {kp}) & "
#                f"({ROS2_SETUP_CMD} && ros2 param set /{target_node} Ki {ki}) & "
#                f"({ROS2_SETUP_CMD} && ros2 param set /{target_node} Kd {kd})")
               
#         print(f"\n--- DISPATCHING ASYNC PID GAINS FOR /{target_node} ---")
#         subprocess.Popen(cmd, shell=True, executable='/bin/bash', stdout=subprocess.DEVNULL)

#     def publish_polynomial(self, poly_type):
#         global ROS2_SETUP_CMD
#         raw_cmd = f"{ROS2_SETUP_CMD} && ros2 topic pub --once /set_polynomial std_msgs/msg/String \"{{data: '{poly_type}'}}\""
#         subprocess.Popen(raw_cmd, shell=True, executable='/bin/bash', stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

#     def publish_waypoints(self, points):
#         if not self.is_running(): return False
            
#         global ROS2_SETUP_CMD
        
#         # Convert Absolute (Webots) back to Relative before publishing via topic
#         relative_points = [[pt[0] - ROBOT_START_X, pt[1] - ROBOT_START_Y] for pt in points]
#         rotated_points = [[round(pt[1], 3), round(-pt[0], 3)] for pt in relative_points]
        
#         points_str = json.dumps(rotated_points)
#         raw_cmd = f"{ROS2_SETUP_CMD} && ros2 topic pub --keep-alive 2 /gcs/via_points std_msgs/msg/String \"{{data: '{points_str}'}}\""
#         subprocess.Popen(raw_cmd, shell=True, executable='/bin/bash', stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
#         return True

# class SimulationAnalyzer:
#     @staticmethod
#     def calculate_wall_following_metrics(df):
#         time, errors = df['Time'].values, df['Error'].values
#         dt = np.mean(np.diff(time)) if len(time) > 1 else 0.1
        
#         rmse = np.sqrt(np.mean(errors**2))
#         iae = np.trapz(np.abs(errors), dx=dt)
#         ise = np.trapz(errors**2, dx=dt)
#         peak_err = np.max(np.abs(errors))
#         initial_err = abs(errors[0]) if len(errors) > 0 else 0.4
#         os_pct = ((peak_err - initial_err) / initial_err) * 100 if peak_err > initial_err else 0.0
#         return [iae, ise, os_pct, 0.0, rmse]


# # =====================================================================
# # --- GUI MODULES ---
# # =====================================================================

# class InteractiveMapDialog(QDialog):
#     def __init__(self, existing_points, parent=None):
#         super().__init__(parent)
#         self.setWindowTitle("Map Waypoint Selector")
#         self.resize(650, 650)
#         self.via_points = existing_points.copy()
        
#         layout = QVBoxLayout(self)
#         self.figure = Figure(figsize=(6, 6), dpi=100)
#         self.canvas = FigureCanvas(self.figure)
#         layout.addWidget(self.canvas)
        
#         self.coord_label = QLabel(f"Absolute Points: {self.via_points}" if self.via_points else "Click on the grid to add Webots absolute coordinates.")
#         self.coord_label.setAlignment(Qt.AlignCenter)
#         self.coord_label.setStyleSheet("font-weight: bold; font-size: 14px; padding: 10px;")
#         layout.addWidget(self.coord_label)
        
#         self.btn_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
#         self.clear_btn = QPushButton("Clear Points")
#         self.btn_box.addButton(self.clear_btn, QDialogButtonBox.ActionRole)
#         layout.addWidget(self.btn_box)
        
#         self.btn_box.accepted.connect(self.accept)
#         self.btn_box.rejected.connect(self.reject)
#         self.clear_btn.clicked.connect(self.clear_plot)
#         self.canvas.mpl_connect('button_press_event', self.on_click)
#         self.setup_plot()

#     def setup_plot(self):
#         self.figure.clear()
#         self.ax = self.figure.add_subplot(111)
#         self.ax.set_title("Interactive Arena Map (Webots 10x10 Arena)")
        
#         # Set bounds to encompass the new 10x10 area with a small buffer for visibility
#         self.ax.set_xlim(-5.5, 5.5) 
#         self.ax.set_ylim(-5.5, 5.5)
#         self.ax.set_xlabel("Webots X (meters)")
#         self.ax.set_ylabel("Webots Y (meters)")
#         self.ax.grid(True, linestyle='--', alpha=0.5)
#         self.ax.set_aspect('equal')
        
#         # 1. Draw the exact center of the Webots world (0,0)
#         self.ax.plot(0, 0, 'k+', markersize=15, markeredgewidth=2, label="Arena Center (0,0)")
        
#         # 2. Draw the solid physical walls of the 10x10 arena (from -5.0 to 5.0)
#         walls = Rectangle((-5.0, -5.0), 10.0, 10.0, fill=False, edgecolor='black', linewidth=3)
#         self.ax.add_patch(walls)

#         # Draw the robot physically where it starts in Webots
#         self.ax.plot(ROBOT_START_X, ROBOT_START_Y, 'gX', markersize=12, label=f"Robot Start ({ROBOT_START_X}, {ROBOT_START_Y})")
#         self.ax.legend(loc="upper right", fontsize='small')
        
#         # Draw existing points
#         for pt in self.via_points:
#             self.ax.plot(pt[0], pt[1], 'ro', markersize=8)
#         if len(self.via_points) > 1:
#             pts = np.array(self.via_points)
#             self.ax.plot(pts[:,0], pts[:,1], 'b-', alpha=0.5)
            
#         self.canvas.draw()

#     def on_click(self, event):
#         if event.inaxes != self.ax: return
#         x, y = round(event.xdata, 2), round(event.ydata, 2)
        
#         # Optional: Prevent clicks completely outside the physical 10x10 wall bounds
#         if x < -5.0 or x > 5.0 or y < -5.0 or y > 5.0:
#             return
            
#         self.via_points.append([x, y])
#         self.coord_label.setText(f"Absolute Points: {self.via_points}")
#         self.ax.plot(x, y, 'ro', markersize=8)
#         if len(self.via_points) > 1:
#             prev_point = self.via_points[-2]
#             self.ax.plot([prev_point[0], x], [prev_point[1], y], 'b-', alpha=0.5)
#         self.canvas.draw()

#     def clear_plot(self):
#         self.via_points.clear()
#         self.coord_label.setText("Click on the grid to add Webots absolute coordinates.")
#         self.setup_plot()
        
#     def get_points(self): return self.via_points


# class AnalyticsWindow(QWidget):
#     def __init__(self, scenario_name):
#         super().__init__()
#         self.scenario_name = scenario_name
#         self.setWindowTitle(f"Performance Evaluation Dashboard - {self.scenario_name}")
#         self.resize(1050, 750) 
#         self.init_ui()

#     def init_ui(self):
#         self.outer_layout = QVBoxLayout(self)
#         self.outer_layout.setContentsMargins(0, 0, 0, 0)
        
#         self.scroll = QScrollArea()
#         self.scroll.setWidgetResizable(True)
#         self.scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        
#         self.content_container = QWidget()
#         self.page_layout = QVBoxLayout(self.content_container)
#         self.page_layout.setSpacing(20)
        
#         self.extract_btn = QPushButton(f"📂 Load {self.scenario_name} Experiment Data")
#         self.extract_btn.setStyleSheet("background-color: #2980b9; color: white; font-weight: bold; padding: 15px; font-size: 14px;")
#         self.extract_btn.clicked.connect(self.load_and_plot_file)
#         self.page_layout.addWidget(self.extract_btn)

#         self.stats_label = QLabel("Experiment Log: Pending Selection")
#         self.stats_label.setFont(QFont("Courier", 11))
#         self.stats_label.setAlignment(Qt.AlignCenter)
#         self.stats_label.setStyleSheet("background-color: #ecf0f1; padding: 10px; border: 1px solid #bdc3c7;")
#         self.page_layout.addWidget(self.stats_label)

#         self.page_layout.addWidget(QLabel("📊 ENGINEERING PERFORMANCE INDICES:"))
#         self.metrics_table = QTableWidget(1, 5) 
#         self.metrics_table.setHorizontalHeaderLabels(["IAE (Area)", "ISE (Penalty)", "OS (%)", "Settling (s)", "RMSE (m)"])
#         self.metrics_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
#         self.metrics_table.setFixedHeight(85)
#         self.metrics_table.setStyleSheet("background-color: #f8f9fa; color: black; font-weight: bold;")
#         self.page_layout.addWidget(self.metrics_table)

#         self.page_layout.addWidget(QLabel("📈 EXPERIMENT RESPONSE PLOTS:"))
#         self.figure = Figure(figsize=(10, 10), dpi=100)
#         self.canvas = FigureCanvas(self.figure)
#         self.toolbar = NavigationToolbar(self.canvas, self)
#         self.canvas.setMinimumHeight(800) 
        
#         self.page_layout.addWidget(self.toolbar)
#         self.page_layout.addWidget(self.canvas)
#         self.scroll.setWidget(self.content_container)
#         self.outer_layout.addWidget(self.scroll)

#     def load_and_plot_file(self):
#         csv_filename, _ = QFileDialog.getOpenFileName(self, f"Open {self.scenario_name} Log", DEFAULT_LOG_DIR, "CSV Files (*.csv)")
#         if not csv_filename: return

#         try:
#             df = pd.read_csv(csv_filename)
#             df.columns = df.columns.str.strip() # Fixes hidden spaces in CSV headers
#         except Exception as e:
#             QMessageBox.critical(self, "Read Error", f"Failed to parse CSV:\n{e}")
#             return
            
#         if self.scenario_name == "Wall Following":
#             self.plot_wall_following(df, csv_filename)
#         elif self.scenario_name == "Obstacle Avoidance":
#             self.plot_obstacle_avoidance(df, csv_filename)
#         elif self.scenario_name == "Trajectory Planning":
#             self.plot_trajectory_planning(df, csv_filename)

#         self.figure.tight_layout(pad=3.0)
#         self.canvas.draw()

#     def update_table(self, values):
#         for i, val in enumerate(values):
#             self.metrics_table.setItem(0, i, QTableWidgetItem(f"{val}"))

#     def plot_wall_following(self, df, path):
#         self.figure.clear()
#         metrics = SimulationAnalyzer.calculate_wall_following_metrics(df)
#         self.update_table([f"{m:.4f}" for m in metrics])
#         self.stats_label.setText(f"Wall Following Analysis | File: {os.path.basename(path)}")

#         time = df['Time'].values
#         errors = df['Error'].values
#         effort = df['Control_Effort'].values
        
#         ax_spatial = self.figure.add_subplot(221)
#         ax_err     = self.figure.add_subplot(222)
#         ax_effort  = self.figure.add_subplot(212)

#         if 'X' in df.columns and 'Y' in df.columns:
#             ax_spatial.plot(df['X'], df['Y'], color='black', linewidth=1.5, label='Robot Path')
#             ax_spatial.set(title="2D Spatial Cornering", xlabel="X (m)", ylabel="Y (m)")
#             ax_spatial.axis('equal')
#         else:
#             ax_spatial.text(0.5, 0.5, 'Missing X, Y in CSV', ha='center', va='center', color='red')
#             ax_spatial.set(title="2D Spatial Cornering")

#         ax_err.plot(time, errors, color='#1f77b4', linewidth=2)
#         ax_err.axhline(0, color='red', linestyle='--', alpha=0.5)
#         ax_err.set(title="Tracking Accuracy (e(t))", xlabel="Time (s)", ylabel="Error (m)")
#         ax_err.grid(True, linestyle=':', alpha=0.6)

#         ax_effort.step(time, effort, color='#2ca02c', linewidth=1.5)
#         ax_effort.set(title="Steering Actuation (Control Effort)", xlabel="Time (s)", ylabel="Steer (rad/s)")
#         ax_effort.grid(True, linestyle=':', alpha=0.6)

#     def plot_obstacle_avoidance(self, df, path):
#         self.figure.clear()
#         time, clearance, ang_vel = df['Time'].values, df['Clearance'].values, df['Angular_Vel'].values
#         min_clear = np.min(clearance)
#         self.stats_label.setText(f"🚧 OBSTACLE AVOIDANCE 🚧 | File: {os.path.basename(path)}")
#         self.update_table(["N/A", "N/A", "N/A", f"{min_clear:.4f}", f"{time[-1]:.1f}"])

#         ax1 = self.figure.add_subplot(211)
#         ax2 = self.figure.add_subplot(212)

#         ax1.plot(time, clearance, color='#e67e22', linewidth=2, label='Clearance')
#         ax1.axhline(0.2, color='red', linestyle='--', label='Safety Limit')
#         ax1.set(title="Proximity Metrics", ylabel="Dist (m)")
#         ax1.grid(True, linestyle=':', alpha=0.7)
        
#         ax2.plot(time, ang_vel, color='#9b59b6', label='Yaw Rate')
#         ax2.set(title="Maneuver Intensity", xlabel="Time (s)", ylabel="Rad/s")
#         ax2.grid(True, linestyle=':', alpha=0.7)

#     def plot_trajectory_planning(self, df, path):
#         self.figure.clear()
#         self.stats_label.setText(f"📐 Quintic Spline & Dynamic Pure Pursuit | File: {os.path.basename(path)}")

#         if df.empty:
#             QMessageBox.critical(self, "Empty File", "The CSV has headers but no data.")
#             return

#         required = ['Time', 'X', 'Y', 'Target_X', 'Target_Y', 'V', 'W', 'Ld', 'CTE']
#         if not all(col in df.columns for col in required):
#             QMessageBox.critical(self, "Data Error", f"CSV missing columns!\nRequires: {required}")
#             return

#         time = df['Time'].values
#         cte = df['CTE'].values

#         rmse_cte = np.sqrt(np.mean(cte**2))
#         max_cte = np.max(np.abs(cte))
#         self.update_table(["N/A", "N/A", "N/A", f"Max: {max_cte:.4f}m", f"RMSE: {rmse_cte:.4f}m"])

#         ax1 = self.figure.add_subplot(221)
#         ax2 = self.figure.add_subplot(222)
#         ax3 = self.figure.add_subplot(223)
#         ax4 = self.figure.add_subplot(224)

#         ax1.plot(df['Target_X'], df['Target_Y'], 'b--', alpha=0.6, label='Ideal')
#         ax1.plot(df['X'], df['Y'], 'g-', linewidth=2, label='Actual')
#         ax1.scatter(df['X'].iloc[0], df['Y'].iloc[0], color='green', s=80, label='Start', zorder=5)
#         ax1.scatter(df['X'].iloc[-1], df['Y'].iloc[-1], color='red', marker='X', s=80, label='Goal', zorder=5)
#         ax1.set_title("2D Spatial Path Tracking")
#         ax1.set_xlabel("X (m)"); ax1.set_ylabel("Y (m)")
#         ax1.axis('equal'); ax1.grid(True, linestyle=':', alpha=0.7); ax1.legend(loc='best', fontsize='small')

#         ax2.plot(time, cte, color='#e74c3c', linewidth=2, label='CTE')
#         ax2.axhline(0, color='black', linestyle='--', alpha=0.6)
#         ax2.fill_between(time, cte, 0, color='#e74c3c', alpha=0.1)
#         ax2.set_title("Kinematic Tracking Error (CTE)")
#         ax2.set_xlabel("Time (s)"); ax2.set_ylabel("Error (m)")
#         ax2.grid(True, linestyle=':', alpha=0.7); ax2.legend(loc='best')

#         ax3.plot(time, df['Ld'], color='purple', linewidth=2, label='Ld')
#         ax3.set_xlabel("Time (s)"); ax3.set_ylabel("Look-Ahead (m)", color='purple')
#         ax3.tick_params(axis='y', labelcolor='purple')
        
#         ax3_twin = ax3.twinx()
#         ax3_twin.plot(time, df['V'], color='orange', linestyle='--', linewidth=2, label='Velocity')
#         ax3_twin.set_ylabel("Velocity (m/s)", color='orange')
#         ax3_twin.tick_params(axis='y', labelcolor='orange')
#         ax3.set_title("Dynamic Look-Ahead Validation")
#         ax3.grid(True, linestyle=':', alpha=0.5)

#         ax4.plot(time, df['W'], color='#2ca02c', linewidth=1.5, label='Angular Vel (W)')
#         ax4.set_title("Steering Actuation Smoothness")
#         ax4.set_xlabel("Time (s)"); ax4.set_ylabel("Commanded rad/s")
#         ax4.grid(True, linestyle=':', alpha=0.7); ax4.legend(loc='best')


# class AFIT_GCS(QWidget):
#     def __init__(self):
#         super().__init__()
#         self.ros_manager = ROS2ProcessManager()
#         self.custom_node = ""
#         self.current_waypoints = []
        
#         # Updated logic matrix with descriptions matching experiment objectives
#         self.logic_matrix = {
#             "Teleop": {
#                 "worlds": ["obstacle_course.wbt", "garden.wbt", "wall_following.wbt"], 
#                 "node": "teleop_node", 
#                 "desc": "Objective: Investigate differential-drive robot motion under manual keyboard control and observe its kinematic response.", 
#                 "controller": "Manual Control",
#                 "has_analytics": False
#             },
#             "Trajectory Planning": {
#                 "worlds": ["obstacle_course.wbt", "garden.wbt", "wall_following.wbt"], 
#                 "node": "niku_controller_viapoints", 
#                 "desc": "Objective: Evaluate autonomous trajectory generation and Pure Pursuit path tracking using user-defined waypoints.", 
#                 "controller": "Pure Pursuit Spline",
#                 "has_analytics": True
#             },
#             "Wall Following": {
#                 "worlds": ["wall_following_1.wbt", "wall_following_2.wbt", "garden.wbt"], 
#                 "node": "wall_follower", 
#                 "desc": "Objective: Evaluate the performance of a PID controller in maintaining a desired distance from a wall through real-time gain adjustment.", 
#                 "controller": "PID Control",
#                 "has_analytics": True
#             },
#             "Obstacle Avoidance": {
#                 "worlds": ["boxes_dense.wbt", "obstacle_course.wbt", "garden.wbt", "wall_following.wbt"], 
#                 "node": "obstacle_avoidance", 
#                 "desc": "Objective: Evaluate real-time LiDAR-based obstacle detection and reactive maneuver intensity.", 
#                 "controller": "Reactive Logic",
#                 "has_analytics": True
#             }
#         }
#         self.init_ui()

#     def init_ui(self):
#         self.setWindowTitle("Integrated Mobile Robot Experimentation Platform")
#         self.resize(650, 750) 
        
#         self.setStyleSheet("""
#             QWidget { background-color: #ECF0F1; } 
#             QLabel { font-weight: bold; color: #2C3E50; }
#             QGroupBox { font-weight: bold; border: 2px solid #BDC3C7; border-radius: 5px; margin-top: 1ex; }
#             QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; padding: 0 3px; }
#             QComboBox, QTextEdit, QLineEdit, QDoubleSpinBox { background-color: white; color: black; border: 1px solid #BDC3C7; border-radius: 3px; padding: 2px;}
#         """)

#         main_layout = QVBoxLayout(self)

#         # Main Title Header
#         title_lbl = QLabel("INTEGRATED MOBILE ROBOT EXPERIMENTATION PLATFORM")
#         title_lbl.setAlignment(Qt.AlignCenter)
#         title_lbl.setStyleSheet("font-size: 16px; background-color: #34495E; color: white; padding: 10px; border-radius: 5px;")
#         main_layout.addWidget(title_lbl)

#         # Build Platform Sections
#         main_layout.addWidget(self.create_summary_panel())
#         main_layout.addWidget(self.create_configuration_panel())
#         main_layout.addWidget(self.create_execution_panel())
#         main_layout.addWidget(self.create_evaluation_panel())
        
#         self.sync_dropdowns()

#     # --- UI COMPONENT BUILDERS ---

#     def create_summary_panel(self):
#         panel = QGroupBox("CURRENT EXPERIMENT SUMMARY")
#         layout = QVBoxLayout(panel)
        
#         self.sum_exp_lbl = QLabel("Experiment : ")
#         self.sum_world_lbl = QLabel("World      : ")
#         self.sum_ctrl_lbl = QLabel("Controller : ")
#         self.sum_status_lbl = QLabel("Status     : ● Ready")
#         self.sum_status_lbl.setStyleSheet("color: #27AE60;") # Green for ready
        
#         for lbl in [self.sum_exp_lbl, self.sum_world_lbl, self.sum_ctrl_lbl, self.sum_status_lbl]:
#             lbl.setFont(QFont("Courier", 10))
#             layout.addWidget(lbl)
            
#         return panel

#     def create_configuration_panel(self):
#         panel = QGroupBox("1. EXPERIMENT CONFIGURATION")
#         layout = QVBoxLayout(panel)
        
#         # Selectors
#         row_selectors = QHBoxLayout()
#         self.scenario_combo = QComboBox()
#         self.scenario_combo.addItems(self.logic_matrix.keys())
#         self.world_combo = QComboBox()
        
#         row_selectors.addWidget(QLabel("Experiment Type:"))
#         row_selectors.addWidget(self.scenario_combo, stretch=2)
#         row_selectors.addWidget(QLabel("Webots World:"))
#         row_selectors.addWidget(self.world_combo, stretch=2)
#         layout.addLayout(row_selectors)

#         # Objective Description
#         layout.addWidget(QLabel("Experiment Objective"))
#         self.desc_box = QTextEdit()
#         self.desc_box.setReadOnly(True)
#         self.desc_box.setMaximumHeight(60)
#         self.desc_box.setStyleSheet("background-color: #F8F9FA; font-style: italic;")
#         layout.addWidget(self.desc_box)
        
#         # Robot Name mapping
#         row_robot = QHBoxLayout()
#         self.robot_name_input = QLineEdit()
#         self.robot_name_input.setPlaceholderText("Robot Name in Webots (Default: robot_a)")
#         row_robot.addWidget(QLabel("Robot Target:"))
#         row_robot.addWidget(self.robot_name_input)
#         layout.addLayout(row_robot)

#         # Dynamic Parameter Panels
#         layout.addWidget(QLabel("Experiment Parameters:"))
#         self.pid_panel = self.create_pid_panel()
#         self.traj_panel = self.create_traj_panel()
#         self.empty_param_panel = QLabel("No runtime parameters required for this experiment.")
#         self.empty_param_panel.setStyleSheet("color: #7F8C8D; font-style: italic;")
        
#         layout.addWidget(self.pid_panel)
#         layout.addWidget(self.traj_panel)
#         layout.addWidget(self.empty_param_panel)

#         self.scenario_combo.currentTextChanged.connect(self.sync_dropdowns)
#         self.world_combo.currentTextChanged.connect(self.update_summary)
        
#         return panel

#     def create_execution_panel(self):
#         panel = QGroupBox("2. EXPERIMENT EXECUTION")
#         layout = QHBoxLayout(panel)
        
#         self.launch_btn = QPushButton("▶ START EXPERIMENT")
#         self.launch_btn.setStyleSheet("background-color: #27AE60; color: white; font-weight: bold; height: 35px;")
        
#         self.kill_btn = QPushButton("■ STOP EXPERIMENT")
#         self.kill_btn.setStyleSheet("background-color: #C0392B; color: white; font-weight: bold; height: 35px;")
        
#         layout.addWidget(self.launch_btn)
#         layout.addWidget(self.kill_btn)
        
#         self.launch_btn.clicked.connect(self.handle_launch)
#         self.kill_btn.clicked.connect(self.handle_kill)
        
#         return panel

#     def create_evaluation_panel(self):
#         panel = QGroupBox("3. PERFORMANCE EVALUATION")
#         layout = QVBoxLayout(panel)
        
#         desc = QLabel("(Plots and engineering performance metrics are computed in the dashboard)")
#         desc.setStyleSheet("color: #7F8C8D;")
#         layout.addWidget(desc)
        
#         self.open_dashboard_btn = QPushButton("📊 OPEN ANALYSIS DASHBOARD")
#         self.open_dashboard_btn.setStyleSheet("background-color: #8E44AD; color: white; font-weight: bold; height: 40px;")
#         layout.addWidget(self.open_dashboard_btn)
        
#         self.open_dashboard_btn.clicked.connect(self.launch_analytics_window)
        
#         return panel

#     def create_pid_panel(self):
#         panel = QWidget()
#         layout = QHBoxLayout(panel)
#         layout.setContentsMargins(0,0,0,0)
#         self.kp_box, self.ki_box, self.kd_box = QDoubleSpinBox(), QDoubleSpinBox(), QDoubleSpinBox()
        
#         for box in (self.kp_box, self.ki_box, self.kd_box):
#             box.setDecimals(3)
#             box.setRange(0.0, 10.0)
#             box.setSingleStep(0.05)
#             box.setValue(1.0)
            
#         self.update_pid_btn = QPushButton("UPDATE GAINS")
#         self.update_pid_btn.setStyleSheet("background-color: #2980B9; color: white; font-weight: bold;")
#         self.update_pid_btn.clicked.connect(self.send_pid_update)
        
#         layout.addWidget(QLabel("Kp:")); layout.addWidget(self.kp_box)
#         layout.addWidget(QLabel("Ki:")); layout.addWidget(self.ki_box)
#         layout.addWidget(QLabel("Kd:")); layout.addWidget(self.kd_box)
#         layout.addWidget(self.update_pid_btn)
#         return panel

#     def create_traj_panel(self):
#         panel = QWidget()
#         layout = QVBoxLayout(panel)
#         layout.setContentsMargins(0,0,0,0)
        
#         row1 = QHBoxLayout()
#         self.open_map_btn = QPushButton("🗺️ Open Waypoint Selector")
#         self.open_map_btn.setStyleSheet("background-color: #E67E22; color: white; font-weight: bold;")
#         self.open_map_btn.clicked.connect(self.open_waypoint_map)
#         row1.addWidget(self.open_map_btn)
#         layout.addLayout(row1)

#         row2 = QHBoxLayout()
#         self.wp_x = QDoubleSpinBox()
#         self.wp_x.setRange(-5.0, 5.0) 
#         self.wp_x.setSingleStep(0.25)
        
#         self.wp_y = QDoubleSpinBox()
#         self.wp_y.setRange(-5.0, 5.0) 
#         self.wp_y.setSingleStep(0.25)

#         self.add_wp_btn = QPushButton("+ Add Pt")
#         self.clear_wp_btn = QPushButton("Clear")
#         self.add_wp_btn.clicked.connect(self.manual_add_wp)
#         self.clear_wp_btn.clicked.connect(self.clear_wps)

#         row2.addWidget(QLabel("X:")); row2.addWidget(self.wp_x)
#         row2.addWidget(QLabel("Y:")); row2.addWidget(self.wp_y)
#         row2.addWidget(self.add_wp_btn)
#         row2.addWidget(self.clear_wp_btn)
#         layout.addLayout(row2)

#         self.waypoints_display = QLineEdit()
#         self.waypoints_display.setReadOnly(True)
#         self.waypoints_display.setPlaceholderText("No absolute points set...")
#         layout.addWidget(self.waypoints_display)

#         return panel

#     # --- BEHAVIOR & LOGIC ---

#     def sync_dropdowns(self):
#         scenario = self.scenario_combo.currentText()
#         data = self.logic_matrix[scenario]
        
#         # Temporarily block signals to prevent redundant update_summary calls during clear/add
#         self.world_combo.blockSignals(True)
#         self.world_combo.clear()
#         self.world_combo.addItems(data["worlds"])
#         self.world_combo.blockSignals(False)
        
#         self.desc_box.setText(data["desc"])

#         is_traj = (scenario == "Trajectory Planning")
#         is_pid = (scenario == "Wall Following")
        
#         self.traj_panel.setVisible(is_traj)
#         self.pid_panel.setVisible(is_pid)
#         self.empty_param_panel.setVisible(not is_traj and not is_pid)

#         if data["has_analytics"]:
#             self.open_dashboard_btn.setEnabled(True)
#             self.open_dashboard_btn.setStyleSheet("background-color: #8E44AD; color: white; font-weight: bold; height: 40px;")
#             self.open_dashboard_btn.setText("📊 OPEN ANALYSIS DASHBOARD")
#         else:
#             self.open_dashboard_btn.setEnabled(False)
#             self.open_dashboard_btn.setStyleSheet("background-color: #95A5A6; color: #ECF0F1; font-weight: bold; height: 40px;")
#             self.open_dashboard_btn.setText("Analytics Disabled for this Experiment")
            
#         self.update_summary()

#     def update_summary(self, status_override=None):
#         scenario = self.scenario_combo.currentText()
#         data = self.logic_matrix[scenario]
        
#         self.sum_exp_lbl.setText(f"Experiment : {scenario}")
#         self.sum_world_lbl.setText(f"World      : {self.world_combo.currentText()}")
#         self.sum_ctrl_lbl.setText(f"Controller : {data['controller']}")
        
#         if status_override:
#             if "Running" in status_override:
#                 self.sum_status_lbl.setStyleSheet("color: #E67E22;") # Orange
#             elif "Stopped" in status_override or "Completed" in status_override:
#                 self.sum_status_lbl.setStyleSheet("color: #34495E;") # Dark Blue
#             self.sum_status_lbl.setText(f"Status     : {status_override}")

#     def manual_add_wp(self):
#         x = round(self.wp_x.value(), 2)
#         y = round(self.wp_y.value(), 2)
#         self.current_waypoints.append([x, y])
#         self.waypoints_display.setText(f"Absolute: {str(self.current_waypoints)}")

#     def clear_wps(self):
#         self.current_waypoints.clear()
#         self.waypoints_display.setText("")

#     def launch_analytics_window(self):
#         self.analytics_window = AnalyticsWindow(self.scenario_combo.currentText())
#         self.analytics_window.show()

#     def handle_launch(self):
#         if self.ros_manager.is_running(): 
#             QMessageBox.warning(self, 'Error', 'Experiment is already running.')
#             return
#         scenario = self.scenario_combo.currentText()
#         node = self.custom_node if self.custom_node else self.logic_matrix[scenario]["node"]
        
#         waypoints_str = ""
#         if scenario == "Trajectory Planning" and self.current_waypoints:
#             try:
#                 flat_list = []
#                 for pt in self.current_waypoints:
#                     rel_x = round(pt[0] - ROBOT_START_X, 3)
#                     rel_y = round(pt[1] - ROBOT_START_Y, 3)
#                     flat_list.extend([rel_x, rel_y])
                
#                 waypoints_str = str(flat_list).replace(" ", "")
#                 print(f"[GUI MATH] Translated Absolute Webots points to Relative Node Points: {waypoints_str}")
#             except Exception as e:
#                 print(f"Error parsing waypoints: {e}")

#         self.update_summary("● Running")
#         self.ros_manager.launch(self.world_combo.currentText(), node, self.robot_name_input.text(), waypoints_str)

#     def handle_kill(self):
#         if not self.ros_manager.kill_all():
#             QMessageBox.warning(self, 'Error', 'No experiment currently running.')
#         else:
#             self.update_summary("● Stopped")

#     def send_pid_update(self):
#         if not self.ros_manager.is_running():
#             QMessageBox.warning(self, "Error", "Start the experiment first!")
#             return
#         node = self.logic_matrix[self.scenario_combo.currentText()]["node"]
#         self.ros_manager.set_pid_params(node, self.robot_name_input.text(), self.kp_box.value(), self.ki_box.value(), self.kd_box.value())

#     def open_waypoint_map(self):
#         map_dialog = InteractiveMapDialog(self.current_waypoints, self)
#         if map_dialog.exec_() == QDialog.Accepted:
#             self.current_waypoints = map_dialog.get_points()
#             if self.current_waypoints:
#                 self.waypoints_display.setText(f"Absolute: {str(self.current_waypoints)}")
#             else:
#                 self.waypoints_display.setText("")


# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     app.setStyle("Fusion") # Cleaner cross-platform look
#     window = AFIT_GCS()
#     window.show()
#     sys.exit(app.exec_())




















































#updated v2
# #!/usr/bin/env python3
# import sys
# import os
# import subprocess
# import signal
# import json
# import pandas as pd
# import numpy as np

# # --- GUI IMPORTS ---
# from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
#                              QComboBox, QPushButton, QTextEdit, QLabel, QMessageBox, 
#                              QFileDialog, QDoubleSpinBox, QGroupBox, QLineEdit, 
#                              QTableWidgetItem, QTableWidget, QHeaderView, QScrollArea,
#                              QDialog, QDialogButtonBox, QFrame, QTextBrowser)
# from PyQt5.QtGui import QFont, QPalette, QColor, QPixmap
# from PyQt5.QtCore import Qt

# # --- MATPLOTLIB IMPORTS ---
# from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
# from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
# from matplotlib.figure import Figure
# from matplotlib.patches import Rectangle


# # =====================================================================
# # --- CONFIGURATION & CONSTANTS ---
# # =====================================================================
# ROS2_SETUP_CMD = "source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash"
# DEFAULT_LOG_DIR = os.path.expanduser('~/ros2_ws/simulation_logs')

# # --- ROBOT STARTING POSITION (WEBOTS ABSOLUTE) ---
# ROBOT_START_X = -2.5
# ROBOT_START_Y = -2.5


# # =====================================================================
# # --- CORE LOGIC MODULES ---
# # =====================================================================

# class ROS2ProcessManager:
#     """Handles OS-level subprocess execution for ROS 2 commands."""
    
#     def __init__(self):
#         self.active_process = None

#     def is_running(self):
#         return self.active_process is not None

#     def launch(self, target_world, node, robot_name, waypoints_str=""):
#         robot_arg = f"robot_name:={robot_name}" if robot_name.strip() else ""
#         waypoint_arg = f" relative_waypoints:=\"{waypoints_str}\"" if waypoints_str else ""
#         launch_args = f"world:={target_world} scenario:={node} {robot_arg}{waypoint_arg}".strip()
#         full_cmd = f"{ROS2_SETUP_CMD} && ros2 launch robot_simulation simulation.launch.py {launch_args}"
        
#         print(f"IGNITION SEQUENCE INITIATED:\n{full_cmd}")
#         self.active_process = subprocess.Popen(
#             full_cmd, shell=True, executable='/bin/bash', preexec_fn=os.setsid
#         )

#     def kill_all(self):
#         if not self.is_running():
#             return False
            
#         print("\nTERMINATE: Initiating complete system sweep...")
#         try:
#             pgid = os.getpgid(self.active_process.pid)
#             os.killpg(pgid, signal.SIGTERM)
#             print(f"SUCCESS: Terminated core simulation group {pgid}")
#         except ProcessLookupError:
#             print("WARNING: Core simulation already down.")
#         finally:
#             self.active_process = None

#         print("SWEEP: Purging background terminal processes...")
#         try:
#             subprocess.run("pkill -f 'ros2 topic pub'", shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
#             global ROS2_SETUP_CMD
#             subprocess.run(f"{ROS2_SETUP_CMD} && ros2 daemon stop", shell=True, executable='/bin/bash', stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
#             print("SUCCESS: ROS 2 background discovery channels purged clean.\n")
#         except Exception as e:
#             print(f"WARNING: Automated sweep encountered an issue: {e}")

#         return True

#     def set_pid_params(self, node, robot_name, kp, ki, kd):
#         global ROS2_SETUP_CMD
#         target_node = f"{robot_name}/{node}" if robot_name.strip() else node
#         target_node = target_node.strip('/')
        
#         cmd = (f"({ROS2_SETUP_CMD} && ros2 param set /{target_node} Kp {kp}) & "
#                f"({ROS2_SETUP_CMD} && ros2 param set /{target_node} Ki {ki}) & "
#                f"({ROS2_SETUP_CMD} && ros2 param set /{target_node} Kd {kd})")
               
#         print(f"\n--- DISPATCHING ASYNC PID GAINS FOR /{target_node} ---")
#         subprocess.Popen(cmd, shell=True, executable='/bin/bash', stdout=subprocess.DEVNULL)

#     def publish_polynomial(self, poly_type):
#         global ROS2_SETUP_CMD
#         raw_cmd = f"{ROS2_SETUP_CMD} && ros2 topic pub --once /set_polynomial std_msgs/msg/String \"{{data: '{poly_type}'}}\""
#         subprocess.Popen(raw_cmd, shell=True, executable='/bin/bash', stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

#     def publish_waypoints(self, points):
#         if not self.is_running(): return False
            
#         global ROS2_SETUP_CMD
        
#         # Convert Absolute (Webots) back to Relative before publishing via topic
#         relative_points = [[pt[0] - ROBOT_START_X, pt[1] - ROBOT_START_Y] for pt in points]
#         rotated_points = [[round(pt[1], 3), round(-pt[0], 3)] for pt in relative_points]
        
#         points_str = json.dumps(rotated_points)
#         raw_cmd = f"{ROS2_SETUP_CMD} && ros2 topic pub --keep-alive 2 /gcs/via_points std_msgs/msg/String \"{{data: '{points_str}'}}\""
#         subprocess.Popen(raw_cmd, shell=True, executable='/bin/bash', stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
#         return True

# class SimulationAnalyzer:
#     @staticmethod
#     def calculate_wall_following_metrics(df):
#         time, errors = df['Time'].values, df['Error'].values
#         dt = np.mean(np.diff(time)) if len(time) > 1 else 0.1
        
#         rmse = np.sqrt(np.mean(errors**2))
#         iae = np.trapz(np.abs(errors), dx=dt)
#         ise = np.trapz(errors**2, dx=dt)
#         peak_err = np.max(np.abs(errors))
#         initial_err = abs(errors[0]) if len(errors) > 0 else 0.4
#         os_pct = ((peak_err - initial_err) / initial_err) * 100 if peak_err > initial_err else 0.0
#         return [iae, ise, os_pct, 0.0, rmse]


# # =====================================================================
# # --- GUI MODULES ---
# # =====================================================================

# class InteractiveMapDialog(QDialog):
#     def __init__(self, existing_points, parent=None):
#         super().__init__(parent)
#         self.setWindowTitle("Map Waypoint Selector")
#         self.resize(650, 650)
#         self.via_points = existing_points.copy()
        
#         layout = QVBoxLayout(self)
#         self.figure = Figure(figsize=(6, 6), dpi=100)
#         self.canvas = FigureCanvas(self.figure)
#         layout.addWidget(self.canvas)
        
#         self.coord_label = QLabel(f"Absolute Points: {self.via_points}" if self.via_points else "Click on the grid to add Webots absolute coordinates.")
#         self.coord_label.setAlignment(Qt.AlignCenter)
#         self.coord_label.setStyleSheet("font-weight: bold; font-size: 14px; padding: 10px;")
#         layout.addWidget(self.coord_label)
        
#         self.btn_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
#         self.clear_btn = QPushButton("Clear Points")
#         self.btn_box.addButton(self.clear_btn, QDialogButtonBox.ActionRole)
#         layout.addWidget(self.btn_box)
        
#         self.btn_box.accepted.connect(self.accept)
#         self.btn_box.rejected.connect(self.reject)
#         self.clear_btn.clicked.connect(self.clear_plot)
#         self.canvas.mpl_connect('button_press_event', self.on_click)
#         self.setup_plot()

#     def setup_plot(self):
#         self.figure.clear()
#         self.ax = self.figure.add_subplot(111)
#         self.ax.set_title("Interactive Arena Map (Webots 10x10 Arena)")
        
#         # Set bounds to encompass the new 10x10 area with a small buffer for visibility
#         self.ax.set_xlim(-5.5, 5.5) 
#         self.ax.set_ylim(-5.5, 5.5)
#         self.ax.set_xlabel("Webots X (meters)")
#         self.ax.set_ylabel("Webots Y (meters)")
#         self.ax.grid(True, linestyle='--', alpha=0.5)
#         self.ax.set_aspect('equal')
        
#         # 1. Draw the exact center of the Webots world (0,0)
#         self.ax.plot(0, 0, 'k+', markersize=15, markeredgewidth=2, label="Arena Center (0,0)")
        
#         # 2. Draw the solid physical walls of the 10x10 arena (from -5.0 to 5.0)
#         walls = Rectangle((-5.0, -5.0), 10.0, 10.0, fill=False, edgecolor='black', linewidth=3)
#         self.ax.add_patch(walls)

#         # Draw the robot physically where it starts in Webots
#         self.ax.plot(ROBOT_START_X, ROBOT_START_Y, 'gX', markersize=12, label=f"Robot Start ({ROBOT_START_X}, {ROBOT_START_Y})")
#         self.ax.legend(loc="upper right", fontsize='small')
        
#         # Draw existing points
#         for pt in self.via_points:
#             self.ax.plot(pt[0], pt[1], 'ro', markersize=8)
#         if len(self.via_points) > 1:
#             pts = np.array(self.via_points)
#             self.ax.plot(pts[:,0], pts[:,1], 'b-', alpha=0.5)
            
#         self.canvas.draw()

#     def on_click(self, event):
#         if event.inaxes != self.ax: return
#         x, y = round(event.xdata, 2), round(event.ydata, 2)
        
#         # Optional: Prevent clicks completely outside the physical 10x10 wall bounds
#         if x < -5.0 or x > 5.0 or y < -5.0 or y > 5.0:
#             return
            
#         self.via_points.append([x, y])
#         self.coord_label.setText(f"Absolute Points: {self.via_points}")
#         self.ax.plot(x, y, 'ro', markersize=8)
#         if len(self.via_points) > 1:
#             prev_point = self.via_points[-2]
#             self.ax.plot([prev_point[0], x], [prev_point[1], y], 'b-', alpha=0.5)
#         self.canvas.draw()

#     def clear_plot(self):
#         self.via_points.clear()
#         self.coord_label.setText("Click on the grid to add Webots absolute coordinates.")
#         self.setup_plot()
        
#     def get_points(self): return self.via_points


# class AnalyticsWindow(QWidget):
#     def __init__(self, scenario_name):
#         super().__init__()
#         self.scenario_name = scenario_name
#         self.setWindowTitle(f"Performance Evaluation Dashboard - {self.scenario_name}")
#         self.resize(1050, 750) 
#         self.init_ui()

#     def init_ui(self):
#         self.outer_layout = QVBoxLayout(self)
#         self.outer_layout.setContentsMargins(0, 0, 0, 0)
        
#         self.scroll = QScrollArea()
#         self.scroll.setWidgetResizable(True)
#         self.scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        
#         self.content_container = QWidget()
#         self.page_layout = QVBoxLayout(self.content_container)
#         self.page_layout.setSpacing(20)
        
#         self.extract_btn = QPushButton(f"📂 Load {self.scenario_name} Experiment Data")
#         self.extract_btn.setStyleSheet("background-color: #2980b9; color: white; font-weight: bold; padding: 15px; font-size: 14px;")
#         self.extract_btn.clicked.connect(self.load_and_plot_file)
#         self.page_layout.addWidget(self.extract_btn)

#         self.stats_label = QLabel("Experiment Log: Pending Selection")
#         self.stats_label.setFont(QFont("Courier", 11))
#         self.stats_label.setAlignment(Qt.AlignCenter)
#         self.stats_label.setStyleSheet("background-color: #ecf0f1; padding: 10px; border: 1px solid #bdc3c7;")
#         self.page_layout.addWidget(self.stats_label)

#         self.page_layout.addWidget(QLabel("📊 ENGINEERING PERFORMANCE INDICES:"))
#         self.metrics_table = QTableWidget(1, 5) 
#         self.metrics_table.setHorizontalHeaderLabels(["IAE (Area)", "ISE (Penalty)", "OS (%)", "Settling (s)", "RMSE (m)"])
#         self.metrics_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
#         self.metrics_table.setFixedHeight(85)
#         self.metrics_table.setStyleSheet("background-color: #f8f9fa; color: black; font-weight: bold;")
#         self.page_layout.addWidget(self.metrics_table)

#         self.page_layout.addWidget(QLabel("📈 EXPERIMENT RESPONSE PLOTS:"))
#         self.figure = Figure(figsize=(10, 10), dpi=100)
#         self.canvas = FigureCanvas(self.figure)
#         self.toolbar = NavigationToolbar(self.canvas, self)
#         self.canvas.setMinimumHeight(800) 
        
#         self.page_layout.addWidget(self.toolbar)
#         self.page_layout.addWidget(self.canvas)
#         self.scroll.setWidget(self.content_container)
#         self.outer_layout.addWidget(self.scroll)

#     def load_and_plot_file(self):
#         csv_filename, _ = QFileDialog.getOpenFileName(self, f"Open {self.scenario_name} Log", DEFAULT_LOG_DIR, "CSV Files (*.csv)")
#         if not csv_filename: return

#         try:
#             df = pd.read_csv(csv_filename)
#             df.columns = df.columns.str.strip() # Fixes hidden spaces in CSV headers
#         except Exception as e:
#             QMessageBox.critical(self, "Read Error", f"Failed to parse CSV:\n{e}")
#             return
            
#         if self.scenario_name == "Wall Following":
#             self.plot_wall_following(df, csv_filename)
#         elif self.scenario_name == "Obstacle Avoidance":
#             self.plot_obstacle_avoidance(df, csv_filename)
#         elif self.scenario_name == "Trajectory Planning":
#             self.plot_trajectory_planning(df, csv_filename)

#         self.figure.tight_layout(pad=3.0)
#         self.canvas.draw()

#     def update_table(self, values):
#         for i, val in enumerate(values):
#             self.metrics_table.setItem(0, i, QTableWidgetItem(f"{val}"))

#     def plot_wall_following(self, df, path):
#         self.figure.clear()
#         metrics = SimulationAnalyzer.calculate_wall_following_metrics(df)
#         self.update_table([f"{m:.4f}" for m in metrics])
#         self.stats_label.setText(f"Wall Following Analysis | File: {os.path.basename(path)}")

#         time = df['Time'].values
#         errors = df['Error'].values
#         effort = df['Control_Effort'].values
        
#         ax_spatial = self.figure.add_subplot(221)
#         ax_err     = self.figure.add_subplot(222)
#         ax_effort  = self.figure.add_subplot(212)

#         if 'X' in df.columns and 'Y' in df.columns:
#             ax_spatial.plot(df['X'], df['Y'], color='black', linewidth=1.5, label='Robot Path')
#             ax_spatial.set(title="2D Spatial Cornering", xlabel="X (m)", ylabel="Y (m)")
#             ax_spatial.axis('equal')
#         else:
#             ax_spatial.text(0.5, 0.5, 'Missing X, Y in CSV', ha='center', va='center', color='red')
#             ax_spatial.set(title="2D Spatial Cornering")

#         ax_err.plot(time, errors, color='#1f77b4', linewidth=2)
#         ax_err.axhline(0, color='red', linestyle='--', alpha=0.5)
#         ax_err.set(title="Tracking Accuracy (e(t))", xlabel="Time (s)", ylabel="Error (m)")
#         ax_err.grid(True, linestyle=':', alpha=0.6)

#         ax_effort.step(time, effort, color='#2ca02c', linewidth=1.5)
#         ax_effort.set(title="Steering Actuation (Control Effort)", xlabel="Time (s)", ylabel="Steer (rad/s)")
#         ax_effort.grid(True, linestyle=':', alpha=0.6)

#     def plot_obstacle_avoidance(self, df, path):
#         self.figure.clear()
#         time, clearance, ang_vel = df['Time'].values, df['Clearance'].values, df['Angular_Vel'].values
#         min_clear = np.min(clearance)
#         self.stats_label.setText(f"🚧 OBSTACLE AVOIDANCE 🚧 | File: {os.path.basename(path)}")
#         self.update_table(["N/A", "N/A", "N/A", f"{min_clear:.4f}", f"{time[-1]:.1f}"])

#         ax1 = self.figure.add_subplot(211)
#         ax2 = self.figure.add_subplot(212)

#         ax1.plot(time, clearance, color='#e67e22', linewidth=2, label='Clearance')
#         ax1.axhline(0.2, color='red', linestyle='--', label='Safety Limit')
#         ax1.set(title="Proximity Metrics", ylabel="Dist (m)")
#         ax1.grid(True, linestyle=':', alpha=0.7)
        
#         ax2.plot(time, ang_vel, color='#9b59b6', label='Yaw Rate')
#         ax2.set(title="Maneuver Intensity", xlabel="Time (s)", ylabel="Rad/s")
#         ax2.grid(True, linestyle=':', alpha=0.7)

#     def plot_trajectory_planning(self, df, path):
#         self.figure.clear()
#         self.stats_label.setText(f"📐 Quintic Spline & Dynamic Pure Pursuit | File: {os.path.basename(path)}")

#         if df.empty:
#             QMessageBox.critical(self, "Empty File", "The CSV has headers but no data.")
#             return

#         required = ['Time', 'X', 'Y', 'Target_X', 'Target_Y', 'V', 'W', 'Ld', 'CTE']
#         if not all(col in df.columns for col in required):
#             QMessageBox.critical(self, "Data Error", f"CSV missing columns!\nRequires: {required}")
#             return

#         time = df['Time'].values
#         cte = df['CTE'].values

#         rmse_cte = np.sqrt(np.mean(cte**2))
#         max_cte = np.max(np.abs(cte))
#         self.update_table(["N/A", "N/A", "N/A", f"Max: {max_cte:.4f}m", f"RMSE: {rmse_cte:.4f}m"])

#         ax1 = self.figure.add_subplot(221)
#         ax2 = self.figure.add_subplot(222)
#         ax3 = self.figure.add_subplot(223)
#         ax4 = self.figure.add_subplot(224)

#         ax1.plot(df['Target_X'], df['Target_Y'], 'b--', alpha=0.6, label='Ideal')
#         ax1.plot(df['X'], df['Y'], 'g-', linewidth=2, label='Actual')
#         ax1.scatter(df['X'].iloc[0], df['Y'].iloc[0], color='green', s=80, label='Start', zorder=5)
#         ax1.scatter(df['X'].iloc[-1], df['Y'].iloc[-1], color='red', marker='X', s=80, label='Goal', zorder=5)
#         ax1.set_title("2D Spatial Path Tracking")
#         ax1.set_xlabel("X (m)"); ax1.set_ylabel("Y (m)")
#         ax1.axis('equal'); ax1.grid(True, linestyle=':', alpha=0.7); ax1.legend(loc='best', fontsize='small')

#         ax2.plot(time, cte, color='#e74c3c', linewidth=2, label='CTE')
#         ax2.axhline(0, color='black', linestyle='--', alpha=0.6)
#         ax2.fill_between(time, cte, 0, color='#e74c3c', alpha=0.1)
#         ax2.set_title("Kinematic Tracking Error (CTE)")
#         ax2.set_xlabel("Time (s)"); ax2.set_ylabel("Error (m)")
#         ax2.grid(True, linestyle=':', alpha=0.7); ax2.legend(loc='best')

#         ax3.plot(time, df['Ld'], color='purple', linewidth=2, label='Ld')
#         ax3.set_xlabel("Time (s)"); ax3.set_ylabel("Look-Ahead (m)", color='purple')
#         ax3.tick_params(axis='y', labelcolor='purple')
        
#         ax3_twin = ax3.twinx()
#         ax3_twin.plot(time, df['V'], color='orange', linestyle='--', linewidth=2, label='Velocity')
#         ax3_twin.set_ylabel("Velocity (m/s)", color='orange')
#         ax3_twin.tick_params(axis='y', labelcolor='orange')
#         ax3.set_title("Dynamic Look-Ahead Validation")
#         ax3.grid(True, linestyle=':', alpha=0.5)

#         ax4.plot(time, df['W'], color='#2ca02c', linewidth=1.5, label='Angular Vel (W)')
#         ax4.set_title("Steering Actuation Smoothness")
#         ax4.set_xlabel("Time (s)"); ax4.set_ylabel("Commanded rad/s")
#         ax4.grid(True, linestyle=':', alpha=0.7); ax4.legend(loc='best')


# class AFIT_GCS(QWidget):
#     def __init__(self):
#         super().__init__()
#         self.ros_manager = ROS2ProcessManager()
#         self.custom_node = ""
#         self.current_waypoints = []
        
#         self.logic_matrix = {
#             "Teleop": {
#                 "worlds": ["obstacle_course.wbt", "garden.wbt", "wall_following.wbt"], 
#                 "node": "teleop_node", 
#                 "controller": "Manual Control",
#                 "has_analytics": False,
#                 "objective": "Understand how manual keyboard commands are translated into robot motion through the ROS2 communication framework and the differential-drive controller.",
#                 "how_it_works": "• Press W, A, S or D\n• The GUI generates velocity commands.\n• ROS2 publishes the commands to the robot.\n• The differential-drive controller converts them into left and right wheel speeds.\n• The robot moves in Webots.",
#                 "diagram_path": ""
#             },
#             "Trajectory Planning": {
#                 "worlds": ["obstacle_course.wbt", "garden.wbt", "wall_following.wbt"], 
#                 "node": "niku_controller_viapoints", 
#                 "controller": "Pure Pursuit Spline",
#                 "has_analytics": True,
#                 "objective": "Understand how a robot generates a smooth trajectory through user-defined waypoints and follows it using the Pure Pursuit controller.",
#                 "how_it_works": "• Select waypoints.\n• A Piecewise Quintic Polynomial trajectory is generated.\n• Pure Pursuit selects a look-ahead point.\n• The controller computes the steering command.\n• The robot follows the planned trajectory.",
#                 "diagram_path": ""
#             },
#             "Wall Following": {
#                 "worlds": ["wall_following_1.wbt", "wall_following_2.wbt", "garden.wbt"], 
#                 "node": "wall_follower", 
#                 "controller": "PID Control",
#                 "has_analytics": True,
#                 "objective": "Investigate how feedback control enables a robot to maintain a desired distance from a wall by adjusting the PID controller gains.",
#                 "how_it_works": "• The laser scanner measures the wall distance.\n• The measured distance is compared with the desired distance.\n• The PID controller computes a steering correction.\n• The steering command is sent to the robot.\n• The process repeats continuously.",
#                 "diagram_path": ""
#             },
#             "Obstacle Avoidance": {
#                 "worlds": ["boxes_dense.wbt", "obstacle_course.wbt", "garden.wbt", "wall_following.wbt"], 
#                 "node": "obstacle_avoidance", 
#                 "controller": "Reactive Logic",
#                 "has_analytics": True,
#                 "objective": "Evaluate real-time LiDAR-based obstacle detection and reactive maneuver intensity to prevent collisions.",
#                 "how_it_works": "• LiDAR scans the immediate environment.\n• Distance to nearest obstacle is calculated.\n• If distance < safety threshold, compute avoidance vector.\n• Publish angular velocity override command.\n• Robot steers away from collision.",
#                 "diagram_path": ""
#             }
#         }
#         self.init_ui()

#     def init_ui(self):
#         self.setWindowTitle("Integrated Mobile Robot Experimentation Platform")
#         self.resize(750, 950) # Increased height to accommodate the Learning Panel
        
#         self.setStyleSheet("""
#             QWidget { background-color: #ECF0F1; } 
#             QLabel { font-weight: bold; color: #2C3E50; }
#             QGroupBox { font-weight: bold; border: 2px solid #BDC3C7; border-radius: 5px; margin-top: 1ex; }
#             QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; padding: 0 3px; }
#             QComboBox, QLineEdit, QDoubleSpinBox { background-color: white; color: black; border: 1px solid #BDC3C7; border-radius: 3px; padding: 2px;}
#             QTextBrowser { background-color: #F8F9FA; font-style: italic; border: 1px solid #BDC3C7; }
#         """)

#         # Wrapping everything in a scroll area just in case screens are small
#         self.scroll_main = QScrollArea()
#         self.scroll_main.setWidgetResizable(True)
#         self.main_container = QWidget()
#         main_layout = QVBoxLayout(self.main_container)

#         # Main Title Header
#         title_lbl = QLabel("INTEGRATED MOBILE ROBOT EXPERIMENTATION PLATFORM")
#         title_lbl.setAlignment(Qt.AlignCenter)
#         title_lbl.setStyleSheet("font-size: 16px; background-color: #34495E; color: white; padding: 10px; border-radius: 5px;")
#         main_layout.addWidget(title_lbl)

#         # Build Platform Sections
#         main_layout.addWidget(self.create_summary_panel())
#         main_layout.addWidget(self.create_configuration_panel())
#         main_layout.addWidget(self.create_execution_panel())
#         main_layout.addWidget(self.create_evaluation_panel())
        
#         self.scroll_main.setWidget(self.main_container)
        
#         # Set central layout
#         central_layout = QVBoxLayout(self)
#         central_layout.setContentsMargins(0, 0, 0, 0)
#         central_layout.addWidget(self.scroll_main)

#         self.sync_dropdowns()

#     # --- UI COMPONENT BUILDERS ---

#     def create_summary_panel(self):
#         panel = QGroupBox("CURRENT EXPERIMENT SUMMARY")
#         layout = QVBoxLayout(panel)
        
#         self.sum_exp_lbl = QLabel("Experiment : ")
#         self.sum_world_lbl = QLabel("World      : ")
#         self.sum_ctrl_lbl = QLabel("Controller : ")
#         self.sum_status_lbl = QLabel("Status     : ● Ready")
#         self.sum_status_lbl.setStyleSheet("color: #27AE60;") # Green for ready
        
#         for lbl in [self.sum_exp_lbl, self.sum_world_lbl, self.sum_ctrl_lbl, self.sum_status_lbl]:
#             lbl.setFont(QFont("Courier", 10))
#             layout.addWidget(lbl)
            
#         return panel

#     def create_configuration_panel(self):
#         panel = QGroupBox("1. EXPERIMENT CONFIGURATION")
#         layout = QVBoxLayout(panel)
        
#         # Selectors
#         row_selectors = QHBoxLayout()
#         self.scenario_combo = QComboBox()
#         self.scenario_combo.addItems(self.logic_matrix.keys())
#         self.world_combo = QComboBox()
        
#         row_selectors.addWidget(QLabel("Experiment Type:"))
#         row_selectors.addWidget(self.scenario_combo, stretch=2)
#         row_selectors.addWidget(QLabel("Webots World:"))
#         row_selectors.addWidget(self.world_combo, stretch=2)
#         layout.addLayout(row_selectors)

#         # Learning Panel replacing the static Experiment Objective
#         self.learning_panel = QGroupBox("Learning Panel")
#         lp_layout = QVBoxLayout(self.learning_panel)
        
#         lp_layout.addWidget(QLabel("Experiment Objective"))
#         self.obj_browser = QTextBrowser()
#         self.obj_browser.setMaximumHeight(50)
#         lp_layout.addWidget(self.obj_browser)
        
#         lp_layout.addWidget(QLabel("How It Works"))
#         self.hw_browser = QTextBrowser()
#         self.hw_browser.setMaximumHeight(130)
#         lp_layout.addWidget(self.hw_browser)
        
#         # Diagram Upload & Display Section
#         diagram_header_layout = QHBoxLayout()
#         diagram_header_layout.addWidget(QLabel("System Diagram"))
#         self.upload_img_btn = QPushButton("Upload System Diagram")
#         self.upload_img_btn.setStyleSheet("background-color: #3498db; color: white; padding: 5px;")
#         self.upload_img_btn.clicked.connect(self.upload_diagram)
#         diagram_header_layout.addWidget(self.upload_img_btn)
#         lp_layout.addLayout(diagram_header_layout)

#         self.diagram_label = QLabel("No Diagram Uploaded")
#         self.diagram_label.setAlignment(Qt.AlignCenter)
#         self.diagram_label.setMinimumHeight(200)
#         self.diagram_label.setStyleSheet("border: 1px dashed #BDC3C7; background-color: #FFFFFF;")
#         lp_layout.addWidget(self.diagram_label)
        
#         layout.addWidget(self.learning_panel)

#         # Robot Name mapping
#         row_robot = QHBoxLayout()
#         self.robot_name_input = QLineEdit()
#         self.robot_name_input.setPlaceholderText("Robot Name in Webots (Default: robot_a)")
#         row_robot.addWidget(QLabel("Robot Target:"))
#         row_robot.addWidget(self.robot_name_input)
#         layout.addLayout(row_robot)

#         # Dynamic Parameter Panels
#         layout.addWidget(QLabel("Experiment Parameters:"))
#         self.pid_panel = self.create_pid_panel()
#         self.traj_panel = self.create_traj_panel()
#         self.empty_param_panel = QLabel("No runtime parameters required for this experiment.")
#         self.empty_param_panel.setStyleSheet("color: #7F8C8D; font-style: italic;")
        
#         layout.addWidget(self.pid_panel)
#         layout.addWidget(self.traj_panel)
#         layout.addWidget(self.empty_param_panel)

#         self.scenario_combo.currentTextChanged.connect(self.sync_dropdowns)
#         self.world_combo.currentTextChanged.connect(self.update_summary)
        
#         return panel

#     def create_execution_panel(self):
#         panel = QGroupBox("2. EXPERIMENT EXECUTION")
#         layout = QHBoxLayout(panel)
        
#         self.launch_btn = QPushButton("▶ START EXPERIMENT")
#         self.launch_btn.setStyleSheet("background-color: #27AE60; color: white; font-weight: bold; height: 35px;")
        
#         self.kill_btn = QPushButton("■ STOP EXPERIMENT")
#         self.kill_btn.setStyleSheet("background-color: #C0392B; color: white; font-weight: bold; height: 35px;")
        
#         layout.addWidget(self.launch_btn)
#         layout.addWidget(self.kill_btn)
        
#         self.launch_btn.clicked.connect(self.handle_launch)
#         self.kill_btn.clicked.connect(self.handle_kill)
        
#         return panel

#     def create_evaluation_panel(self):
#         panel = QGroupBox("3. PERFORMANCE EVALUATION")
#         layout = QVBoxLayout(panel)
        
#         desc = QLabel("(Plots and engineering performance metrics are computed in the dashboard)")
#         desc.setStyleSheet("color: #7F8C8D;")
#         layout.addWidget(desc)
        
#         self.open_dashboard_btn = QPushButton("📊 OPEN ANALYSIS DASHBOARD")
#         self.open_dashboard_btn.setStyleSheet("background-color: #8E44AD; color: white; font-weight: bold; height: 40px;")
#         layout.addWidget(self.open_dashboard_btn)
        
#         self.open_dashboard_btn.clicked.connect(self.launch_analytics_window)
        
#         return panel

#     def create_pid_panel(self):
#         panel = QWidget()
#         layout = QHBoxLayout(panel)
#         layout.setContentsMargins(0,0,0,0)
#         self.kp_box, self.ki_box, self.kd_box = QDoubleSpinBox(), QDoubleSpinBox(), QDoubleSpinBox()
        
#         for box in (self.kp_box, self.ki_box, self.kd_box):
#             box.setDecimals(3)
#             box.setRange(0.0, 10.0)
#             box.setSingleStep(0.05)
#             box.setValue(1.0)
            
#         self.update_pid_btn = QPushButton("UPDATE GAINS")
#         self.update_pid_btn.setStyleSheet("background-color: #2980B9; color: white; font-weight: bold;")
#         self.update_pid_btn.clicked.connect(self.send_pid_update)
        
#         layout.addWidget(QLabel("Kp:")); layout.addWidget(self.kp_box)
#         layout.addWidget(QLabel("Ki:")); layout.addWidget(self.ki_box)
#         layout.addWidget(QLabel("Kd:")); layout.addWidget(self.kd_box)
#         layout.addWidget(self.update_pid_btn)
#         return panel

#     def create_traj_panel(self):
#         panel = QWidget()
#         layout = QVBoxLayout(panel)
#         layout.setContentsMargins(0,0,0,0)
        
#         row1 = QHBoxLayout()
#         self.open_map_btn = QPushButton("🗺️ Open Waypoint Selector")
#         self.open_map_btn.setStyleSheet("background-color: #E67E22; color: white; font-weight: bold;")
#         self.open_map_btn.clicked.connect(self.open_waypoint_map)
#         row1.addWidget(self.open_map_btn)
#         layout.addLayout(row1)

#         row2 = QHBoxLayout()
#         self.wp_x = QDoubleSpinBox()
#         self.wp_x.setRange(-5.0, 5.0) 
#         self.wp_x.setSingleStep(0.25)
        
#         self.wp_y = QDoubleSpinBox()
#         self.wp_y.setRange(-5.0, 5.0) 
#         self.wp_y.setSingleStep(0.25)

#         self.add_wp_btn = QPushButton("+ Add Pt")
#         self.clear_wp_btn = QPushButton("Clear")
#         self.add_wp_btn.clicked.connect(self.manual_add_wp)
#         self.clear_wp_btn.clicked.connect(self.clear_wps)

#         row2.addWidget(QLabel("X:")); row2.addWidget(self.wp_x)
#         row2.addWidget(QLabel("Y:")); row2.addWidget(self.wp_y)
#         row2.addWidget(self.add_wp_btn)
#         row2.addWidget(self.clear_wp_btn)
#         layout.addLayout(row2)

#         self.waypoints_display = QLineEdit()
#         self.waypoints_display.setReadOnly(True)
#         self.waypoints_display.setPlaceholderText("No absolute points set...")
#         layout.addWidget(self.waypoints_display)

#         return panel

#     # --- BEHAVIOR & LOGIC ---

#     def upload_diagram(self):
#         file_path, _ = QFileDialog.getOpenFileName(self, "Select System Diagram", "", "Image Files (*.png *.jpg *.jpeg *.svg *.bmp)")
#         if file_path:
#             scenario = self.scenario_combo.currentText()
#             self.logic_matrix[scenario]["diagram_path"] = file_path
#             self.load_diagram_image(file_path)

#     def load_diagram_image(self, file_path):
#         if file_path and os.path.exists(file_path):
#             pixmap = QPixmap(file_path)
#             # Scale pixmap to fit the label width while maintaining aspect ratio
#             scaled_pixmap = pixmap.scaled(self.diagram_label.width(), 300, Qt.KeepAspectRatio, Qt.SmoothTransformation)
#             self.diagram_label.setPixmap(scaled_pixmap)
#             self.diagram_label.setText("")
#         else:
#             self.diagram_label.clear()
#             self.diagram_label.setText("No Diagram Uploaded")

#     def sync_dropdowns(self):
#         scenario = self.scenario_combo.currentText()
#         data = self.logic_matrix[scenario]
        
#         # Temporarily block signals to prevent redundant update_summary calls during clear/add
#         self.world_combo.blockSignals(True)
#         self.world_combo.clear()
#         self.world_combo.addItems(data["worlds"])
#         self.world_combo.blockSignals(False)
        
#         # Update Learning Panel
#         self.obj_browser.setText(data["objective"])
#         self.hw_browser.setText(data["how_it_works"])
#         self.load_diagram_image(data.get("diagram_path", ""))

#         is_traj = (scenario == "Trajectory Planning")
#         is_pid = (scenario == "Wall Following")
        
#         self.traj_panel.setVisible(is_traj)
#         self.pid_panel.setVisible(is_pid)
#         self.empty_param_panel.setVisible(not is_traj and not is_pid)

#         if data["has_analytics"]:
#             self.open_dashboard_btn.setEnabled(True)
#             self.open_dashboard_btn.setStyleSheet("background-color: #8E44AD; color: white; font-weight: bold; height: 40px;")
#             self.open_dashboard_btn.setText("📊 OPEN ANALYSIS DASHBOARD")
#         else:
#             self.open_dashboard_btn.setEnabled(False)
#             self.open_dashboard_btn.setStyleSheet("background-color: #95A5A6; color: #ECF0F1; font-weight: bold; height: 40px;")
#             self.open_dashboard_btn.setText("Analytics Disabled for this Experiment")
            
#         self.update_summary()

#     def update_summary(self, status_override=None):
#         scenario = self.scenario_combo.currentText()
#         data = self.logic_matrix[scenario]
        
#         self.sum_exp_lbl.setText(f"Experiment : {scenario}")
#         self.sum_world_lbl.setText(f"World      : {self.world_combo.currentText()}")
#         self.sum_ctrl_lbl.setText(f"Controller : {data['controller']}")
        
#         if status_override:
#             if "Running" in status_override:
#                 self.sum_status_lbl.setStyleSheet("color: #E67E22;") # Orange
#             elif "Stopped" in status_override or "Completed" in status_override:
#                 self.sum_status_lbl.setStyleSheet("color: #34495E;") # Dark Blue
#             self.sum_status_lbl.setText(f"Status     : {status_override}")

#     def manual_add_wp(self):
#         x = round(self.wp_x.value(), 2)
#         y = round(self.wp_y.value(), 2)
#         self.current_waypoints.append([x, y])
#         self.waypoints_display.setText(f"Absolute: {str(self.current_waypoints)}")

#     def clear_wps(self):
#         self.current_waypoints.clear()
#         self.waypoints_display.setText("")

#     def launch_analytics_window(self):
#         self.analytics_window = AnalyticsWindow(self.scenario_combo.currentText())
#         self.analytics_window.show()

#     def handle_launch(self):
#         if self.ros_manager.is_running(): 
#             QMessageBox.warning(self, 'Error', 'Experiment is already running.')
#             return
#         scenario = self.scenario_combo.currentText()
#         node = self.custom_node if self.custom_node else self.logic_matrix[scenario]["node"]
        
#         waypoints_str = ""
#         if scenario == "Trajectory Planning" and self.current_waypoints:
#             try:
#                 flat_list = []
#                 for pt in self.current_waypoints:
#                     rel_x = round(pt[0] - ROBOT_START_X, 3)
#                     rel_y = round(pt[1] - ROBOT_START_Y, 3)
#                     flat_list.extend([rel_x, rel_y])
                
#                 waypoints_str = str(flat_list).replace(" ", "")
#                 print(f"[GUI MATH] Translated Absolute Webots points to Relative Node Points: {waypoints_str}")
#             except Exception as e:
#                 print(f"Error parsing waypoints: {e}")

#         self.update_summary("● Running")
#         self.ros_manager.launch(self.world_combo.currentText(), node, self.robot_name_input.text(), waypoints_str)

#     def handle_kill(self):
#         if not self.ros_manager.kill_all():
#             QMessageBox.warning(self, 'Error', 'No experiment currently running.')
#         else:
#             self.update_summary("● Stopped")

#     def send_pid_update(self):
#         if not self.ros_manager.is_running():
#             QMessageBox.warning(self, "Error", "Start the experiment first!")
#             return
#         node = self.logic_matrix[self.scenario_combo.currentText()]["node"]
#         self.ros_manager.set_pid_params(node, self.robot_name_input.text(), self.kp_box.value(), self.ki_box.value(), self.kd_box.value())

#     def open_waypoint_map(self):
#         map_dialog = InteractiveMapDialog(self.current_waypoints, self)
#         if map_dialog.exec_() == QDialog.Accepted:
#             self.current_waypoints = map_dialog.get_points()
#             if self.current_waypoints:
#                 self.waypoints_display.setText(f"Absolute: {str(self.current_waypoints)}")
#             else:
#                 self.waypoints_display.setText("")


# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     app.setStyle("Fusion") # Cleaner cross-platform look
#     window = AFIT_GCS()
#     window.show()
#     sys.exit(app.exec_())


































































































































































































# #!/usr/bin/env python3
# import sys
# import os
# import subprocess
# import signal
# import json
# import pandas as pd
# import numpy as np

# # --- GUI IMPORTS ---
# from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
#                              QComboBox, QPushButton, QTextEdit, QLabel, QMessageBox, 
#                              QFileDialog, QDoubleSpinBox, QGroupBox, QLineEdit, 
#                              QTableWidgetItem, QTableWidget, QHeaderView, QScrollArea,
#                              QDialog, QDialogButtonBox, QFrame, QTextBrowser)
# from PyQt5.QtGui import QFont, QPalette, QColor, QPixmap
# from PyQt5.QtCore import Qt, pyqtSignal

# # --- MATPLOTLIB IMPORTS ---
# from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
# from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
# from matplotlib.figure import Figure
# from matplotlib.patches import Rectangle


# # =====================================================================
# # --- CONFIGURATION & CONSTANTS ---
# # =====================================================================
# ROS2_SETUP_CMD = "source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash"
# DEFAULT_LOG_DIR = os.path.expanduser('~/ros2_ws/simulation_logs')

# # --- ROBOT STARTING POSITION (WEBOTS ABSOLUTE) ---
# ROBOT_START_X = -2.5
# ROBOT_START_Y = -2.5


# # =====================================================================
# # --- CORE LOGIC MODULES ---
# # =====================================================================

# class ROS2ProcessManager:
#     """Handles OS-level subprocess execution for ROS 2 commands."""
    
#     def __init__(self):
#         self.active_process = None

#     def is_running(self):
#         return self.active_process is not None

#     def launch(self, target_world, node, robot_name, waypoints_str=""):
#         robot_arg = f"robot_name:={robot_name}" if robot_name.strip() else ""
#         waypoint_arg = f" relative_waypoints:=\"{waypoints_str}\"" if waypoints_str else ""
#         launch_args = f"world:={target_world} scenario:={node} {robot_arg}{waypoint_arg}".strip()
#         full_cmd = f"{ROS2_SETUP_CMD} && ros2 launch robot_simulation simulation.launch.py {launch_args}"
        
#         print(f"IGNITION SEQUENCE INITIATED:\n{full_cmd}")
#         self.active_process = subprocess.Popen(
#             full_cmd, shell=True, executable='/bin/bash', preexec_fn=os.setsid
#         )

#     def kill_all(self):
#         if not self.is_running():
#             return False
            
#         print("\nTERMINATE: Initiating complete system sweep...")
#         try:
#             pgid = os.getpgid(self.active_process.pid)
#             os.killpg(pgid, signal.SIGTERM)
#             print(f"SUCCESS: Terminated core simulation group {pgid}")
#         except ProcessLookupError:
#             print("WARNING: Core simulation already down.")
#         finally:
#             self.active_process = None

#         print("SWEEP: Purging background terminal processes...")
#         try:
#             subprocess.run("pkill -f 'ros2 topic pub'", shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
#             global ROS2_SETUP_CMD
#             subprocess.run(f"{ROS2_SETUP_CMD} && ros2 daemon stop", shell=True, executable='/bin/bash', stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
#             print("SUCCESS: ROS 2 background discovery channels purged clean.\n")
#         except Exception as e:
#             print(f"WARNING: Automated sweep encountered an issue: {e}")

#         return True

#     def set_pid_params(self, node, robot_name, kp, ki, kd):
#         global ROS2_SETUP_CMD
#         target_node = f"{robot_name}/{node}" if robot_name.strip() else node
#         target_node = target_node.strip('/')
        
#         cmd = (f"({ROS2_SETUP_CMD} && ros2 param set /{target_node} Kp {kp}) & "
#                f"({ROS2_SETUP_CMD} && ros2 param set /{target_node} Ki {ki}) & "
#                f"({ROS2_SETUP_CMD} && ros2 param set /{target_node} Kd {kd})")
               
#         print(f"\n--- DISPATCHING ASYNC PID GAINS FOR /{target_node} ---")
#         subprocess.Popen(cmd, shell=True, executable='/bin/bash', stdout=subprocess.DEVNULL)

#     def publish_polynomial(self, poly_type):
#         global ROS2_SETUP_CMD
#         raw_cmd = f"{ROS2_SETUP_CMD} && ros2 topic pub --once /set_polynomial std_msgs/msg/String \"{{data: '{poly_type}'}}\""
#         subprocess.Popen(raw_cmd, shell=True, executable='/bin/bash', stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

#     def publish_waypoints(self, points):
#         if not self.is_running(): return False
            
#         global ROS2_SETUP_CMD
        
#         # Convert Absolute (Webots) back to Relative before publishing via topic
#         relative_points = [[pt[0] - ROBOT_START_X, pt[1] - ROBOT_START_Y] for pt in points]
#         rotated_points = [[round(pt[1], 3), round(-pt[0], 3)] for pt in relative_points]
        
#         points_str = json.dumps(rotated_points)
#         raw_cmd = f"{ROS2_SETUP_CMD} && ros2 topic pub --keep-alive 2 /gcs/via_points std_msgs/msg/String \"{{data: '{points_str}'}}\""
#         subprocess.Popen(raw_cmd, shell=True, executable='/bin/bash', stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
#         return True

# class SimulationAnalyzer:
#     @staticmethod
#     def calculate_wall_following_metrics(df):
#         time, errors = df['Time'].values, df['Error'].values
#         dt = np.mean(np.diff(time)) if len(time) > 1 else 0.1
        
#         rmse = np.sqrt(np.mean(errors**2))
#         iae = np.trapz(np.abs(errors), dx=dt)
#         ise = np.trapz(errors**2, dx=dt)
#         peak_err = np.max(np.abs(errors))
#         initial_err = abs(errors[0]) if len(errors) > 0 else 0.4
#         os_pct = ((peak_err - initial_err) / initial_err) * 100 if peak_err > initial_err else 0.0
#         return [iae, ise, os_pct, 0.0, rmse]


# # =====================================================================
# # --- GUI MODULES ---
# # =====================================================================

# class ClickableLabel(QLabel):
#     """Custom Label that emits a signal when clicked."""
#     clicked = pyqtSignal()
#     def mousePressEvent(self, event):
#         if event.button() == Qt.LeftButton:
#             self.clicked.emit()
#         super().mousePressEvent(event)

# class DiagramViewer(QDialog):
#     """Popup window for high-resolution diagram inspection."""
#     def __init__(self, image_path, parent=None):
#         super().__init__(parent)
#         self.setWindowTitle("System Diagram - Full Resolution")
#         self.resize(1000, 800)
        
#         layout = QVBoxLayout(self)
#         layout.setContentsMargins(0, 0, 0, 0)
        
#         self.label = QLabel()
#         pixmap = QPixmap(image_path)
#         self.label.setPixmap(pixmap)
        
#         # Explicitly lock the label to the exact pixel dimensions of the image.
#         # This absolutely forces the QScrollArea to map the correct panning bounds.
#         self.label.setFixedSize(pixmap.width(), pixmap.height())
        
#         # Scroll area allows panning around the massive unscaled image
#         scroll = QScrollArea()
#         scroll.setStyleSheet("background-color: #ECF0F1;")
#         scroll.setWidgetResizable(False) # Strict enforcement of the FixedSize above
#         scroll.setWidget(self.label)
#         layout.addWidget(scroll)


# class InteractiveMapDialog(QDialog):
#     def __init__(self, existing_points, parent=None):
#         super().__init__(parent)
#         self.setWindowTitle("Map Waypoint Selector")
#         self.resize(650, 650)
#         self.via_points = existing_points.copy()
        
#         layout = QVBoxLayout(self)
#         self.figure = Figure(figsize=(6, 6), dpi=100)
#         self.canvas = FigureCanvas(self.figure)
#         layout.addWidget(self.canvas)
        
#         self.coord_label = QLabel(f"Absolute Points: {self.via_points}" if self.via_points else "Click on the grid to add Webots absolute coordinates.")
#         self.coord_label.setAlignment(Qt.AlignCenter)
#         self.coord_label.setStyleSheet("font-weight: bold; font-size: 14px; padding: 10px;")
#         layout.addWidget(self.coord_label)
        
#         self.btn_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
#         self.clear_btn = QPushButton("Clear Points")
#         self.btn_box.addButton(self.clear_btn, QDialogButtonBox.ActionRole)
#         layout.addWidget(self.btn_box)
        
#         self.btn_box.accepted.connect(self.accept)
#         self.btn_box.rejected.connect(self.reject)
#         self.clear_btn.clicked.connect(self.clear_plot)
#         self.canvas.mpl_connect('button_press_event', self.on_click)
#         self.setup_plot()

#     def setup_plot(self):
#         self.figure.clear()
#         self.ax = self.figure.add_subplot(111)
#         self.ax.set_title("Interactive Arena Map (Webots 10x10 Arena)")
        
#         # Set bounds to encompass the new 10x10 area with a small buffer for visibility
#         self.ax.set_xlim(-5.5, 5.5) 
#         self.ax.set_ylim(-5.5, 5.5)
#         self.ax.set_xlabel("Webots X (meters)")
#         self.ax.set_ylabel("Webots Y (meters)")
#         self.ax.grid(True, linestyle='--', alpha=0.5)
#         self.ax.set_aspect('equal')
        
#         # 1. Draw the exact center of the Webots world (0,0)
#         self.ax.plot(0, 0, 'k+', markersize=15, markeredgewidth=2, label="Arena Center (0,0)")
        
#         # 2. Draw the solid physical walls of the 10x10 arena (from -5.0 to 5.0)
#         walls = Rectangle((-5.0, -5.0), 10.0, 10.0, fill=False, edgecolor='black', linewidth=3)
#         self.ax.add_patch(walls)

#         # Draw the robot physically where it starts in Webots
#         self.ax.plot(ROBOT_START_X, ROBOT_START_Y, 'gX', markersize=12, label=f"Robot Start ({ROBOT_START_X}, {ROBOT_START_Y})")
#         self.ax.legend(loc="upper right", fontsize='small')
        
#         # Draw existing points
#         for pt in self.via_points:
#             self.ax.plot(pt[0], pt[1], 'ro', markersize=8)
#         if len(self.via_points) > 1:
#             pts = np.array(self.via_points)
#             self.ax.plot(pts[:,0], pts[:,1], 'b-', alpha=0.5)
            
#         self.canvas.draw()

#     def on_click(self, event):
#         if event.inaxes != self.ax: return
#         x, y = round(event.xdata, 2), round(event.ydata, 2)
        
#         # Optional: Prevent clicks completely outside the physical 10x10 wall bounds
#         if x < -5.0 or x > 5.0 or y < -5.0 or y > 5.0:
#             return
            
#         self.via_points.append([x, y])
#         self.coord_label.setText(f"Absolute Points: {self.via_points}")
#         self.ax.plot(x, y, 'ro', markersize=8)
#         if len(self.via_points) > 1:
#             prev_point = self.via_points[-2]
#             self.ax.plot([prev_point[0], x], [prev_point[1], y], 'b-', alpha=0.5)
#         self.canvas.draw()

#     def clear_plot(self):
#         self.via_points.clear()
#         self.coord_label.setText("Click on the grid to add Webots absolute coordinates.")
#         self.setup_plot()
        
#     def get_points(self): return self.via_points


# class AnalyticsWindow(QWidget):
#     def __init__(self, scenario_name):
#         super().__init__()
#         self.scenario_name = scenario_name
#         self.setWindowTitle(f"Performance Evaluation Dashboard - {self.scenario_name}")
#         self.resize(1050, 750) 
#         self.init_ui()

#     def init_ui(self):
#         self.outer_layout = QVBoxLayout(self)
#         self.outer_layout.setContentsMargins(0, 0, 0, 0)
        
#         self.scroll = QScrollArea()
#         self.scroll.setWidgetResizable(True)
#         self.scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        
#         self.content_container = QWidget()
#         self.page_layout = QVBoxLayout(self.content_container)
#         self.page_layout.setSpacing(20)
        
#         self.extract_btn = QPushButton(f"📂 Load {self.scenario_name} Experiment Data")
#         self.extract_btn.setStyleSheet("background-color: #2980b9; color: white; font-weight: bold; padding: 15px; font-size: 14px;")
#         self.extract_btn.clicked.connect(self.load_and_plot_file)
#         self.page_layout.addWidget(self.extract_btn)

#         self.stats_label = QLabel("Experiment Log: Pending Selection")
#         self.stats_label.setFont(QFont("Courier", 11))
#         self.stats_label.setAlignment(Qt.AlignCenter)
#         self.stats_label.setStyleSheet("background-color: #ecf0f1; padding: 10px; border: 1px solid #bdc3c7;")
#         self.page_layout.addWidget(self.stats_label)

#         self.page_layout.addWidget(QLabel("📊 ENGINEERING PERFORMANCE INDICES:"))
#         self.metrics_table = QTableWidget(1, 2) 
#         self.metrics_table.setHorizontalHeaderLabels(["Settling (s)", "RMSE (m)"])
#         self.metrics_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
#         self.metrics_table.setFixedHeight(85)
#         self.metrics_table.setStyleSheet("background-color: #f8f9fa; color: black; font-weight: bold;")
#         self.page_layout.addWidget(self.metrics_table)

#         self.page_layout.addWidget(QLabel("📈 EXPERIMENT RESPONSE PLOTS:"))
#         self.figure = Figure(figsize=(10, 10), dpi=100)
#         self.canvas = FigureCanvas(self.figure)
#         self.toolbar = NavigationToolbar(self.canvas, self)
#         self.canvas.setMinimumHeight(800) 
        
#         self.page_layout.addWidget(self.toolbar)
#         self.page_layout.addWidget(self.canvas)
#         self.scroll.setWidget(self.content_container)
#         self.outer_layout.addWidget(self.scroll)

#     def load_and_plot_file(self):
#         csv_filename, _ = QFileDialog.getOpenFileName(self, f"Open {self.scenario_name} Log", DEFAULT_LOG_DIR, "CSV Files (*.csv)")
#         if not csv_filename: return

#         try:
#             df = pd.read_csv(csv_filename)
#             df.columns = df.columns.str.strip() # Fixes hidden spaces in CSV headers
#         except Exception as e:
#             QMessageBox.critical(self, "Read Error", f"Failed to parse CSV:\n{e}")
#             return
            
#         if self.scenario_name == "Wall Following":
#             self.plot_wall_following(df, csv_filename)
#         elif self.scenario_name == "Obstacle Avoidance":
#             self.plot_obstacle_avoidance(df, csv_filename)
#         elif self.scenario_name == "Trajectory Planning":
#             self.plot_trajectory_planning(df, csv_filename)

#         self.figure.tight_layout(pad=3.0)
#         self.canvas.draw()

#     def update_table(self, values):
#         for i, val in enumerate(values):
#             self.metrics_table.setItem(0, i, QTableWidgetItem(f"{val}"))
#     def plot_wall_following(self, df, path):
#         self.figure.clear()
        
#         required = ['Time', 'Desired_Distance', 'Measured_Distance', 'Angular_Vel', 'X', 'Y']
#         if not all(col in df.columns for col in required):
#             QMessageBox.critical(self, "Data Error", f"CSV missing columns for new plotter.\nRequires: {required}")
#             return

#         self.stats_label.setText(f"Wall Following Analysis | File: {os.path.basename(path)}")

#         times = df['Time'].values
#         measured = df['Measured_Distance'].values
#         setpoint = df['Desired_Distance'].iloc[0]

#         # --- CALCULATE METRICS ---
#         initial_val = measured[0]
#         diff = abs(initial_val - setpoint)
        
#         try:
#             # Find time indices for 10% and 90% crossing
#             t_10 = times[np.where(abs(measured - initial_val) >= 0.10 * diff)[0][0]]
#             t_90 = times[np.where(abs(measured - initial_val) >= 0.90 * diff)[0][0]]
#             rise_time = t_90 - t_10
#         except IndexError:
#             t_10, t_90, rise_time = None, None, 0.0

#         rmse = np.sqrt(np.mean((measured - setpoint)**2))
#         self.update_table([f"Rise (10-90): {rise_time:.2f}s", f"RMSE: {rmse:.4f}m"])

#         # Setup Subplots (1 Top Wide, 2 Bottom)
#         ax1 = self.figure.add_subplot(211)
#         ax2 = self.figure.add_subplot(223)
#         ax3 = self.figure.add_subplot(224)

#         # ---------------------------------------------------------
#         # FIGURE 1: Wall Distance Response (ax1)
#         # ---------------------------------------------------------
#         tolerance = 0.05 * setpoint
#         ax1.fill_between(times, setpoint - tolerance, setpoint + tolerance, 
#                          color='gray', alpha=0.2, label='5% Tolerance Band')
        
#         ax1.plot(df['Time'], measured, color='tab:blue', linewidth=2, label='Measured Distance')
#         ax1.axhline(setpoint, color='tab:red', linestyle='--', linewidth=2, label='Setpoint')

#         if t_10 and t_90:
#             ax1.axvline(t_10, color='green', linestyle=':', label='Rise Time Start (10%)')
#             ax1.axvline(t_90, color='green', linestyle=':', label='Rise Time End (90%)')

#         ax1.set_title('Figure 1 - Wall Distance Response')
#         ax1.set_xlabel('Time (s)')
#         ax1.set_ylabel('Distance to wall (m)')
#         ax1.legend(loc='upper right', fontsize=9)
#         ax1.grid(True, alpha=0.5)

#         # ---------------------------------------------------------
#         # FIGURE 2: Angular Velocity (ax2)
#         # ---------------------------------------------------------
#         ax2.plot(df['Time'], df['Angular_Vel'], color='tab:orange', linewidth=2, label='Angular Velocity')
#         ax2.axhline(0, color='black', linestyle='--', linewidth=1.5)
#         ax2.set_title('Figure 2 - Angular Velocity vs Time')
#         ax2.set_xlabel('Time (s)')
#         ax2.set_ylabel('Angular Velocity (rad/s)')
#         ax2.legend(loc='upper right')
#         ax2.grid(True, alpha=0.5)

#         # ---------------------------------------------------------
#         # FIGURE 5: Robot Trajectory (ax3)
#         # ---------------------------------------------------------
#         ax3.plot(df['X'], df['Y'], color='blue', linewidth=2, label='Robot Trajectory')
#         ax3.scatter(df['X'].iloc[0], df['Y'].iloc[0], color='green', marker='o', s=100, label='Start', zorder=5)
#         ax3.scatter(df['X'].iloc[-1], df['Y'].iloc[-1], color='red', marker='X', s=100, label='End', zorder=5)
#         ax3.set_title('Figure 5 - Robot Trajectory')
#         ax3.set_xlabel('X Position (m)')
#         ax3.set_ylabel('Y Position (m)')
#         ax3.axis('equal')
#         ax3.legend(loc='upper right')
#         ax3.grid(True, alpha=0.5)

#     def plot_trajectory_planning(self, df, path):
#         self.figure.clear()
#         self.stats_label.setText(f"📐 Pure Pursuit Analysis | File: {os.path.basename(path)}")

#         if df.empty:
#             QMessageBox.critical(self, "Empty File", "The CSV has headers but no data.")
#             return

#         required = ['Time', 'Ref_X', 'Ref_Y', 'Robot_X', 'Robot_Y', 'Cross_Track_Error', 'Heading_Error']
#         if not all(col in df.columns for col in required):
#             QMessageBox.critical(self, "Data Error", f"CSV missing columns for new plotter.\nRequires: {required}")
#             return

#         time = df['Time'].values
#         cte = df['Cross_Track_Error'].values

#         rmse_cte = np.sqrt(np.mean(cte**2))
#         max_cte = np.max(np.abs(cte))
#         self.update_table([f"Max: {max_cte:.4f}m", f"RMSE: {rmse_cte:.4f}m"])

#         # Setup Subplots (1 Top Wide, 2 Bottom)
#         ax1 = self.figure.add_subplot(211)
#         ax2 = self.figure.add_subplot(223)
#         ax3 = self.figure.add_subplot(224)

#         # ---------------------------------------------------------
#         # FIGURE 1: Reference Path vs Actual Trajectory (ax1)
#         # ---------------------------------------------------------
#         ax1.plot(df['Ref_X'], df['Ref_Y'], color='black', linestyle='--', linewidth=2, label='Reference Path')
#         ax1.plot(df['Robot_X'], df['Robot_Y'], color='tab:blue', linewidth=2, label='Actual Trajectory')
        
#         ax1.scatter(df['Ref_X'].iloc[0], df['Ref_Y'].iloc[0], color='green', marker='o', s=120, label='Start Point', zorder=5)
#         ax1.scatter(df['Ref_X'].iloc[-1], df['Ref_Y'].iloc[-1], color='red', marker='X', s=120, label='Goal Point', zorder=5)
        
#         ax1.set_title('Figure 1 - Reference Path vs Actual Robot Trajectory')
#         ax1.set_xlabel('X Position (m)')
#         ax1.set_ylabel('Y Position (m)')
#         ax1.axis('equal') 
#         ax1.grid(True, alpha=0.5)
#         ax1.legend()

#         # ---------------------------------------------------------
#         # FIGURE 2: Cross-Track Error vs Time (ax2)
#         # ---------------------------------------------------------
#         ax2.plot(time, cte, color='tab:red', linewidth=2, label='Cross-Track Error')
#         ax2.axhline(0, color='black', linestyle='-', linewidth=1)
        
#         ax2.set_title('Figure 2 - Cross-Track Error vs Time')
#         ax2.set_xlabel('Time (s)')
#         ax2.set_ylabel('Cross-track error (m)')
#         ax2.grid(True, alpha=0.5)
#         ax2.legend()

#         # ---------------------------------------------------------
#         # FIGURE 3: Heading Error vs Time (ax3)
#         # ---------------------------------------------------------
#         ax3.plot(time, df['Heading_Error'], color='tab:purple', linewidth=2, label='Heading Error')
#         ax3.axhline(0, color='black', linestyle='-', linewidth=1)
        
#         ax3.set_title('Figure 3 - Heading Error vs Time')
#         ax3.set_xlabel('Time (s)')
#         ax3.set_ylabel('Heading error (degrees)')
#         ax3.grid(True, alpha=0.5)
#         ax3.legend()
    
#     def plot_obstacle_avoidance(self, df, path):
#         self.figure.clear()
#         time, clearance, ang_vel = df['Time'].values, df['Clearance'].values, df['Angular_Vel'].values
#         min_clear = np.min(clearance)
#         self.stats_label.setText(f"🚧 OBSTACLE AVOIDANCE 🚧 | File: {os.path.basename(path)}")
#         self.update_table([f"{min_clear:.4f}", f"{time[-1]:.1f}"])

#         ax1 = self.figure.add_subplot(211)
#         ax2 = self.figure.add_subplot(212)

#         ax1.plot(time, clearance, color='#e67e22', linewidth=2, label='Clearance')
#         ax1.axhline(0.2, color='red', linestyle='--', label='Safety Limit')
#         ax1.set(title="Proximity Metrics", ylabel="Dist (m)")
#         ax1.grid(True, linestyle=':', alpha=0.7)
        
#         ax2.plot(time, ang_vel, color='#9b59b6', label='Yaw Rate')
#         ax2.set(title="Maneuver Intensity", xlabel="Time (s)", ylabel="Rad/s")
#         ax2.grid(True, linestyle=':', alpha=0.7)

    


# class AFIT_GCS(QWidget):
#     def __init__(self):
#         super().__init__()
#         self.ros_manager = ROS2ProcessManager()
#         self.custom_node = ""
#         self.current_waypoints = []
#         self.current_diagram_path = ""
        
#         self.logic_matrix = {
#             "Teleop": {
#                 "worlds": ["obstacle_course.wbt", "garden.wbt", "wall_following_1.wbt"], 
#                 "node": "teleop_node", 
#                 "controller": "Manual Control",
#                 "has_analytics": False,
#                 "objective": "Understand how manual keyboard commands are translated into robot motion through the ROS2 communication framework and the differential-drive controller.",
#                 "how_it_works": "• Press the W, A, S or D keys to control the robot.\n• The teleoperation node converts the selected key into linear and angular velocity values.\n• A Twist velocity command is published through the ROS2 topic (/diffdrive_controller/cmd_vel_unstamped)\n• The differential drive controller converts the velocity command into wheel velocities.\n• The robot moves in Webots.",
#                 "diagram_path": "diagram_teleop.png"
#             },
#             "Trajectory Planning": {
#                 "worlds": ["obstacle_course.wbt"], 
#                 "node": "niku_controller_viapoints", 
#                 "controller": "Pure Pursuit Spline",
#                 "has_analytics": True,
#                 "objective": "Understand how a robot generates a smooth trajectory through user-defined waypoints and follows it using the Pure Pursuit controller.",
#                 "how_it_works": "• Select waypoints.\n• A Piecewise Quintic Polynomial trajectory is generated.\n• Pure Pursuit selects a look-ahead point.\n• The controller computes the steering command.\n• The robot follows the planned trajectory.",
#                 "diagram_path": "diagram_trajectory.png"
#             },
#             "Wall Following": {
#                 "worlds": ["wall_following_1.wbt", "wall_following_2.wbt"], 
#                 "node": "wall_follower", 
#                 "controller": "PID Control",
#                 "has_analytics": True,
#                 "objective": "Investigate how feedback control enables a robot to maintain a desired distance from a wall by adjusting the PID controller gains.",
#                 "how_it_works": "• The laser scanner measures the wall distance.\n• The measured distance is compared with the desired distance.\n• The PID controller computes a steering correction.\n• The steering command is sent to the robot.\n• The process repeats continuously.",
#                 "diagram_path": "diagram_wall_following.jpeg" 
#             },
#             # "Obstacle Avoidance": {
#             #     "worlds": ["boxes_dense.wbt", "obstacle_course.wbt", "garden.wbt", "wall_following.wbt"], 
#             #     "node": "obstacle_avoidance", 
#             #     "controller": "Reactive Logic",
#             #     "has_analytics": True,
#             #     "objective": "Evaluate real-time LiDAR-based obstacle detection and reactive maneuver intensity to prevent collisions.",
#             #     "how_it_works": "• LiDAR scans the immediate environment.\n• Distance to nearest obstacle is calculated.\n• If distance < safety threshold, compute avoidance vector.\n• Publish angular velocity override command.\n• Robot steers away from collision.",
#             #     "diagram_path": "diagram_obstacle_avoidance.png"
#             # }
#         }
#         self.init_ui()

#     def init_ui(self):
#         self.setWindowTitle("Integrated Mobile Robot Experimentation Platform")
#         self.resize(750, 950) 
        
#         self.setStyleSheet("""
#             QWidget { background-color: #ECF0F1; } 
#             QLabel { font-weight: bold; color: #2C3E50; }
#             QGroupBox { font-weight: bold; border: 2px solid #BDC3C7; border-radius: 5px; margin-top: 1ex; }
#             QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; padding: 0 3px; }
#             QComboBox, QLineEdit, QDoubleSpinBox { background-color: white; color: black; border: 1px solid #BDC3C7; border-radius: 3px; padding: 2px;}
#             QTextBrowser { background-color: #F8F9FA; font-style: italic; border: 1px solid #BDC3C7; }
#         """)

#         self.scroll_main = QScrollArea()
#         self.scroll_main.setWidgetResizable(True)
#         self.main_container = QWidget()
#         main_layout = QVBoxLayout(self.main_container)

#         self.logo_label = QLabel()
#         self.logo_label.setAlignment(Qt.AlignCenter)
#         if os.path.exists("afit_logo.png"):
#             pix = QPixmap("afit_logo.png")
#             self.logo_label.setPixmap(pix.scaledToHeight(120, Qt.SmoothTransformation))
#         else:
#             self.logo_label.setText("[afit_logo.png missing]")
#         main_layout.addWidget(self.logo_label)

#         group_lbl = QLabel("MCT GROUP B")
#         group_lbl.setAlignment(Qt.AlignCenter)
#         group_lbl.setStyleSheet("font-size: 18px; font-weight: bold; color: #2C3E50; margin-bottom: 10px;")
#         main_layout.addWidget(group_lbl)
        
#         title_lbl = QLabel("INTEGRATED MOBILE ROBOT EXPERIMENTATION PLATFORM")
#         title_lbl.setAlignment(Qt.AlignCenter)
#         title_lbl.setStyleSheet("font-size: 16px; background-color: #34495E; color: white; padding: 10px; border-radius: 5px;")
#         main_layout.addWidget(title_lbl)

#         main_layout.addWidget(self.create_summary_panel())
#         main_layout.addWidget(self.create_configuration_panel())
#         main_layout.addWidget(self.create_execution_panel())
#         main_layout.addWidget(self.create_evaluation_panel())
        
#         self.scroll_main.setWidget(self.main_container)
        
#         central_layout = QVBoxLayout(self)
#         central_layout.setContentsMargins(0, 0, 0, 0)
#         central_layout.addWidget(self.scroll_main)

#         self.sync_dropdowns()

#     def create_summary_panel(self):
#         panel = QGroupBox("CURRENT EXPERIMENT SUMMARY")
#         layout = QVBoxLayout(panel)
        
#         self.sum_exp_lbl = QLabel("Experiment : ")
#         self.sum_world_lbl = QLabel("World      : ")
#         self.sum_ctrl_lbl = QLabel("Controller : ")
#         self.sum_status_lbl = QLabel("Status     : ● Ready")
#         self.sum_status_lbl.setStyleSheet("color: #27AE60;") 
        
#         for lbl in [self.sum_exp_lbl, self.sum_world_lbl, self.sum_ctrl_lbl, self.sum_status_lbl]:
#             lbl.setFont(QFont("Courier", 10))
#             layout.addWidget(lbl)
            
#         return panel

#     def create_configuration_panel(self):
#         panel = QGroupBox("1. EXPERIMENT CONFIGURATION")
#         layout = QVBoxLayout(panel)
        
#         row_selectors = QHBoxLayout()
#         self.scenario_combo = QComboBox()
#         self.scenario_combo.addItems(self.logic_matrix.keys())
#         self.world_combo = QComboBox()
        
#         row_selectors.addWidget(QLabel("Experiment Type:"))
#         row_selectors.addWidget(self.scenario_combo, stretch=2)
#         row_selectors.addWidget(QLabel("Webots World:"))
#         row_selectors.addWidget(self.world_combo, stretch=2)
#         layout.addLayout(row_selectors)

#         self.learning_panel = QGroupBox("Learning Panel")
#         lp_layout = QVBoxLayout(self.learning_panel)
        
#         lp_layout.addWidget(QLabel("Experiment Objective"))
#         self.obj_browser = QTextBrowser()
#         self.obj_browser.setMaximumHeight(50)
#         lp_layout.addWidget(self.obj_browser)
        
#         lp_layout.addWidget(QLabel("How It Works"))
#         self.hw_browser = QTextBrowser()
#         self.hw_browser.setMaximumHeight(130)
#         lp_layout.addWidget(self.hw_browser)
        
#         lp_layout.addWidget(QLabel("System Diagram (Click to enlarge)"))
        
#         self.diagram_label = ClickableLabel("Loading Diagram...")
#         self.diagram_label.setAlignment(Qt.AlignCenter)
#         self.diagram_label.setMinimumHeight(280)
#         self.diagram_label.setStyleSheet("background-color: #FFFFFF; border: 1px dashed #BDC3C7;")
#         self.diagram_label.clicked.connect(self.open_diagram_popup)
        
#         lp_layout.addWidget(self.diagram_label)
        
#         layout.addWidget(self.learning_panel)

#         row_robot = QHBoxLayout()
#         self.robot_name_input = QLineEdit()
#         self.robot_name_input.setPlaceholderText("Robot Name in Webots (Default: robot_a)")
#         row_robot.addWidget(QLabel("Robot Target:"))
#         row_robot.addWidget(self.robot_name_input)
#         layout.addLayout(row_robot)

#         layout.addWidget(QLabel("Experiment Parameters:"))
#         self.pid_panel = self.create_pid_panel()
#         self.traj_panel = self.create_traj_panel()
#         self.empty_param_panel = QLabel("No runtime parameters required for this experiment.")
#         self.empty_param_panel.setStyleSheet("color: #7F8C8D; font-style: italic;")
        
#         layout.addWidget(self.pid_panel)
#         layout.addWidget(self.traj_panel)
#         layout.addWidget(self.empty_param_panel)

#         self.scenario_combo.currentTextChanged.connect(self.sync_dropdowns)
#         self.world_combo.currentTextChanged.connect(self.update_summary)
        
#         return panel

#     def create_execution_panel(self):
#         panel = QGroupBox("2. EXPERIMENT EXECUTION")
#         layout = QHBoxLayout(panel)
        
#         self.launch_btn = QPushButton("▶ START EXPERIMENT")
#         self.launch_btn.setStyleSheet("background-color: #27AE60; color: white; font-weight: bold; height: 35px;")
        
#         self.kill_btn = QPushButton("■ STOP EXPERIMENT")
#         self.kill_btn.setStyleSheet("background-color: #C0392B; color: white; font-weight: bold; height: 35px;")
        
#         layout.addWidget(self.launch_btn)
#         layout.addWidget(self.kill_btn)
        
#         self.launch_btn.clicked.connect(self.handle_launch)
#         self.kill_btn.clicked.connect(self.handle_kill)
        
#         return panel

#     def create_evaluation_panel(self):
#         panel = QGroupBox("3. PERFORMANCE EVALUATION")
#         layout = QVBoxLayout(panel)
        
#         desc = QLabel("(Plots and engineering performance metrics are computed in the dashboard)")
#         desc.setStyleSheet("color: #7F8C8D;")
#         layout.addWidget(desc)
        
#         self.open_dashboard_btn = QPushButton("📊 OPEN ANALYSIS DASHBOARD")
#         self.open_dashboard_btn.setStyleSheet("background-color: #8E44AD; color: white; font-weight: bold; height: 40px;")
#         layout.addWidget(self.open_dashboard_btn)
        
#         self.open_dashboard_btn.clicked.connect(self.launch_analytics_window)
        
#         return panel

#     def create_pid_panel(self):
#         panel = QWidget()
#         layout = QHBoxLayout(panel)
#         layout.setContentsMargins(0,0,0,0)
#         self.kp_box, self.ki_box, self.kd_box = QDoubleSpinBox(), QDoubleSpinBox(), QDoubleSpinBox()
        
#         for box in (self.kp_box, self.ki_box, self.kd_box):
#             box.setDecimals(3)
#             box.setRange(0.0, 10.0)
#             box.setSingleStep(0.05)
#             box.setValue(1.0)
            
#         self.update_pid_btn = QPushButton("UPDATE GAINS")
#         self.update_pid_btn.setStyleSheet("background-color: #2980B9; color: white; font-weight: bold;")
#         self.update_pid_btn.clicked.connect(self.send_pid_update)
        
#         layout.addWidget(QLabel("Kp:")); layout.addWidget(self.kp_box)
#         layout.addWidget(QLabel("Ki:")); layout.addWidget(self.ki_box)
#         layout.addWidget(QLabel("Kd:")); layout.addWidget(self.kd_box)
#         layout.addWidget(self.update_pid_btn)
#         return panel

#     def create_traj_panel(self):
#         panel = QWidget()
#         layout = QVBoxLayout(panel)
#         layout.setContentsMargins(0,0,0,0)
        
#         row1 = QHBoxLayout()
#         self.open_map_btn = QPushButton("🗺️ Open Waypoint Selector")
#         self.open_map_btn.setStyleSheet("background-color: #E67E22; color: white; font-weight: bold;")
#         self.open_map_btn.clicked.connect(self.open_waypoint_map)
#         row1.addWidget(self.open_map_btn)
#         layout.addLayout(row1)

#         row2 = QHBoxLayout()
#         self.wp_x = QDoubleSpinBox()
#         self.wp_x.setRange(-5.0, 5.0) 
#         self.wp_x.setSingleStep(0.25)
        
#         self.wp_y = QDoubleSpinBox()
#         self.wp_y.setRange(-5.0, 5.0) 
#         self.wp_y.setSingleStep(0.25)

#         self.add_wp_btn = QPushButton("+ Add Pt")
#         self.clear_wp_btn = QPushButton("Clear")
#         self.add_wp_btn.clicked.connect(self.manual_add_wp)
#         self.clear_wp_btn.clicked.connect(self.clear_wps)

#         row2.addWidget(QLabel("X:")); row2.addWidget(self.wp_x)
#         row2.addWidget(QLabel("Y:")); row2.addWidget(self.wp_y)
#         row2.addWidget(self.add_wp_btn)
#         row2.addWidget(self.clear_wp_btn)
#         layout.addLayout(row2)

#         self.waypoints_display = QLineEdit()
#         self.waypoints_display.setReadOnly(True)
#         self.waypoints_display.setPlaceholderText("No absolute points set...")
#         layout.addWidget(self.waypoints_display)

#         return panel

#     # --- BEHAVIOR & LOGIC ---

#     def load_diagram_image(self, file_path):
#         self.current_diagram_path = file_path
#         if file_path and os.path.exists(file_path):
#             pixmap = QPixmap(file_path)
#             scaled_pixmap = pixmap.scaled(600, 280, Qt.KeepAspectRatio, Qt.SmoothTransformation)
#             self.diagram_label.setPixmap(scaled_pixmap)
#             self.diagram_label.setCursor(Qt.PointingHandCursor) 
#         else:
#             self.diagram_label.clear()
#             self.diagram_label.setText(f"Missing File: {file_path}")
#             self.diagram_label.setCursor(Qt.ArrowCursor)

#     def open_diagram_popup(self):
#         if self.current_diagram_path and os.path.exists(self.current_diagram_path):
#             self.diagram_viewer = DiagramViewer(self.current_diagram_path, self)
#             self.diagram_viewer.show()

#     def sync_dropdowns(self):
#         scenario = self.scenario_combo.currentText()
#         data = self.logic_matrix[scenario]
        
#         self.world_combo.blockSignals(True)
#         self.world_combo.clear()
#         self.world_combo.addItems(data["worlds"])
#         self.world_combo.blockSignals(False)
        
#         self.obj_browser.setText(data["objective"])
#         self.hw_browser.setText(data["how_it_works"])
#         self.load_diagram_image(data.get("diagram_path", ""))

#         is_traj = (scenario == "Trajectory Planning")
#         is_pid = (scenario == "Wall Following")
        
#         self.traj_panel.setVisible(is_traj)
#         self.pid_panel.setVisible(is_pid)
#         self.empty_param_panel.setVisible(not is_traj and not is_pid)

#         if data["has_analytics"]:
#             self.open_dashboard_btn.setEnabled(True)
#             self.open_dashboard_btn.setStyleSheet("background-color: #8E44AD; color: white; font-weight: bold; height: 40px;")
#             self.open_dashboard_btn.setText("📊 OPEN ANALYSIS DASHBOARD")
#         else:
#             self.open_dashboard_btn.setEnabled(False)
#             self.open_dashboard_btn.setStyleSheet("background-color: #95A5A6; color: #ECF0F1; font-weight: bold; height: 40px;")
#             self.open_dashboard_btn.setText("Analytics Disabled for this Experiment")
            
#         self.update_summary()

#     def update_summary(self, status_override=None):
#         scenario = self.scenario_combo.currentText()
#         data = self.logic_matrix[scenario]
        
#         self.sum_exp_lbl.setText(f"Experiment : {scenario}")
#         self.sum_world_lbl.setText(f"World      : {self.world_combo.currentText()}")
#         self.sum_ctrl_lbl.setText(f"Controller : {data['controller']}")
        
#         if status_override:
#             if "Running" in status_override:
#                 self.sum_status_lbl.setStyleSheet("color: #E67E22;") 
#             elif "Stopped" in status_override or "Completed" in status_override:
#                 self.sum_status_lbl.setStyleSheet("color: #34495E;") 
#             self.sum_status_lbl.setText(f"Status     : {status_override}")

#     def manual_add_wp(self):
#         x = round(self.wp_x.value(), 2)
#         y = round(self.wp_y.value(), 2)
#         self.current_waypoints.append([x, y])
#         self.waypoints_display.setText(f"Absolute: {str(self.current_waypoints)}")

#     def clear_wps(self):
#         self.current_waypoints.clear()
#         self.waypoints_display.setText("")

#     def launch_analytics_window(self):
#         self.analytics_window = AnalyticsWindow(self.scenario_combo.currentText())
#         self.analytics_window.show()

#     def handle_launch(self):
#         if self.ros_manager.is_running(): 
#             QMessageBox.warning(self, 'Error', 'Experiment is already running.')
#             return
#         scenario = self.scenario_combo.currentText()
#         node = self.custom_node if self.custom_node else self.logic_matrix[scenario]["node"]
        
#         waypoints_str = ""
#         if scenario == "Trajectory Planning" and self.current_waypoints:
#             try:
#                 flat_list = []
#                 for pt in self.current_waypoints:
#                     rel_x = round(pt[0] - ROBOT_START_X, 3)
#                     rel_y = round(pt[1] - ROBOT_START_Y, 3)
#                     flat_list.extend([rel_x, rel_y])
                
#                 waypoints_str = str(flat_list).replace(" ", "")
#                 print(f"[GUI MATH] Translated Absolute Webots points to Relative Node Points: {waypoints_str}")
#             except Exception as e:
#                 print(f"Error parsing waypoints: {e}")

#         self.update_summary("● Running")
#         self.ros_manager.launch(self.world_combo.currentText(), node, self.robot_name_input.text(), waypoints_str)

#     def handle_kill(self):
#         if not self.ros_manager.kill_all():
#             QMessageBox.warning(self, 'Error', 'No experiment currently running.')
#         else:
#             self.update_summary("● Stopped")

#     def send_pid_update(self):
#         if not self.ros_manager.is_running():
#             QMessageBox.warning(self, "Error", "Start the experiment first!")
#             return
#         node = self.logic_matrix[self.scenario_combo.currentText()]["node"]
#         self.ros_manager.set_pid_params(node, self.robot_name_input.text(), self.kp_box.value(), self.ki_box.value(), self.kd_box.value())

#     def open_waypoint_map(self):
#         map_dialog = InteractiveMapDialog(self.current_waypoints, self)
#         if map_dialog.exec_() == QDialog.Accepted:
#             self.current_waypoints = map_dialog.get_points()
#             if self.current_waypoints:
#                 self.waypoints_display.setText(f"Absolute: {str(self.current_waypoints)}")
#             else:
#                 self.waypoints_display.setText("")


# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     app.setStyle("Fusion") 
#     window = AFIT_GCS()
#     window.show()
#     sys.exit(app.exec_())










#educational panel, but tooltips not working
#!/usr/bin/env python3
import sys
import os
import subprocess
import signal
import json
import pandas as pd
import numpy as np

# --- GUI IMPORTS ---
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                             QComboBox, QPushButton, QTextEdit, QLabel, QMessageBox, 
                             QFileDialog, QDoubleSpinBox, QGroupBox, QLineEdit, 
                             QTableWidgetItem, QTableWidget, QHeaderView, QScrollArea,
                             QDialog, QDialogButtonBox, QFrame, QTextBrowser)
from PyQt5.QtGui import QFont, QPalette, QColor, QPixmap
from PyQt5.QtCore import Qt, pyqtSignal

# --- MATPLOTLIB IMPORTS ---
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
from matplotlib.patches import Rectangle


# =====================================================================
# --- CONFIGURATION & CONSTANTS ---
# =====================================================================
ROS2_SETUP_CMD = "source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash"
DEFAULT_LOG_DIR = os.path.expanduser('~/ros2_ws/simulation_logs')

# --- ROBOT STARTING POSITION (WEBOTS ABSOLUTE) ---
ROBOT_START_X = -2.5
ROBOT_START_Y = -2.5


# =====================================================================
# --- CORE LOGIC MODULES ---
# =====================================================================

class ROS2ProcessManager:
    """Handles OS-level subprocess execution for ROS 2 commands."""
    
    def __init__(self):
        self.active_process = None

    def is_running(self):
        return self.active_process is not None

    def launch(self, target_world, node, robot_name, waypoints_str=""):
        robot_arg = f"robot_name:={robot_name}" if robot_name.strip() else ""
        waypoint_arg = f" relative_waypoints:=\"{waypoints_str}\"" if waypoints_str else ""
        launch_args = f"world:={target_world} scenario:={node} {robot_arg}{waypoint_arg}".strip()
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

        print("SWEEP: Purging background terminal processes...")
        try:
            subprocess.run("pkill -f 'ros2 topic pub'", shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            global ROS2_SETUP_CMD
            subprocess.run(f"{ROS2_SETUP_CMD} && ros2 daemon stop", shell=True, executable='/bin/bash', stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print("SUCCESS: ROS 2 background discovery channels purged clean.\n")
        except Exception as e:
            print(f"WARNING: Automated sweep encountered an issue: {e}")

        return True

    def set_pid_params(self, node, robot_name, kp, ki, kd):
        global ROS2_SETUP_CMD
        target_node = f"{robot_name}/{node}" if robot_name.strip() else node
        target_node = target_node.strip('/')
        
        cmd = (f"({ROS2_SETUP_CMD} && ros2 param set /{target_node} Kp {kp}) & "
               f"({ROS2_SETUP_CMD} && ros2 param set /{target_node} Ki {ki}) & "
               f"({ROS2_SETUP_CMD} && ros2 param set /{target_node} Kd {kd})")
               
        print(f"\n--- DISPATCHING ASYNC PID GAINS FOR /{target_node} ---")
        subprocess.Popen(cmd, shell=True, executable='/bin/bash', stdout=subprocess.DEVNULL)

    def publish_polynomial(self, poly_type):
        global ROS2_SETUP_CMD
        raw_cmd = f"{ROS2_SETUP_CMD} && ros2 topic pub --once /set_polynomial std_msgs/msg/String \"{{data: '{poly_type}'}}\""
        subprocess.Popen(raw_cmd, shell=True, executable='/bin/bash', stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def publish_waypoints(self, points):
        if not self.is_running(): return False
            
        global ROS2_SETUP_CMD
        
        # Convert Absolute (Webots) back to Relative before publishing via topic
        relative_points = [[pt[0] - ROBOT_START_X, pt[1] - ROBOT_START_Y] for pt in points]
        rotated_points = [[round(pt[1], 3), round(-pt[0], 3)] for pt in relative_points]
        
        points_str = json.dumps(rotated_points)
        raw_cmd = f"{ROS2_SETUP_CMD} && ros2 topic pub --keep-alive 2 /gcs/via_points std_msgs/msg/String \"{{data: '{points_str}'}}\""
        subprocess.Popen(raw_cmd, shell=True, executable='/bin/bash', stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
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

class DiagramViewer(QDialog):
    """Popup window for high-resolution diagram inspection."""
    def __init__(self, image_path, parent=None):
        super().__init__(parent)
        self.setWindowTitle("System Diagram - Full Resolution")
        self.resize(1000, 800)
        
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        self.label = QLabel()
        pixmap = QPixmap(image_path)
        self.label.setPixmap(pixmap)
        
        # Explicitly lock the label to the exact pixel dimensions of the image.
        # This absolutely forces the QScrollArea to map the correct panning bounds.
        self.label.setFixedSize(pixmap.width(), pixmap.height())
        
        # Scroll area allows panning around the massive unscaled image
        scroll = QScrollArea()
        scroll.setStyleSheet("background-color: #ECF0F1;")
        scroll.setWidgetResizable(False) # Strict enforcement of the FixedSize above
        scroll.setWidget(self.label)
        layout.addWidget(scroll)


class InteractiveMapDialog(QDialog):
    def __init__(self, existing_points, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Map Waypoint Selector")
        self.resize(650, 650)
        self.via_points = existing_points.copy()
        
        layout = QVBoxLayout(self)
        self.figure = Figure(figsize=(6, 6), dpi=100)
        self.canvas = FigureCanvas(self.figure)
        layout.addWidget(self.canvas)
        
        self.coord_label = QLabel(f"Absolute Points: {self.via_points}" if self.via_points else "Click on the grid to add Webots absolute coordinates.")
        self.coord_label.setAlignment(Qt.AlignCenter)
        self.coord_label.setStyleSheet("font-weight: bold; font-size: 14px; padding: 10px;")
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
        
        # Set bounds to encompass the new 10x10 area with a small buffer for visibility
        self.ax.set_xlim(-5.5, 5.5) 
        self.ax.set_ylim(-5.5, 5.5)
        self.ax.set_xlabel("Webots X (meters)")
        self.ax.set_ylabel("Webots Y (meters)")
        self.ax.grid(True, linestyle='--', alpha=0.5)
        self.ax.set_aspect('equal')
        
        # 1. Draw the exact center of the Webots world (0,0)
        self.ax.plot(0, 0, 'k+', markersize=15, markeredgewidth=2, label="Arena Center (0,0)")
        
        # 2. Draw the solid physical walls of the 10x10 arena (from -5.0 to 5.0)
        walls = Rectangle((-5.0, -5.0), 10.0, 10.0, fill=False, edgecolor='black', linewidth=3)
        self.ax.add_patch(walls)

        # Draw the robot physically where it starts in Webots
        self.ax.plot(ROBOT_START_X, ROBOT_START_Y, 'gX', markersize=12, label=f"Robot Start ({ROBOT_START_X}, {ROBOT_START_Y})")
        self.ax.legend(loc="upper right", fontsize='small')
        
        # Draw existing points
        for pt in self.via_points:
            self.ax.plot(pt[0], pt[1], 'ro', markersize=8)
        if len(self.via_points) > 1:
            pts = np.array(self.via_points)
            self.ax.plot(pts[:,0], pts[:,1], 'b-', alpha=0.5)
            
        self.canvas.draw()

    def on_click(self, event):
        if event.inaxes != self.ax: return
        x, y = round(event.xdata, 2), round(event.ydata, 2)
        
        # Optional: Prevent clicks completely outside the physical 10x10 wall bounds
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


class AnalyticsWindow(QWidget):
    def __init__(self, scenario_name):
        super().__init__()
        self.scenario_name = scenario_name
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
        
        self.extract_btn = QPushButton(f"📂 Load {self.scenario_name} Experiment Data")
        self.extract_btn.setStyleSheet("background-color: #2980b9; color: white; font-weight: bold; padding: 15px; font-size: 14px;")
        self.extract_btn.clicked.connect(self.load_and_plot_file)
        self.page_layout.addWidget(self.extract_btn)

        self.stats_label = QLabel("Experiment Log: Pending Selection")
        self.stats_label.setFont(QFont("Courier", 11))
        self.stats_label.setAlignment(Qt.AlignCenter)
        self.stats_label.setStyleSheet("background-color: #ecf0f1; padding: 10px; border: 1px solid #bdc3c7;")
        self.page_layout.addWidget(self.stats_label)

        self.page_layout.addWidget(QLabel("📊 ENGINEERING PERFORMANCE INDICES:"))
        self.metrics_table = QTableWidget(1, 2) 
        self.metrics_table.setHorizontalHeaderLabels(["Settling (s)", "RMSE (m)"])
        self.metrics_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.metrics_table.setFixedHeight(85)
        self.metrics_table.setStyleSheet("background-color: #f8f9fa; color: black; font-weight: bold;")
        self.page_layout.addWidget(self.metrics_table)

        self.page_layout.addWidget(QLabel("📈 EXPERIMENT RESPONSE PLOTS:"))
        self.figure = Figure(figsize=(10, 10), dpi=100)
        self.canvas = FigureCanvas(self.figure)
        self.toolbar = NavigationToolbar(self.canvas, self)
        self.canvas.setMinimumHeight(800) 
        
        self.page_layout.addWidget(self.toolbar)
        self.page_layout.addWidget(self.canvas)
        self.scroll.setWidget(self.content_container)
        self.outer_layout.addWidget(self.scroll)

    def load_and_plot_file(self):
        csv_filename, _ = QFileDialog.getOpenFileName(self, f"Open {self.scenario_name} Log", DEFAULT_LOG_DIR, "CSV Files (*.csv)")
        if not csv_filename: return

        try:
            df = pd.read_csv(csv_filename)
            df.columns = df.columns.str.strip() # Fixes hidden spaces in CSV headers
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
        self.figure.clear()
        
        required = ['Time', 'Desired_Distance', 'Measured_Distance', 'Angular_Vel', 'X', 'Y']
        if not all(col in df.columns for col in required):
            QMessageBox.critical(self, "Data Error", f"CSV missing columns for new plotter.\nRequires: {required}")
            return

        self.stats_label.setText(f"Wall Following Analysis | File: {os.path.basename(path)}")

        times = df['Time'].values
        measured = df['Measured_Distance'].values
        setpoint = df['Desired_Distance'].iloc[0]

        # --- CALCULATE METRICS ---
        initial_val = measured[0]
        diff = abs(initial_val - setpoint)
        
        try:
            # Find time indices for 10% and 90% crossing
            t_10 = times[np.where(abs(measured - initial_val) >= 0.10 * diff)[0][0]]
            t_90 = times[np.where(abs(measured - initial_val) >= 0.90 * diff)[0][0]]
            rise_time = t_90 - t_10
        except IndexError:
            t_10, t_90, rise_time = None, None, 0.0

        rmse = np.sqrt(np.mean((measured - setpoint)**2))
        self.update_table([f"Rise (10-90): {rise_time:.2f}s", f"RMSE: {rmse:.4f}m"])

        # Setup Subplots (1 Top Wide, 2 Bottom)
        ax1 = self.figure.add_subplot(211)
        ax2 = self.figure.add_subplot(223)
        ax3 = self.figure.add_subplot(224)

        # ---------------------------------------------------------
        # FIGURE 1: Wall Distance Response (ax1)
        # ---------------------------------------------------------
        tolerance = 0.05 * setpoint
        ax1.fill_between(times, setpoint - tolerance, setpoint + tolerance, 
                         color='gray', alpha=0.2, label='5% Tolerance Band')
        
        ax1.plot(df['Time'], measured, color='tab:blue', linewidth=2, label='Measured Distance')
        ax1.axhline(setpoint, color='tab:red', linestyle='--', linewidth=2, label='Setpoint')

        if t_10 and t_90:
            ax1.axvline(t_10, color='green', linestyle=':', label='Rise Time Start (10%)')
            ax1.axvline(t_90, color='green', linestyle=':', label='Rise Time End (90%)')

        ax1.set_title('Figure 1 - Wall Distance Response')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Distance to wall (m)')
        ax1.legend(loc='upper right', fontsize=9)
        ax1.grid(True, alpha=0.5)

        # ---------------------------------------------------------
        # FIGURE 2: Angular Velocity (ax2)
        # ---------------------------------------------------------
        ax2.plot(df['Time'], df['Angular_Vel'], color='tab:orange', linewidth=2, label='Angular Velocity')
        ax2.axhline(0, color='black', linestyle='--', linewidth=1.5)
        ax2.set_title('Figure 2 - Angular Velocity vs Time')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Angular Velocity (rad/s)')
        ax2.legend(loc='upper right')
        ax2.grid(True, alpha=0.5)

        # ---------------------------------------------------------
        # FIGURE 5: Robot Trajectory (ax3)
        # ---------------------------------------------------------
        ax3.plot(df['X'], df['Y'], color='blue', linewidth=2, label='Robot Trajectory')
        ax3.scatter(df['X'].iloc[0], df['Y'].iloc[0], color='green', marker='o', s=100, label='Start', zorder=5)
        ax3.scatter(df['X'].iloc[-1], df['Y'].iloc[-1], color='red', marker='X', s=100, label='End', zorder=5)
        ax3.set_title('Figure 5 - Robot Trajectory')
        ax3.set_xlabel('X Position (m)')
        ax3.set_ylabel('Y Position (m)')
        ax3.axis('equal')
        ax3.legend(loc='upper right')
        ax3.grid(True, alpha=0.5)

    def plot_trajectory_planning(self, df, path):
        self.figure.clear()
        self.stats_label.setText(f"📐 Pure Pursuit Analysis | File: {os.path.basename(path)}")

        if df.empty:
            QMessageBox.critical(self, "Empty File", "The CSV has headers but no data.")
            return

        required = ['Time', 'Ref_X', 'Ref_Y', 'Robot_X', 'Robot_Y', 'Cross_Track_Error', 'Heading_Error']
        if not all(col in df.columns for col in required):
            QMessageBox.critical(self, "Data Error", f"CSV missing columns for new plotter.\nRequires: {required}")
            return

        time = df['Time'].values
        cte = df['Cross_Track_Error'].values

        rmse_cte = np.sqrt(np.mean(cte**2))
        max_cte = np.max(np.abs(cte))
        self.update_table([f"Max: {max_cte:.4f}m", f"RMSE: {rmse_cte:.4f}m"])

        # Setup Subplots (1 Top Wide, 2 Bottom)
        ax1 = self.figure.add_subplot(211)
        ax2 = self.figure.add_subplot(223)
        ax3 = self.figure.add_subplot(224)

        # ---------------------------------------------------------
        # FIGURE 1: Reference Path vs Actual Trajectory (ax1)
        # ---------------------------------------------------------
        ax1.plot(df['Ref_X'], df['Ref_Y'], color='black', linestyle='--', linewidth=2, label='Reference Path')
        ax1.plot(df['Robot_X'], df['Robot_Y'], color='tab:blue', linewidth=2, label='Actual Trajectory')
        
        ax1.scatter(df['Ref_X'].iloc[0], df['Ref_Y'].iloc[0], color='green', marker='o', s=120, label='Start Point', zorder=5)
        ax1.scatter(df['Ref_X'].iloc[-1], df['Ref_Y'].iloc[-1], color='red', marker='X', s=120, label='Goal Point', zorder=5)
        
        ax1.set_title('Figure 1 - Reference Path vs Actual Robot Trajectory')
        ax1.set_xlabel('X Position (m)')
        ax1.set_ylabel('Y Position (m)')
        ax1.axis('equal') 
        ax1.grid(True, alpha=0.5)
        ax1.legend()

        # ---------------------------------------------------------
        # FIGURE 2: Cross-Track Error vs Time (ax2)
        # ---------------------------------------------------------
        ax2.plot(time, cte, color='tab:red', linewidth=2, label='Cross-Track Error')
        ax2.axhline(0, color='black', linestyle='-', linewidth=1)
        
        ax2.set_title('Figure 2 - Cross-Track Error vs Time')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Cross-track error (m)')
        ax2.grid(True, alpha=0.5)
        ax2.legend()

        # ---------------------------------------------------------
        # FIGURE 3: Heading Error vs Time (ax3)
        # ---------------------------------------------------------
        ax3.plot(time, df['Heading_Error'], color='tab:purple', linewidth=2, label='Heading Error')
        ax3.axhline(0, color='black', linestyle='-', linewidth=1)
        
        ax3.set_title('Figure 3 - Heading Error vs Time')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Heading error (degrees)')
        ax3.grid(True, alpha=0.5)
        ax3.legend()
    
    def plot_obstacle_avoidance(self, df, path):
        self.figure.clear()
        time, clearance, ang_vel = df['Time'].values, df['Clearance'].values, df['Angular_Vel'].values
        min_clear = np.min(clearance)
        self.stats_label.setText(f"🚧 OBSTACLE AVOIDANCE 🚧 | File: {os.path.basename(path)}")
        self.update_table([f"{min_clear:.4f}", f"{time[-1]:.1f}"])

        ax1 = self.figure.add_subplot(211)
        ax2 = self.figure.add_subplot(212)

        ax1.plot(time, clearance, color='#e67e22', linewidth=2, label='Clearance')
        ax1.axhline(0.2, color='red', linestyle='--', label='Safety Limit')
        ax1.set(title="Proximity Metrics", ylabel="Dist (m)")
        ax1.grid(True, linestyle=':', alpha=0.7)
        
        ax2.plot(time, ang_vel, color='#9b59b6', label='Yaw Rate')
        ax2.set(title="Maneuver Intensity", xlabel="Time (s)", ylabel="Rad/s")
        ax2.grid(True, linestyle=':', alpha=0.7)

    


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
                "diagram_path": "diagram_teleop.png"
            },
            "Trajectory Planning": {
                "worlds": ["obstacle_course.wbt"], 
                "node": "niku_controller_viapoints", 
                "controller": "Pure Pursuit Spline",
                "has_analytics": True,
                "objective": "Understand how a robot generates a smooth trajectory through user-defined waypoints and follows it using the Pure Pursuit controller.",
                "how_it_works": "• Select waypoints.\n• A Piecewise Quintic Polynomial trajectory is generated.\n• Pure Pursuit selects a look-ahead point.\n• The controller computes the steering command.\n• The robot follows the planned trajectory.",
                "diagram_path": "diagram_trajectory.png"
            },
            "Wall Following": {
                "worlds": ["wall_following_1.wbt", "wall_following_2.wbt"], 
                "node": "wall_follower", 
                "controller": "PID Control",
                "has_analytics": True,
                "objective": "Investigate how feedback control enables a robot to maintain a desired distance from a wall by adjusting the PID controller gains.",
                "how_it_works": "• The laser scanner measures the wall distance.\n• The measured distance is compared with the desired distance.\n• The PID controller computes a steering correction.\n• The steering command is sent to the robot.\n• The process repeats continuously.",
                "diagram_path": "diagram_wall_following.jpeg" 
            },
            # "Obstacle Avoidance": {
            #     "worlds": ["boxes_dense.wbt", "obstacle_course.wbt", "garden.wbt", "wall_following.wbt"], 
            #     "node": "obstacle_avoidance", 
            #     "controller": "Reactive Logic",
            #     "has_analytics": True,
            #     "objective": "Evaluate real-time LiDAR-based obstacle detection and reactive maneuver intensity to prevent collisions.",
            #     "how_it_works": "• LiDAR scans the immediate environment.\n• Distance to nearest obstacle is calculated.\n• If distance < safety threshold, compute avoidance vector.\n• Publish angular velocity override command.\n• Robot steers away from collision.",
            #     "diagram_path": "diagram_obstacle_avoidance.png"
            # }
        }
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("Integrated Mobile Robot Experimentation Platform")
        self.resize(750, 950) 
        
        self.setStyleSheet("""
            QWidget { background-color: #ECF0F1; } 
            QLabel { font-weight: bold; color: #2C3E50; }
            QGroupBox { font-weight: bold; border: 2px solid #BDC3C7; border-radius: 5px; margin-top: 1ex; }
            QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; padding: 0 3px; }
            QComboBox, QLineEdit, QDoubleSpinBox { background-color: white; color: black; border: 1px solid #BDC3C7; border-radius: 3px; padding: 2px;}
            QTextBrowser { background-color: #F8F9FA; font-style: italic; border: 1px solid #BDC3C7; }
        """)

        self.scroll_main = QScrollArea()
        self.scroll_main.setWidgetResizable(True)
        self.main_container = QWidget()
        main_layout = QVBoxLayout(self.main_container)

        self.logo_label = QLabel()
        self.logo_label.setAlignment(Qt.AlignCenter)
        if os.path.exists("afit_logo.png"):
            pix = QPixmap("afit_logo.png")
            self.logo_label.setPixmap(pix.scaledToHeight(120, Qt.SmoothTransformation))
        else:
            self.logo_label.setText("[afit_logo.png missing]")
        main_layout.addWidget(self.logo_label)

        group_lbl = QLabel("MCT GROUP B")
        group_lbl.setAlignment(Qt.AlignCenter)
        group_lbl.setStyleSheet("font-size: 18px; font-weight: bold; color: #2C3E50; margin-bottom: 10px;")
        main_layout.addWidget(group_lbl)
        
        title_lbl = QLabel("INTEGRATED MOBILE ROBOT EXPERIMENTATION PLATFORM")
        title_lbl.setAlignment(Qt.AlignCenter)
        title_lbl.setStyleSheet("font-size: 16px; background-color: #34495E; color: white; padding: 10px; border-radius: 5px;")
        main_layout.addWidget(title_lbl)

        main_layout.addWidget(self.create_summary_panel())
        main_layout.addWidget(self.create_configuration_panel())
        main_layout.addWidget(self.create_execution_panel())
        main_layout.addWidget(self.create_evaluation_panel())
        
        # --- NEW: Terminal Output Box ---
        term_label = QLabel("Raw ROS 2 Terminal Execution:")
        term_label.setStyleSheet("margin-top: 10px;")
        main_layout.addWidget(term_label)
        
        self.terminal_display = QTextEdit()
        self.terminal_display.setReadOnly(True)
        self.terminal_display.setPlaceholderText("Awaiting ROS 2 subsystem execution...")
        self.terminal_display.setMaximumHeight(100)
        self.terminal_display.setStyleSheet("""
            background-color: #1e1e1e; 
            color: #4af626; 
            font-family: 'Courier New', Courier, monospace;
            font-size: 12px;
            border: 2px solid #2C3E50;
        """)
        main_layout.addWidget(self.terminal_display)
        # ---------------------------------
        
        self.scroll_main.setWidget(self.main_container)
        
        central_layout = QVBoxLayout(self)
        central_layout.setContentsMargins(0, 0, 0, 0)
        central_layout.addWidget(self.scroll_main)

        self.sync_dropdowns()

    def log_terminal(self, command):
        """Appends raw commands to the GUI terminal and auto-scrolls."""
        self.terminal_display.append(f"$ {command}")
        scrollbar = self.terminal_display.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def create_summary_panel(self):
        panel = QGroupBox("CURRENT EXPERIMENT SUMMARY")
        layout = QVBoxLayout(panel)
        
        self.sum_exp_lbl = QLabel("Experiment : ")
        self.sum_world_lbl = QLabel("World      : ")
        self.sum_ctrl_lbl = QLabel("Controller : ")
        self.sum_status_lbl = QLabel("Status     : ● Ready")
        self.sum_status_lbl.setStyleSheet("color: #27AE60;") 
        
        for lbl in [self.sum_exp_lbl, self.sum_world_lbl, self.sum_ctrl_lbl, self.sum_status_lbl]:
            lbl.setFont(QFont("Courier", 10))
            layout.addWidget(lbl)
            
        return panel

    def create_configuration_panel(self):
        panel = QGroupBox("1. EXPERIMENT CONFIGURATION")
        layout = QVBoxLayout(panel)
        
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
        self.diagram_label.setStyleSheet("background-color: #FFFFFF; border: 1px dashed #BDC3C7;")
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
        self.empty_param_panel.setStyleSheet("color: #7F8C8D; font-style: italic;")
        
        layout.addWidget(self.pid_panel)
        layout.addWidget(self.traj_panel)
        layout.addWidget(self.empty_param_panel)

        self.scenario_combo.currentTextChanged.connect(self.sync_dropdowns)
        self.world_combo.currentTextChanged.connect(self.update_summary)
        
        return panel

    def create_execution_panel(self):
        panel = QGroupBox("2. EXPERIMENT EXECUTION")
        layout = QHBoxLayout(panel)
        
        self.launch_btn = QPushButton("▶ START EXPERIMENT")
        self.launch_btn.setStyleSheet("background-color: #27AE60; color: white; font-weight: bold; height: 35px;")
        
        self.kill_btn = QPushButton("■ STOP EXPERIMENT")
        self.kill_btn.setStyleSheet("background-color: #C0392B; color: white; font-weight: bold; height: 35px;")
        
        layout.addWidget(self.launch_btn)
        layout.addWidget(self.kill_btn)
        
        self.launch_btn.clicked.connect(self.handle_launch)
        self.kill_btn.clicked.connect(self.handle_kill)
        
        return panel

    def create_evaluation_panel(self):
        panel = QGroupBox("3. PERFORMANCE EVALUATION")
        layout = QVBoxLayout(panel)
        
        desc = QLabel("(Plots and engineering performance metrics are computed in the dashboard)")
        desc.setStyleSheet("color: #7F8C8D;")
        layout.addWidget(desc)
        
        self.open_dashboard_btn = QPushButton("📊 OPEN ANALYSIS DASHBOARD")
        self.open_dashboard_btn.setStyleSheet("background-color: #8E44AD; color: white; font-weight: bold; height: 40px;")
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
            
        # --- NEW: Educational Tooltips ---
        self.kp_box.setToolTip("<b>Kp (Proportional):</b> Drives the robot towards the target. Higher values mean a more aggressive steering response.")
        self.ki_box.setToolTip("<b>Ki (Integral):</b> Eliminates steady-state error by accumulating past errors. Use sparingly to avoid oscillation.")
        self.kd_box.setToolTip("<b>Kd (Derivative):</b> Dampens the response. Predicts future error to prevent the robot from overshooting the target.")
        # ---------------------------------
            
        self.update_pid_btn = QPushButton("UPDATE GAINS")
        self.update_pid_btn.setStyleSheet("background-color: #2980B9; color: white; font-weight: bold;")
        self.update_pid_btn.clicked.connect(self.send_pid_update)
        
        layout.addWidget(QLabel("Kp:")); layout.addWidget(self.kp_box)
        layout.addWidget(QLabel("Ki:")); layout.addWidget(self.ki_box)
        layout.addWidget(QLabel("Kd:")); layout.addWidget(self.kd_box)
        layout.addWidget(self.update_pid_btn)
        return panel

    def create_traj_panel(self):
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0,0,0,0)
        
        row1 = QHBoxLayout()
        self.open_map_btn = QPushButton("🗺️ Open Waypoint Selector")
        self.open_map_btn.setStyleSheet("background-color: #E67E22; color: white; font-weight: bold;")
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
            self.open_dashboard_btn.setStyleSheet("background-color: #8E44AD; color: white; font-weight: bold; height: 40px;")
            self.open_dashboard_btn.setText("📊 OPEN ANALYSIS DASHBOARD")
        else:
            self.open_dashboard_btn.setEnabled(False)
            self.open_dashboard_btn.setStyleSheet("background-color: #95A5A6; color: #ECF0F1; font-weight: bold; height: 40px;")
            self.open_dashboard_btn.setText("Analytics Disabled for this Experiment")
            
        self.update_summary()

    def update_summary(self, status_override=None):
        scenario = self.scenario_combo.currentText()
        data = self.logic_matrix[scenario]
        
        self.sum_exp_lbl.setText(f"Experiment : {scenario}")
        self.sum_world_lbl.setText(f"World      : {self.world_combo.currentText()}")
        self.sum_ctrl_lbl.setText(f"Controller : {data['controller']}")
        
        if status_override:
            if "Running" in status_override:
                self.sum_status_lbl.setStyleSheet("color: #E67E22;") 
            elif "Stopped" in status_override or "Completed" in status_override:
                self.sum_status_lbl.setStyleSheet("color: #34495E;") 
            self.sum_status_lbl.setText(f"Status     : {status_override}")

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

        # --- NEW: Log to Terminal ---
        robot_arg = f"robot_name:={self.robot_name_input.text()} " if self.robot_name_input.text().strip() else ""
        wp_arg = f"relative_waypoints:=\"{waypoints_str}\"" if waypoints_str else ""
        self.log_terminal(f"ros2 launch robot_simulation simulation.launch.py world:={self.world_combo.currentText()} scenario:={node} {robot_arg}{wp_arg}")
        # ----------------------------

        self.update_summary("● Running")
        self.ros_manager.launch(self.world_combo.currentText(), node, self.robot_name_input.text(), waypoints_str)

    def handle_kill(self):
        if not self.ros_manager.kill_all():
            QMessageBox.warning(self, 'Error', 'No experiment currently running.')
        else:
            # --- NEW: Log to Terminal ---
            self.log_terminal("pkill -f 'ros2 topic pub'")
            self.log_terminal("ros2 daemon stop")
            # ----------------------------
            self.update_summary("● Stopped")

    def send_pid_update(self):
        if not self.ros_manager.is_running():
            QMessageBox.warning(self, "Error", "Start the experiment first!")
            return
        
        node = self.logic_matrix[self.scenario_combo.currentText()]["node"]
        robot = self.robot_name_input.text()
        
        target_node = f"{robot}/{node}" if robot.strip() else node
        target_node = target_node.strip('/')

        # --- NEW: Log to Terminal ---
        self.log_terminal(f"ros2 param set /{target_node} Kp {self.kp_box.value()}")
        self.log_terminal(f"ros2 param set /{target_node} Ki {self.ki_box.value()}")
        self.log_terminal(f"ros2 param set /{target_node} Kd {self.kd_box.value()}")
        # ----------------------------

        self.ros_manager.set_pid_params(node, robot, self.kp_box.value(), self.ki_box.value(), self.kd_box.value())

    def open_waypoint_map(self):
        map_dialog = InteractiveMapDialog(self.current_waypoints, self)
        if map_dialog.exec_() == QDialog.Accepted:
            self.current_waypoints = map_dialog.get_points()
            if self.current_waypoints:
                self.waypoints_display.setText(f"Absolute: {str(self.current_waypoints)}")
            else:
                self.waypoints_display.setText("")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion") 
    window = AFIT_GCS()
    window.show()
    sys.exit(app.exec_())