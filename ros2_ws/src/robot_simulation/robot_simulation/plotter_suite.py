#!/usr/bin/env python3
import os
import glob
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# LOG_DIR = os.path.expanduser('~/ros2_ws/simulation_logs')
# GRAPH_DIR = os.path.join(LOG_DIR, 'graphs')
# os.makedirs(GRAPH_DIR, exist_ok=True)

LOG_DIR = os.environ.get('THESIS_LOG_DIR', os.path.expanduser('~/simulation_logs'))
GRAPH_DIR = os.path.join(LOG_DIR, 'graphs')
os.makedirs(GRAPH_DIR, exist_ok=True)

plt.rcParams.update({
    'font.family': 'serif',
    'axes.grid': True,
    'grid.alpha': 0.5,
    'axes.labelsize': 12,
    'axes.titlesize': 14,
    'figure.titlesize': 16,
    'legend.fontsize': 10
})

def get_latest_file(pattern):
    files = glob.glob(os.path.join(LOG_DIR, pattern))
    if not files: return None
    return max(files, key=os.path.getctime)

def plot_hardware_profile():
    hw_file = get_latest_file('hardware_profile_*.csv')
    if not hw_file: return
    
    df = pd.read_csv(hw_file)
    # Cleaning Logic: Drop NaNs to prevent gaps in the plot
    df = df.dropna(subset=['RTF'])
    
    fig, (ax1, ax3) = plt.subplots(2, 1, figsize=(10, 8), sharex=True, gridspec_kw={'height_ratios': [2, 1]})
    
    color_cpu = 'tab:red'
    ax1.set_ylabel('CPU Usage (%)', color=color_cpu)
    ax1.plot(df['Time_Elapsed_s'], df['CPU_Usage_Percent'], color=color_cpu, linewidth=1.5, label='CPU')
    ax1.tick_params(axis='y', labelcolor=color_cpu)
    ax1.set_ylim([0, 100])
    
    ax2 = ax1.twinx()
    color_ram = 'tab:blue'
    ax2.set_ylabel('RAM Usage (MB)', color=color_ram)
    ax2.plot(df['Time_Elapsed_s'], df['RAM_Usage_MB'], color=color_ram, linewidth=1.5, label='RAM')
    ax2.tick_params(axis='y', labelcolor=color_ram)
    
    ax1.set_title('System Resource Utilization & Real-Time Factor')
    
    color_rtf = 'tab:green'
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Real-Time Factor', color=color_rtf)
    ax3.plot(df['Time_Elapsed_s'], df['RTF'], color=color_rtf, linewidth=1.5)
    ax3.tick_params(axis='y', labelcolor=color_rtf)
    
    fig.tight_layout()
    plt.savefig(os.path.join(GRAPH_DIR, 'hardware_and_rtf_profile.png'), dpi=300)
    plt.close()

def plot_kinematic_square():
    sq_file = get_latest_file('thesis_open_loop_*.csv')
    if not sq_file: return
    df = pd.read_csv(sq_file)
    plt.figure(figsize=(7, 7))
    plt.plot(df['X'], df['Y'], label='Actual Trajectory', color='k', linewidth=2)
    plt.scatter(0, 0, color='g', label='Start (0,0)', zorder=5)
    plt.scatter(df['X'].iloc[-1], df['Y'].iloc[-1], color='r', label='End', zorder=5)
    plt.title('Open-Loop Kinematic Square Drift')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.legend()
    plt.axis('equal')
    plt.savefig(os.path.join(GRAPH_DIR, 'kinematic_square.png'), dpi=300)
    plt.close()

def plot_pure_pursuit():
    # Sort files chronologically to map them to Run 1, Run 2, etc.
    files = glob.glob(os.path.join(LOG_DIR, 'run_*_tracking.csv'))
    files.sort(key=os.path.getmtime)
    
    if not files: 
        return

    print(f"-> Generating Pure Pursuit thesis plots for {len(files)} runs...")

    for idx, f in enumerate(files, start=1):
        df = pd.read_csv(f)
        run_name = f"run{idx}"
        display_name = f"Run {idx}"
        
        if 'Robot_X' not in df.columns:
            continue

        # ---------------------------------------------------------
        # FIGURE 1: Reference Path vs Actual Robot Trajectory
        # ---------------------------------------------------------
        plt.figure(figsize=(8, 8))
        plt.plot(df['Ref_X'], df['Ref_Y'], color='black', linestyle='--', linewidth=2, label='Reference Path')
        plt.plot(df['Robot_X'], df['Robot_Y'], color='tab:blue', linewidth=2, label='Actual Trajectory')
        
        plt.scatter(df['Ref_X'].iloc[0], df['Ref_Y'].iloc[0], color='green', marker='o', s=120, label='Start Point', zorder=5)
        plt.scatter(df['Ref_X'].iloc[-1], df['Ref_Y'].iloc[-1], color='red', marker='X', s=120, label='Goal Point', zorder=5)
        
        plt.title(f'Figure 1 - Reference Path vs Actual Robot Trajectory ({display_name})')
        plt.xlabel('X Position (m)')
        plt.ylabel('Y Position (m)')
        plt.axis('equal') 
        plt.grid(True, alpha=0.5)
        plt.legend()
        plt.tight_layout()
        plt.savefig(os.path.join(GRAPH_DIR, f'trajectory_{run_name}.png'), dpi=300)
        plt.close()

        # ---------------------------------------------------------
        # FIGURE 2: Cross-Track Error vs Time
        # ---------------------------------------------------------
        plt.figure(figsize=(10, 5))
        plt.plot(df['Time'], df['Cross_Track_Error'], color='tab:red', linewidth=2, label='Cross-Track Error')
        plt.axhline(0, color='black', linestyle='-', linewidth=1)
        
        plt.title(f'Figure 2 - Cross-Track Error vs Time ({display_name})')
        plt.xlabel('Time (s)')
        plt.ylabel('Cross-track error (m)')
        plt.grid(True, alpha=0.5)
        plt.legend()
        plt.tight_layout()
        plt.savefig(os.path.join(GRAPH_DIR, f'cross_track_error_{run_name}.png'), dpi=300)
        plt.close()

        # ---------------------------------------------------------
        # FIGURE 3: Heading Error vs Time
        # ---------------------------------------------------------
        plt.figure(figsize=(10, 5))
        plt.plot(df['Time'], df['Heading_Error'], color='tab:purple', linewidth=2, label='Heading Error')
        plt.axhline(0, color='black', linestyle='-', linewidth=1)
        
        plt.title(f'Figure 3 - Heading Error vs Time ({display_name})')
        plt.xlabel('Time (s)')
        plt.ylabel('Heading error (degrees)')
        plt.grid(True, alpha=0.5)
        plt.legend()
        plt.tight_layout()
        plt.savefig(os.path.join(GRAPH_DIR, f'heading_error_{run_name}.png'), dpi=300)
        plt.close()

    print("-> Pure pursuit publication plots (Figs 1, 2, 3) generated successfully.")

# --- MODIFIED SECTION ---
def plot_wall_follower():
    # Find all wall follower files and sort chronologically
    files = glob.glob(os.path.join(LOG_DIR, 'wall_follow_run_*.csv'))
    files.sort(key=os.path.getmtime)
    
    if not files: 
        print("-> No wall follower logs found for plotting.")
        return

    print(f"-> Generating Wall Follower plots for {len(files)} runs...")

    for idx, f in enumerate(files, start=1):
        df = pd.read_csv(f)
        run_name = f"run{idx}"
        display_name = f"Run {idx}"
        
        if 'Desired_Distance' not in df.columns:
            continue

        setpoint = df['Desired_Distance'].iloc[0]
        times = df['Time'].values
        measured = df['Measured_Distance'].values
        
        # --- CALCULATE METRICS FOR ANNOTATION ---
        # Rise Time (10% to 90%)
        initial_val = measured[0]
        diff = abs(initial_val - setpoint)
        # Find time indices for 10% and 90% crossing
        t_10 = times[np.where(abs(measured - initial_val) >= 0.10 * diff)[0][0]]
        t_90 = times[np.where(abs(measured - initial_val) >= 0.90 * diff)[0][0]]

        # ---------------------------------------------------------
        # FIGURE 1: Wall Distance Response with Annotations
        # ---------------------------------------------------------
        plt.figure(figsize=(10, 5))
        
        # 1. Tolerance Band (±5%)
        tolerance = 0.05 * setpoint
        plt.fill_between(times, setpoint - tolerance, setpoint + tolerance, 
                         color='gray', alpha=0.2, label='5% Tolerance Band')
        
        # 2. Main Response Lines
        plt.plot(df['Time'], measured, color='tab:blue', linewidth=2, label='Measured Distance')
        plt.axhline(setpoint, color='tab:red', linestyle='--', linewidth=2, label='Setpoint')

        # 3. Annotations
        plt.axvline(t_10, color='green', linestyle=':', label='Rise Time Start (10%)')
        plt.axvline(t_90, color='green', linestyle=':', label='Rise Time End (90%)')

        plt.title(f'Figure 1 - Wall Distance Response ({display_name})')
        plt.xlabel('Time (s)')
        plt.ylabel('Distance to wall (m)')
        
        # LEGEND MOVED TO TOP RIGHT
        plt.legend(loc='upper right', fontsize=9)
        
        plt.grid(True, alpha=0.5)
        plt.tight_layout()
        plt.savefig(os.path.join(GRAPH_DIR, f'fig1_wall_distance_response_{run_name}.png'), dpi=300)
        plt.close()
        
        # ---------------------------------------------------------
        # FIGURE 2: Angular Velocity
        # ---------------------------------------------------------
        plt.figure(figsize=(10, 5))
        plt.plot(df['Time'], df['Angular_Vel'], color='tab:orange', linewidth=2, label='Angular Velocity')
        plt.axhline(0, color='black', linestyle='--', linewidth=1.5)
        plt.title(f'Figure 2 - Angular Velocity vs Time ({display_name})')
        plt.xlabel('Time (s)')
        plt.ylabel('Angular Velocity (rad/s)')
        plt.legend(loc='upper right')
        plt.grid(True, alpha=0.5)
        plt.tight_layout()
        plt.savefig(os.path.join(GRAPH_DIR, f'fig2_angular_velocity_{run_name}.png'), dpi=300)
        plt.close()

        # ---------------------------------------------------------
        # FIGURE 5: Robot Trajectory
        # ---------------------------------------------------------
        plt.figure(figsize=(8, 8))
        plt.plot(df['X'], df['Y'], color='blue', linewidth=2, label='Robot Trajectory')
        plt.scatter(df['X'].iloc[0], df['Y'].iloc[0], color='green', marker='o', s=100, label='Start', zorder=5)
        plt.scatter(df['X'].iloc[-1], df['Y'].iloc[-1], color='red', marker='X', s=100, label='End', zorder=5)
        plt.title(f'Figure 5 - Robot Trajectory ({display_name})')
        plt.xlabel('X Position (m)')
        plt.ylabel('Y Position (m)')
        plt.axis('equal')
        plt.legend(loc='upper right')
        plt.grid(True, alpha=0.5)
        plt.tight_layout()
        plt.savefig(os.path.join(GRAPH_DIR, f'fig5_robot_trajectory_{run_name}.png'), dpi=300)
        plt.close()
        
    print("-> Wall follower plots (Figs 1, 2, 5) saved.")
# --- END MODIFIED SECTION ---

def plot_ekf_validation():
    ekf_file = get_latest_file('_*.csv')
    if not ekf_file: return
    df = pd.read_csv(ekf_file)
    plt.figure(figsize=(10, 8))
    plt.plot(df['Ground_Truth_X'], df['Ground_Truth_Y'], color='k', linestyle='-', linewidth=2, label='Ground Truth')
    plt.plot(df['Raw_Odom_X'], df['Raw_Odom_Y'], color='tab:red', linestyle='--', linewidth=1.5, label='Raw Odometry')
    plt.plot(df['EKF_X'], df['EKF_Y'], color='tab:green', linestyle='-.', linewidth=1.5, label='EKF Filtered')
    plt.title('Sensor Fusion Localization Parity')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.legend()
    plt.axis('equal')
    plt.savefig(os.path.join(GRAPH_DIR, 'ekf_parity.png'), dpi=300)
    plt.close()

def main():
    print(f"\n[PLOTTER SUITE] Generating High-Res Graphics to: {GRAPH_DIR}")
    plot_hardware_profile()
    plot_kinematic_square()
    plot_pure_pursuit()
    plot_wall_follower()
    plot_ekf_validation()
    print("[PLOTTER SUITE] Graphics generation complete.\n")

if __name__ == '__main__':
    main()