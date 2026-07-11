# #!/usr/bin/env python3
# import os
# import glob
# import pandas as pd
# import numpy as np

# LOG_DIR = os.path.expanduser('~/ros2_ws/simulation_logs')

# def get_latest_file(pattern):
#     files = glob.glob(os.path.join(LOG_DIR, pattern))
#     if not files:
#         return None
#     return max(files, key=os.path.getctime)

# def calculate_rmse(errors):
#     return np.sqrt(np.mean(np.square(errors)))

# def calculate_mae(errors):
#     return np.mean(np.abs(errors))

# def analyze_hardware_and_rtf():
#     print("\n" + "="*50)
#     print(" 4.0 HARDWARE UTILIZATION & RTF (RTF-GATED)")
#     print("="*50)
    
#     hw_file = get_latest_file('hardware_profile_*.csv')
#     if hw_file:
#         df = pd.read_csv(hw_file)
        
#         # --- THE RTF-GATING LOGIC ---
#         # Only keep rows where RTF is a valid, non-zero number
#         # This automatically ignores the startup "preamble" and shutdown period
#         active_df = df[df['RTF'].notnull() & (df['RTF'] > 0.001)]
        
#         if active_df.empty:
#             print("No active simulation window found in log.")
#             return

#         # --- CALCULATION ---
#         peak_cpu = active_df['CPU_Usage_Percent'].max()
#         mean_cpu = active_df['CPU_Usage_Percent'].mean()
#         std_cpu  = active_df['CPU_Usage_Percent'].std()
        
#         mean_rtf = active_df['RTF'].mean()
#         std_rtf  = active_df['RTF'].std()

#         print(f"CPU Allocation:       Mean: {mean_cpu:.2f}% ± {std_cpu:.2f}% (Peak: {peak_cpu:.2f}%)")
#         print(f"Real-Time Factor:     Mean: {mean_rtf:.4f} ± {std_rtf:.4f}")
#     else:
#         print("Hardware profile log not found.")

# def analyze_latency_and_jitter():
#     print("\n" + "="*50)
#     print(" 4.1 SYSTEM VALIDATION (LATENCY & JITTER)")
#     print("="*50)
    
#     latency_file = get_latest_file('latency_log.txt')
#     if latency_file:
#         try:
#             df_lat = pd.read_csv(latency_file, header=None, names=['Node', 'Time', 'Latency'])
#             mean_lat = df_lat['Latency'].mean()
#             std_lat = df_lat['Latency'].std()
#             print(f"Startup Latency (μ ± σ): {mean_lat:.4f} s ± {std_lat:.4f} s")
#         except Exception as e:
#             print(f"Latency file found but unreadable: {e}")
#     else:
#         print("Latency log not found.")

#     pp_file = get_latest_file('run_*.csv')
#     if pp_file:
#         df_pp = pd.read_csv(pp_file)
#         if 'Jitter_dt' in df_pp.columns:
#             mean_dt = df_pp['Jitter_dt'].mean()
#             std_dt = df_pp['Jitter_dt'].std()
#             target_dt = 0.10
#             deviation = (std_dt / target_dt) * 100
#             print(f"Control Loop Jitter:     {mean_dt:.4f} s ± {std_dt:.4f} s")
#             print(f"Timing Jitter (σ rel):   {deviation:.2f}% relative to 10Hz target")
#         else:
#             print("Jitter data not found in run log.")
#     else:
#          print("Pure Pursuit run log not found for Jitter calculation.")

# def analyze_kinematic_square():
#     print("\n" + "="*50)
#     print(" 4.3 OPEN-LOOP KINEMATIC DRIFT")
#     print("="*50)
#     sq_file = get_latest_file('thesis_open_loop_*.csv')
#     if sq_file:
#         df = pd.read_csv(sq_file)
#         final_x = df['X'].iloc[-1]
#         final_y = df['Y'].iloc[-1]
#         drift = np.sqrt(final_x**2 + final_y**2)
#         print(f"Final Coordinate:        (X: {final_x:.4f}, Y: {final_y:.4f})")
#         print(f"Total Positional Err:    {drift:.4f} m")
#     else:
#         print("Kinematic Square log not found.")

# def analyze_wall_follower():
#     print("\n" + "="*50)
#     print(" 4.4 PID WALL FOLLOWING PERFORMANCE")
#     print("="*50)
#     wf_file = get_latest_file('wall_follow_run_*.csv')
#     if wf_file:
#         df = pd.read_csv(wf_file)
#         df_steady = df[df['Time'] > 5.0]
#         rmse = calculate_rmse(df_steady['Error'])
#         mae = calculate_mae(df_steady['Error'])
#         chatter = df['Control_Effort'].var()
#         print(f"Tracking RMSE:           {rmse:.4f} m")
#         print(f"Tracking MAE:            {mae:.4f} m")
#         print(f"Chattering Variance:     {chatter:.4f} (Control Effort)")
#     else:
#         print("Wall Follower log not found.")

# def analyze_pure_pursuit():
#     print("\n" + "="*50)
#     print(" 4.5 PURE PURSUIT PATH TRACKING")
#     print("="*50)
#     pp_file = get_latest_file('run_*.csv')
#     if pp_file:
#         df = pd.read_csv(pp_file)
#         rmse_cte = calculate_rmse(df['CTE'])
#         max_cte = df['CTE'].abs().max()
#         mean_cte = df['CTE'].mean()
#         print(f"CTE RMSE:                {rmse_cte:.4f} m")
#         print(f"Max Absolute CTE:        {max_cte:.4f} m")
#         print(f"Mean CTE:                {mean_cte:.4f} m")
#     else:
#         print("Pure Pursuit log not found.")

# def analyze_ekf():
#     print("\n" + "="*50)
#     print(" 4.6 EKF STATE ESTIMATION (FUSION PARITY)")
#     print("="*50)
#     ekf_file = get_latest_file('ekf_*.csv')
#     if ekf_file:
#         df = pd.read_csv(ekf_file)
#         df['Raw_Err'] = np.sqrt((df['Raw_Odom_X'] - df['Ground_Truth_X'])**2 + (df['Raw_Odom_Y'] - df['Ground_Truth_Y'])**2)
#         df['EKF_Err'] = np.sqrt((df['EKF_X'] - df['Ground_Truth_X'])**2 + (df['EKF_Y'] - df['Ground_Truth_Y'])**2)
#         raw_rmse = calculate_rmse(df['Raw_Err'])
#         ekf_rmse = calculate_rmse(df['EKF_Err'])
#         improvement = ((raw_rmse - ekf_rmse) / raw_rmse) * 100 if raw_rmse > 0 else 0
#         print(f"Raw Odometry RMSE:       {raw_rmse:.4f} m")
#         print(f"Filtered EKF RMSE:       {ekf_rmse:.4f} m")
#         print(f"Estimation Imprv:        {improvement:.2f}%")
#     else:
#         print("EKF Validation log not found.")

# def main():
#     print("\n[METRIC CALCULATOR] Scanning logs in: " + LOG_DIR)
#     analyze_hardware_and_rtf()
#     analyze_latency_and_jitter()
#     analyze_kinematic_square()
#     analyze_wall_follower()
#     analyze_pure_pursuit()
#     analyze_ekf()
#     print("\n" + "="*50 + "\n")

# if __name__ == '__main__':
#     main()






































#!/usr/bin/env python3
import os
import sys
import glob
import pandas as pd
import numpy as np

# DEFAULT DIR: Fallback if no argument is provided
DEFAULT_LOG_DIR = os.path.expanduser('~/ros2_ws/simulation_logs')

# DYNAMIC DIR: Allows you to pass the specific thesis_results folder via terminal
LOG_DIR = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_LOG_DIR

def get_all_files(pattern):
    files = glob.glob(os.path.join(LOG_DIR, pattern))
    return files

def calculate_rmse(errors):
    return np.sqrt(np.mean(np.square(errors)))

def analyze_hardware_and_rtf():
    print("\n" + "="*50)
    print(" 4.0 HARDWARE UTILIZATION & RTF (BATCH AGGREGATE)")
    print("="*50)
    files = get_all_files('hardware_profile_*.csv')
    if not files: 
        print("-> No hardware profiles found in this batch.")
        return
    
    means_cpu, means_rtf = [], []
    for f in files:
        df = pd.read_csv(f)
        active_df = df[df['RTF'].notnull() & (df['RTF'] > 0.001)]
        if not active_df.empty:
            means_cpu.append(active_df['CPU_Usage_Percent'].mean())
            means_rtf.append(active_df['RTF'].mean())
            
    print(f"Trials Analyzed:      {len(files)}")
    print(f"CPU Allocation:       {np.mean(means_cpu):.2f}% ± {np.std(means_cpu):.2f}%")
    print(f"Real-Time Factor:     {np.mean(means_rtf):.4f} ± {np.std(means_rtf):.4f}")

def analyze_kinematic_square():
    print("\n" + "="*50)
    print(" 4.3 OPEN-LOOP KINEMATIC DRIFT (BATCH AGGREGATE)")
    print("="*50)
    files = get_all_files('thesis_open_loop_*.csv')
    
    # --- ENFORCE N=5 LIMIT ---
    files = sorted(files)[:5] 
    # -------------------------

    if not files: 
        print("-> No kinematic square logs found in this batch.")
        return

    drifts = []
    
    # Header for individual runs
    print(f"{'Run Filename':<35} | {'Drift (m)':<10}")
    print("-" * 50)
    
    for f in files:
        df = pd.read_csv(f)
        final_x, final_y = df['X'].iloc[-1], df['Y'].iloc[-1]
        drift = np.sqrt(final_x**2 + final_y**2)
        drifts.append(drift)
        
        # Display each run
        print(f"{os.path.basename(f):<35} | {drift:.4f}")

    # Summary Statistics
    print("-" * 50)
    print(f"OVERALL AGGREGATE (n={len(files)}):")
    print(f"Mean Positional Drift:   {np.mean(drifts):.4f} m ± {np.std(drifts):.4f} m")
    print(f"Worst-Case Drift (Max):  {np.max(drifts):.4f} m")
    

# --- MODIFIED SECTION ---
def analyze_wall_follower():
    print("\n" + "="*110)
    print(" 4.4 PID WALL FOLLOWING PERFORMANCE (ADVANCED CONTROL METRICS)")
    print("="*110)

    files = get_all_files('wall_follow_run_*.csv')
    if not files:
        print("-> No wall follower logs found in this batch.")
        return

    metrics = {
        'rise_time': [],
        'settling_time': [],
        'overshoot': [],
        'sse': [],
        'rmse': [],
        'mae': [],
        'max_err': [],
        'iae': []
    }

    print(f"{'Run Number (File)':<25} | {'Rise(s)':<8} | {'Settl(s)':<8} | {'OvSht(%)':<8} | {'SSE(m)':<8} | {'RMSE(m)':<8} | {'MAE(m)':<8} | {'MaxE(m)':<8} | {'IAE':<8}")
    print("-" * 110)

    for f in files:

        df = pd.read_csv(f)

        if 'Desired_Distance' not in df.columns:
            print(f"{os.path.basename(f):<25} | -> Missing required columns for advanced metrics.")
            continue

        times = df['Time'].values
        measured = df['Measured_Distance'].values
        error = df['Error'].values
        setpoint = df['Desired_Distance'].iloc[0]
        initial_val = measured[0]

        # -------------------------------------------------------
        # 1. Rise Time (10%-90%)
        # -------------------------------------------------------
        step_amp = abs(initial_val - setpoint)
        sign = 1 if initial_val < setpoint else -1

        try:
            idx10 = np.where(
                sign * (measured - initial_val) >= 0.10 * step_amp
            )[0][0]

            idx90 = np.where(
                sign * (measured - initial_val) >= 0.90 * step_amp
            )[0][0]

            rise_time = times[idx90] - times[idx10]

        except IndexError:
            rise_time = 0.0

        # -------------------------------------------------------
        # 2. Settling Time (Ogata definition)
        # First instant after which response NEVER leaves
        # the ±5% band again.
        # -------------------------------------------------------
        tolerance = 0.05 * setpoint

        settling_time = times[-1]

        for i in range(len(times)):
            remaining = measured[i:]

            if np.all(np.abs(remaining - setpoint) <= tolerance):
                settling_time = times[i]
                break

        # -------------------------------------------------------
        # 3. Maximum Overshoot
        # -------------------------------------------------------
        if step_amp > 0:

            if initial_val > setpoint:
                peak = np.min(measured)
            else:
                peak = np.max(measured)

            crossed = (
                (initial_val > setpoint and peak < setpoint)
                or
                (initial_val < setpoint and peak > setpoint)
            )

            if crossed:
                overshoot = abs(peak - setpoint) / step_amp * 100.0
            else:
                overshoot = 0.0

        else:
            overshoot = 0.0

        # -------------------------------------------------------
        # 4. Steady-State Error
        # Mean error over the final 5 seconds
        # -------------------------------------------------------
        if len(times) > 1:

            final_window_start = times[-1] - 5.0

            steady_df = df[df['Time'] >= final_window_start]

            if not steady_df.empty:
                sse = steady_df['Error'].mean()
            else:
                sse = error[-1]

        else:
            sse = error[-1]

        # -------------------------------------------------------
        # 5. RMSE
        # -------------------------------------------------------
        rmse = calculate_rmse(error)

        # -------------------------------------------------------
        # 6. MAE
        # -------------------------------------------------------
        mae = np.mean(np.abs(error))

        # -------------------------------------------------------
        # 7. Maximum Error
        # -------------------------------------------------------
        max_err = np.max(np.abs(error))

        # -------------------------------------------------------
        # 8. Integral Absolute Error
        # -------------------------------------------------------
        iae = np.trapz(np.abs(error), times)

        metrics['rise_time'].append(rise_time)
        metrics['settling_time'].append(settling_time)
        metrics['overshoot'].append(overshoot)
        metrics['sse'].append(sse)
        metrics['rmse'].append(rmse)
        metrics['mae'].append(mae)
        metrics['max_err'].append(max_err)
        metrics['iae'].append(iae)

        filename_short = os.path.basename(f)[:22]

        print(
            f"{filename_short:<25} | "
            f"{rise_time:<8.2f} | "
            f"{settling_time:<8.2f} | "
            f"{overshoot:<8.2f} | "
            f"{sse:<8.4f} | "
            f"{rmse:<8.4f} | "
            f"{mae:<8.4f} | "
            f"{max_err:<8.4f} | "
            f"{iae:<8.2f}"
        )

    print("-" * 110)

    if metrics['rise_time']:

        print(f"OVERALL AGGREGATE (n={len(metrics['rise_time'])}):")

        print(
            f"{'Mean (μ)':<25} | "
            f"{np.mean(metrics['rise_time']):<8.2f} | "
            f"{np.mean(metrics['settling_time']):<8.2f} | "
            f"{np.mean(metrics['overshoot']):<8.2f} | "
            f"{np.mean(metrics['sse']):<8.4f} | "
            f"{np.mean(metrics['rmse']):<8.4f} | "
            f"{np.mean(metrics['mae']):<8.4f} | "
            f"{np.mean(metrics['max_err']):<8.4f} | "
            f"{np.mean(metrics['iae']):<8.2f}"
        )

        print(
            f"{'Std Dev (σ)':<25} | "
            f"{np.std(metrics['rise_time']):<8.2f} | "
            f"{np.std(metrics['settling_time']):<8.2f} | "
            f"{np.std(metrics['overshoot']):<8.2f} | "
            f"{np.std(metrics['sse']):<8.4f} | "
            f"{np.std(metrics['rmse']):<8.4f} | "
            f"{np.std(metrics['mae']):<8.4f} | "
            f"{np.std(metrics['max_err']):<8.4f} | "
            f"{np.std(metrics['iae']):<8.2f}"
        )

# --- REPLACE analyze_pure_pursuit() IN metric_calculator.py ---
def analyze_pure_pursuit():
    print("\n" + "="*95)
    print(" 4.5 PURE PURSUIT PATH TRACKING EVALUATION")
    print("="*95)
    
    # Target all tracking logs and sort them chronologically by file modification time
    files = glob.glob(os.path.join(LOG_DIR, 'run_*_tracking.csv'))
    files.sort(key=os.path.getmtime)
    
    if not files: 
        print("-> No pure pursuit tracking logs found in this batch.")
        return

    summary_data = []
    print(f"{'Run Number':<18} | {'RMSE(m)':<8} | {'MAE(m)':<8} | {'MaxE(m)':<8} | {'StdDev(m)':<9} | {'Comp(%)':<8}")
    print("-" * 95)

    for idx, f in enumerate(files, start=1):
        df = pd.read_csv(f)
        run_name = f"Run {idx}"
        
        if 'Cross_Track_Error' not in df.columns:
            print(f"-> Skipping {run_name}: Missing 'Cross_Track_Error' column.")
            continue
            
        cte = df['Cross_Track_Error'].values
        
        # Metrics 1-4
        rmse = np.sqrt(np.mean(np.square(cte)))
        mae = np.mean(np.abs(cte))
        max_err = np.max(np.abs(cte))
        std_dev = np.std(cte)
        
        # Metric 5: Completion Rate
        ref_x, ref_y = df['Ref_X'].values, df['Ref_Y'].values
        rob_x, rob_y = df['Robot_X'].values, df['Robot_Y'].values
        
        total_ref_length = np.sum(np.sqrt(np.diff(ref_x)**2 + np.diff(ref_y)**2))
        total_rob_length = np.sum(np.sqrt(np.diff(rob_x)**2 + np.diff(rob_y)**2))
        
        # Distance to final waypoint
        dist_to_goal = np.sqrt((rob_x[-1] - ref_x[-1])**2 + (rob_y[-1] - ref_y[-1])**2)
        if dist_to_goal <= 0.3:
            comp_rate = 100.0
        else:
            comp_rate = min(100.0, (total_rob_length / total_ref_length) * 100)

        summary_data.append([run_name, rmse, mae, max_err, std_dev, comp_rate])
        print(f"{run_name:<18} | {rmse:<8.4f} | {mae:<8.4f} | {max_err:<8.4f} | {std_dev:<9.4f} | {comp_rate:<8.2f}")

    # Generate pure_pursuit_summary.csv
    if summary_data:
        summary_df = pd.DataFrame(summary_data, columns=['Run', 'RMSE (m)', 'MAE (m)', 'Maximum Error (m)', 'Standard Deviation (m)', 'Completion Rate (%)'])
        
        mean_row = ['Mean', summary_df['RMSE (m)'].mean(), summary_df['MAE (m)'].mean(), summary_df['Maximum Error (m)'].mean(), summary_df['Standard Deviation (m)'].mean(), summary_df['Completion Rate (%)'].mean()]
        std_row = ['Standard Deviation', summary_df['RMSE (m)'].std(), summary_df['MAE (m)'].std(), summary_df['Maximum Error (m)'].std(), summary_df['Standard Deviation (m)'].std(), summary_df['Completion Rate (%)'].std()]
        
        summary_df.loc[len(summary_df)] = mean_row
        summary_df.loc[len(summary_df)] = std_row
        
        summary_file = os.path.join(LOG_DIR, 'pure_pursuit_summary.csv')
        summary_df.to_csv(summary_file, index=False)
        print("-" * 95)
        print(f"-> Summary data successfully written to: {summary_file}")
        
def analyze_ekf():
    print("\n" + "="*75)
    print(" 4.6 EKF STATE ESTIMATION PARITY (BATCH AGGREGATE)")
    print("="*75)
    files = get_all_files('ekf_L*_R*.csv')
    if not files: 
        print("-> No EKF validation logs found in this batch.")
        return

    raw_rmses = []
    ekf_rmses = []
    improvements = []
    
    print(f"{'Trial (Filename)':<25} | {'Raw RMSE (m)':<15} | {'EKF RMSE (m)':<15} | {'Imprv (%)':<10}")
    print("-" * 75)
    
    for f in files:
        df = pd.read_csv(f)
        
        # --- ALIGN GROUND TRUTH TO START AT THE SAME ORIGIN AS ODOMETRY ---
        gt_x_aligned = df['Ground_Truth_X'] - df['Ground_Truth_X'].iloc[0] + df['Raw_Odom_X'].iloc[0]
        gt_y_aligned = df['Ground_Truth_Y'] - df['Ground_Truth_Y'].iloc[0] + df['Raw_Odom_Y'].iloc[0]
        
        # Calculate raw errors using the aligned ground truth paths
        raw_err = np.sqrt((df['Raw_Odom_X'] - gt_x_aligned)**2 + (df['Raw_Odom_Y'] - gt_y_aligned)**2)
        ekf_err = np.sqrt((df['EKF_X'] - gt_x_aligned)**2 + (df['EKF_Y'] - gt_y_aligned)**2)
        
        # Compute RMSE
        raw_rmse = calculate_rmse(raw_err)
        ekf_rmse = calculate_rmse(ekf_err)
        
        # Calculate improvement
        imp = ((raw_rmse - ekf_rmse) / raw_rmse) * 100 if raw_rmse > 0 else 0
        
        raw_rmses.append(raw_rmse)
        ekf_rmses.append(ekf_rmse)
        improvements.append(imp)
        
        print(f"{os.path.basename(f):<25} | {raw_rmse:<15.4f} | {ekf_rmse:<15.4f} | {imp:.2f}%")

    print("-" * 75)
    print(f"OVERALL AGGREGATE (n={len(improvements)}):")
    print(f"Mean Raw RMSE:    {np.mean(raw_rmses):.4f} m")
    print(f"Mean EKF RMSE:    {np.mean(ekf_rmses):.4f} m")
    print(f"Mean Improvement: {np.mean(improvements):.2f}%")
    print(f"Std Dev:          {np.std(improvements):.2f}%")

def main():
    print(f"\n[METRIC CALCULATOR] Scanning targets in: {LOG_DIR}")
    if not os.path.exists(LOG_DIR):
        print(f"[ERROR] Directory does not exist: {LOG_DIR}")
        return
        
    analyze_hardware_and_rtf()
    analyze_kinematic_square()
    analyze_wall_follower()
    analyze_pure_pursuit()
    analyze_ekf()
    print("\n" + "="*50 + "\n")

if __name__ == '__main__':
    main()
