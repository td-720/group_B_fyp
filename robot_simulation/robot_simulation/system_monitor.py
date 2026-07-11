#!/usr/bin/env python3
import psutil
import time
import csv
import os
from datetime import datetime

def read_current_rtf():
    """
    Attempts to read RTF from a shared file written by the Webots controller.
    If your controller doesn't write this file, this returns NaN.
    """
    rtf_file = '/tmp/webots_rtf.txt'
    try:
        if os.path.exists(rtf_file):
            with open(rtf_file, 'r') as f:
                return float(f.read().strip())
    except Exception:
        pass
    return float('nan')

def main():
    print("[SYSTEM MONITOR] Booting hardware profiler...")
    
    log_dir = os.path.expanduser('~/ros2_ws/simulation_logs')
    os.makedirs(log_dir, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_filename = os.path.join(log_dir, f'hardware_profile_{timestamp}.csv')

    with open(csv_filename, mode='w', newline='') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(['Time_Elapsed_s', 'CPU_Usage_Percent', 'RAM_Usage_MB', 'RAM_Usage_Percent', 'RTF'])

        start_time = time.time()
        print(f"[SYSTEM MONITOR] Logging to {csv_filename} at 1 Hz. Press Ctrl+C to stop.")
        print("[WARNING] Ensure Webots is writing to /tmp/webots_rtf.txt, or RTF will log as NaN.")

        try:
            while True:
                elapsed = round(time.time() - start_time, 2)
                cpu = psutil.cpu_percent(interval=1.0)
                
                ram = psutil.virtual_memory()
                ram_mb = round(ram.used / (1024 * 1024), 2)
                
                rtf = read_current_rtf()

                writer.writerow([elapsed, cpu, ram_mb, ram.percent, rtf])
                
                csv_file.flush()
                os.fsync(csv_file.fileno())

        except KeyboardInterrupt:
            print("\n[SYSTEM MONITOR] Hardware profiling terminated. Data saved safely.")

if __name__ == '__main__':
    main()
