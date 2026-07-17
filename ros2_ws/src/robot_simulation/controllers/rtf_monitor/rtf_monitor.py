from controller import Supervisor

def main():
    # Initialize the supervisor
    supervisor = Supervisor()
    time_step = int(supervisor.getBasicTimeStep())
    
    # Open the shared file once for the duration of the simulation
    with open('/tmp/webots_rtf.txt', 'w') as f:
        # Loop while the simulation is active
        while supervisor.step(time_step) != -1:
            # Query the physics engine directly for ground-truth RTF
            rtf = supervisor.getRealTimeFactor()
            
            # Efficiently overwrite the file content
            f.seek(0)
            f.write(f"{rtf:.4f}")
            f.truncate()
            
            # Update at approximately 2Hz (500ms)
            for _ in range(int(500 / time_step)):
                if supervisor.step(time_step) == -1:
                    break

if __name__ == "__main__":
    main()