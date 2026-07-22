# A Tool for Simulation and Control of a Mobile Robot Using ROS 2 and Webots

An integrated experimentation platform for configuring, controlling, simulating, and evaluating mobile robot experiments using **ROS 2 Humble** and **Webots R2025a**. The platform provides a graphical user interface (GUI) that simplifies experiment setup while supporting common robotics algorithms such as teleoperation, wall following, trajectory tracking, and obstacle avoidance.

---

## Overview

Developing and evaluating mobile robot algorithms often requires launching multiple ROS nodes, configuring simulation environments, and manually collecting experimental results. This project integrates these tasks into a single platform that allows users to configure experiments through a graphical interface, execute simulations, and automatically collect performance data.

The platform combines:

- ROS 2 Humble
- Webots R2025a
- PyQt5 graphical user interface
- Docker for reproducible deployment

The project was developed as a final-year Mechatronics Engineering project.

---

## Features

- Graphical user interface for experiment configuration
- ROS 2 Humble integration
- Webots R2025a simulation
- Teleoperation using keyboard control
- PID wall-following controller
- Trajectory planning and tracking
- LiDAR-based obstacle avoidance
- Automatic experiment logging
- Performance monitoring
- Docker support for reproducibility

---

# Screenshots

## Graphical User Interface

![GUI](docs/images/gui.png)

The GUI provides a centralized interface for configuring experiments, selecting simulation worlds, and getting data for analysis.

---

## Webots Simulation Environment

![Simulation](docs/images/a_world_file.png)

Experiments are executed inside Webots using a differential-drive mobile robot equipped with sensors including LiDAR, IMU, GPS, and wheel encoders.

---

# Supported Software

| Component | Version |
|------------|---------|
| Ubuntu | 22.04 LTS |
| ROS | ROS 2 Humble |
| Webots | R2025a |
| Python | 3.10 |
| Docker | Latest |

---

# Repository Structure

```text
.
├── docs/
│   └── images/
│       ├── gui.png
│       └── a_world_file.png
├── ros2_ws/
│   └── src/
│       └── robot_simulation/
├── simulation_logs/
├── Dockerfile
├── .dockerignore
├── .gitignore
└── README.md
```

---

# Getting Started

## Clone the Repository

```bash
git clone https://github.com/<YOUR_USERNAME>/robot-simulation-control-tool.git
cd robot-simulation-control-tool
```

> Cloning creates a folder named `robot-simulation-control-tool` (matching the repo name above) — `cd` into that. If you instead already have a local copy of this project under a different folder name (e.g. from a `.tar` extract or an existing working copy), `cd` into that folder instead; the folder name itself doesn't matter, what matters is that it's the one containing the `Dockerfile`.

---

## Allow X11 Connections (Linux host)

The GUI is displayed on your host desktop via X11 forwarding. Before running the container for the first time in a session, allow local connections:

```bash
xhost +local:docker
```

This must be run in a terminal on your actual desktop session (not over SSH without X forwarding configured).

---

## Build the Docker Image

Build the image once after cloning the repository, and whenever you change code under `ros2_ws/src`. Source files are copied into the image at build time, so a rebuild is required to pick up code changes — there is no live source mount.

```bash
docker build -t robot_simulation:v1.2 .
```

> **Note on tags:** always build with an explicit version tag (e.g. `v1.2`) and run that same tag. If you omit the tag, Docker defaults to `:latest`, which only updates if you explicitly build it — running an untagged image can silently run an old build even after you've made changes. If you want `docker run robot_simulation` (no tag) to always grab your newest build, tag both at once:
> ```bash
> docker build -t robot_simulation:v1.2 -t robot_simulation:latest .
> ```

---

## Run the Container

```bash
docker run -it \
    --name robot_tool \
    --net=host \
    --env DISPLAY=$DISPLAY \
    --env QT_X11_NO_MITSHM=1 \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --volume ~/simulation_logs:/home/robot/simulation_logs \
    --device /dev/dri \
    --device /dev/snd \
    --group-add video \
    robot_simulation:v1.2
```

This creates a new container named `robot_tool` from the `robot_simulation:v1.2` image, with:

- Host networking (`--net=host`) so ROS 2 DDS discovery works normally
- X11 forwarding so GUI windows (PyQt5 and Webots) display on your desktop
- GPU/audio device passthrough (`/dev/dri`, `/dev/snd`) for rendering
- A mounted logs folder (`~/simulation_logs` on your host ↔ `/home/robot/simulation_logs` in the container) so experiment CSVs persist after the container stops

If a container named `robot_tool` already exists (e.g. from a previous run), remove it first so you don't accidentally attach to a stale one:

```bash
docker rm -f robot_tool
```

To resume an existing (stopped) container instead of creating a new one:

```bash
docker start -ai robot_tool
```

---

## Open a Terminal Inside the Container

If you need an additional shell inside the already-running container (e.g. to run a second ROS node or check logs), open a new terminal on the host and run:

```bash
docker exec -it robot_tool bash
```

---

## Launch the GUI

Inside the container:

```bash
ros2 run robot_simulation gui
```

The GUI will appear on the host desktop using X11 forwarding.

---

# Available Experiments

## Teleoperation

Allows manual control of the mobile robot using keyboard commands. This experiment demonstrates how user inputs are translated into ROS 2 velocity commands and executed by the robot.

---

## Wall Following

Implements a PID-based wall-following controller. Users can modify controller gains through the GUI and evaluate the robot's tracking performance.

---

## Trajectory Planning and Tracking

Generates smooth waypoint-based trajectories and tracks them using a Pure Pursuit controller. Performance metrics and trajectory plots are automatically generated.

---

# Project Components

The project consists of:

- ROS 2 nodes
- Robot models (URDF)
- Webots worlds
- Controller implementations
- PyQt5 graphical interface
- Configuration files
- Docker environment

---

# Docker

The repository includes a Dockerfile for building a reproducible environment.

The Docker image contains:

- Ubuntu 22.04
- ROS 2 Humble
- Webots R2025a
- Required ROS packages
- Required Python dependencies
- A pre-built ROS workspace (`colcon build --symlink-install`, baked in at image build time)

Source code under `ros2_ws/src` is copied into the image when it's built. To pick up code changes, rebuild the image (see [Build the Docker Image](#build-the-docker-image)) — there is currently no bind-mounted source directory for live editing.

---

# Simulation Logs

Simulation logs generated during experiments are stored on the host machine in

```text
~/simulation_logs/
```

This directory is mounted into the container at `/home/robot/simulation_logs` via the `--volume` flag in the `docker run` command above, so logs persist after the container is stopped or removed.

---

# Troubleshooting

**GUI window doesn't appear / X11 errors:**
Run `xhost +local:docker` on the host before starting the container, and confirm `$DISPLAY` is set in the terminal you're running `docker run` from.

**Container won't start because the name is already in use:**
```bash
docker rm -f robot_tool
```
then re-run the `docker run` command.

**Code changes don't seem to take effect after rebuilding:**
Confirm you're running the tag you just built — check with `docker images | grep robot_simulation` and make sure your `docker run` command references that exact tag (not a bare `robot_simulation`, which defaults to `:latest`).

---

# Future Work

Possible future improvements include:

- SLAM integration
- Navigation2 support
- Multi-robot simulation
- Additional robot platforms
- More experiment scenarios
- Additional evaluation metrics
- Support for external ROS packages
- Docker Compose support (currently untested — a `docker-compose.yaml` may be reintroduced once verified)

---

# Authors

- **Dauda Theophilus**
- **Bamikole John**
- **Ishaku Joseph**
- **Bisong Prince**

### Supervisors

- **Dr. K. O. Shobowale**
- **Engr. M. Habib**

---

# Citation

If you use this project in academic work, please cite the associated undergraduate thesis.

---

# License

This project is currently released without an open-source license.

All rights reserved by the authors.
