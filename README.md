# Nerites Underwater Simulator

## Motivation

The **Nerites Underwater Simulator** is a **complete ROS 2 Foxy port and extension of the original UUV Simulator framework (https://github.com/uuvsimulator/uuv_simulator)**.

It focuses on:

- Full **ROS 1 → ROS 2 Foxy** migration of underwater simulation tools  
- Support for **realistic AUV/ROV vehicles**  
- **Reinforcement learning (RL)** for underwater control  
- **Underwater manipulation** with Oberon arms  
- Multi-vehicle, sensor-rich simulation  

Nerites currently includes the following vehicles:

- **RexROV**  (https://www.researchgate.net/figure/RexRov-vehicle-in-simulation_fig1_367514536)
- **ECA A9** (https://uuvsimulator.github.io/packages/eca_a9/docs/packages/eca_a9_description/)
- **Twin GRAALTech X300** (https://www.graaltech.com/products/x-300-auv/)
- **Oberon4** and **Oberon7** manipulators  (https://github.com/uuvsimulator/uuv_manipulators)
- A **Gym-compatible RL framework** with **Soft Actor-Critic (SAC)** control training pipelines for RexROV, ECA_A9 and X300 vehicles.  

---

## Features

### Underwater Worlds

Nerites ports and extends the original UUV Simulator worlds to ROS 2:

- Ocean environments  
- Pool / tank setups  
- Hydrodynamic & buoyancy models  
- Current, turbidity and noise plugins  
- Structured test environments for benchmarking  

---

### Supported Vehicles

| Vehicle | Description |
|--------|-------------|
| **RexROV** | Work-class ROV with full thruster configuration |
| **RexROV + Oberon4** | ROV with 4-DOF manipulator |
| **RexROV + Oberon7** | ROV with 7-DOF serial arm |
| **RexROV + Dual Arms** | ROV with **both** Oberon4 and Oberon7 |
| **ECA A9 AUV** | Torpedo-style autonomous underwater vehicle |
| **GRAALTech X300** | Compact hybrid AUV/ROV with hovering and profiling capability |

<p align="center">
  <img src="images/rexrov.png" width="300">
  <img src="images/eca_a9.png" width="300">
  <img src="images/x300.png" width="300">
  <img src="images/oberon4.png" width="300">
  <img src="images/oberon7.png" width="300">
</p>

All vehicles include hydrodynamic models, thruster layouts, sensor suites, ROS 2 interfaces and launch files.

---

## Manipulators (Oberon4 & Oberon7)

Nerites supports:

| Arm | DOF | Notes |
|-----|-----|------|
| **Oberon4** | 4 | ROV-mounted arm with jaw gripper |
| **Oberon7** | 7 | Serial arm with wrist and parallel gripper |
| **Oberon4 + Oberon7** | 11 | Dual-arm setup on RexROV |

Features:

- `gazebo_ros2_control` integration  
- `joint_state_broadcaster`  
- Position and effort controllers  
- Gripper control (jaw / parallel)  
- Kinematics service (FK/IK)  

---

## GRAALTech X300 AUV (Hybrid AUV/ROV)

The **GRAALTech X300** is a compact, **torpedo-shaped hybrid AUV/ROV**, designed for:

- Long-range autonomous navigation  
- Precise hovering and station-keeping  
- Environmental monitoring and inspection  

In Nerites, X300 is fully modeled in ROS 2 with:

- Thruster allocation  
- Hydrodynamics  
- Sensors  
- A dedicated **SAC RL training environment** (`x300_env.py`)  

### Actuation (Simulated in Nerites)

- **Two rear main thrusters** → Surge (forward/backward)  
- **Four vertical tunnel thrusters** → Depth / heave control  
- **Four horizontal tunnel thrusters** → Yaw and lateral maneuvering  

This supports:

- AUV cruise mode  
- Hover / station-keeping  
- Vertical profiling  
- Low-speed precision maneuvering  

---

## Controllers

### RexROV Trajectory Controllers (Verified)

The following **trajectory controllers for RexROV** are fully working in the ROS 2 Foxy port:

1. **Launch a Gazebo world**

```
ros2 launch uuv_gazebo_worlds auv_underwater_world.launch.py
```

Launch RexROV

```
ros2 launch uuv_descriptions upload_rexrov_default.launch.py
```

## Run one of the controllers

# Classic PID controller
```
ros2 launch uuv_trajectory_control rov_pid_controller.launch.py uuv_name:=rexrov
```

# Model-based finite-time Lyapunov controller
```
ros2 launch uuv_trajectory_control rov_mb_fl_controller.launch.py uuv_name:=rexrov
```

# Model-based sliding mode controller
```
ros2 launch uuv_trajectory_control rov_mb_sm_controller.launch.py uuv_name:=rexrov
```

# Nonlinear PID controller
```
ros2 launch uuv_trajectory_control rov_nl_pid_controller.launch.py uuv_name:=rexrov model_name:=rexrov
```

# PD + gravity compensation controller
```
ros2 launch uuv_trajectory_control rov_pd_grav_compensation_controller.launch.py uuv_name:=rexrov model_name:=rexrov
```

# Sliding mode + feedforward (SF) controller
```
ros2 launch uuv_trajectory_control rov_sf_controller.launch.py uuv_name:=rexrov
```

# Underactuated AUV PID controller (UA-PID)
```
ros2 launch uuv_trajectory_control rov_ua_pid_controller.launch.py uuv_name:=rexrov
```

# Non-model-based sliding mode controller
```
ros2 launch uuv_trajectory_control rov_nmb_sm_controller.launch.py uuv_name:=rexrov model_name:=rexrov
```

## Send a waypoint file

```
ros2 launch uuv_control_utils send_waypoints_file.launch.py uuv_name:=rexrov
```

This pipeline gives you: world → vehicle → controller → waypoint tracking.
Disturbance & Thruster Tutorials

The following tutorial launch files are fully working and useful for testing disturbances, thruster behavior and robustness:

# Disturbance manager tutorial
```
ros2 launch uuv_tutorial_disturbances tutorial_disturbance_manager.launch.py
```

# Body wrench (external forces/torques)
```
ros2 launch uuv_tutorial_disturbances tutorial_body_wrench.launch.py
```

# Thruster efficiency changes
```
ros2 launch uuv_tutorial_disturbances tutorial_thruster_efficiency.launch.py
```

# Thruster state visualization/monitoring
```
ros2 launch uuv_tutorial_disturbances tutorial_thruster_state.launch.py
```

# Time-varying current fields
```
ros2 launch uuv_tutorial_disturbances tutorial_timed_current.launch.py
```

These are perfect to combine with the RexROV controllers for robustness and RL training scenarios.

## Reinforcement Learning Environment

Nerites provides a Gym-style RL interface adapted to underwater vehicles.
Core Environment:

Base_env.py: Base environment for AUV/ROV control

* reset() / step() API
* Continuous action spaces (thrusters, fins, etc.)
* Rich observation spaces (pose, velocities, sensors)

RL Support Per Vehicle

* RexROV → rexrov_env.py
* ECA A9 → eca_a9_env.py
* X300 → x300_env.py

All three are trained using Soft Actor-Critic (SAC) via a custom training script.

# How to Train SAC (RexROV / ECA A9 / X300)

* Choose your vehicle: eca_a9, rexrov or x300
* Point the environment and trainer to the right settings file:
In base_uuv_env.py and custom_algorithms/my_sac_train.py update the settings path so it points to the configuration file for your chosen vehicle, e.g.:

SETTINGS_PATH = "settings/settings_rexrov.txt"
# or
SETTINGS_PATH = "settings/settings_eca_a9.txt"
# or
SETTINGS_PATH = "settings/settings_x300.txt"

* Edit the settings file for your vehicle

Open the matching settings_<vehicle>.txt file, for example:

* settings_rexrov.txt
* settings_eca_a9.txt
* settings_x300.txt

and adjust parameters such as:

* Episode length
* Number of steps per episode
* Observation and action scaling
* SAC hyperparameters (learning rates, γ, τ, etc.)

Run training

From the RL package directory (e.g. nerites_gym or your RL project root):

python3 custom_algorithms/my_sac_train.py

This will:

* Instantiate the chosen vehicle environment (RexROV / ECA A9 / X300)

* Load parameters from settings_<vehicle>.txt

* Train a SAC policy using the custom training loop

<p align="center"> <img width="650" src="images/rl_training.png"> </p>
Requirements

This project requires:

* ROS 2 Foxy
* Gazebo Classic 11
* Python ≥ 3.8
* Colcon build tools

# Installation
Clone the workspace

git clone <your-repo-url> ~/nerites_sim_ws
cd ~/nerites_sim_ws/src

Install dependencies

rosdep update
rosdep install --from-paths . --ignore-src -y

Build

cd ~/nerites_sim_ws
colcon build --symlink-install
source install/setup.bash

Running the Simulator (Quick Start)
1. Launch world + RexROV + controller

# World
ros2 launch uuv_gazebo_worlds auv_underwater_world.launch.py

# ROV
ros2 launch uuv_descriptions upload_rexrov_default.launch.py

# Choose ONE controller (example: nonlinear PID)
ros2 launch uuv_trajectory_control rov_nl_pid_controller.launch.py uuv_name:=rexrov model_name:=rexrov

# Send trajectory waypoints
ros2 launch uuv_control_utils send_waypoints_file.launch.py uuv_name:=rexrov

Usage
View camera output

rqt_image_view /rexrov/camera/image

(Or any other camera topic active in the selected world/vehicle setup.)
View sonar output

rviz2 -d sonar_visualization.rviz

Repository Structure

nerites_sim_ws/
└── src/
    ├── Nerites/
    │   ├── uuv_simulator        # ROS 2 port of original UUV Simulator
    │   ├── uuv_descriptions
    │   ├── uuv_gazebo
    │   ├── uuv_gazebo_worlds
    │   ├── uuv_sensor_plugins
    │   ├── uuv_world_plugins
    │   ├── uuv_control
    │   ├── uuv_assistants
    │   ├── uuv_teleop
    │   ├── nerites_gym          # Gym interface + RL envs + SAC training
    │   └── nerites_utils        # Helper utilities
    │
    ├── eca_a9_nerites/
    ├── x300_description/
    ├── oberon4_control/
    ├── oberon7_control/
    ├── kdl_parser_py/
    └── ...

Contributing

Contributions are welcome. You can help with:

* New vehicle models
* Improved sonar / sensor simulation
* Manipulation and dual-arm coordination
* New RL tasks and benchmarks
* Documentation and tutorials

Workflow: Fork → Branch → Pull Request.
Cite as

@misc{nerites2025,
  title  = {Nerites Underwater Simulator (ROS 2)},
  author = {Alice S. and Contributors},
  year   = {2025},
  url    = {https://github.com/<your-repo>}
}

Final Note

Nerites is a research-grade ROS 2 Foxy port of the UUV Simulator ecosystem, extended with:

* Realistic AUV/ROV vehicle models
* Underwater manipulator support
* Gym-compatible RL environments
* SAC-based training for RexROV, ECA A9 and X300

You bring the ideas. Nerites gives you the ocean. 🌊🤖

::contentReference[oaicite:0]{index=0}
