🌊 Nerites Underwater Simulator

A Modular ROS 2 Underwater Robotics Framework for AUVs, ROVs & Reinforcement Learning

<p align="center"> <img src="https://img.shields.io/badge/ROS-2%20Foxy-blue.svg" /> <img src="https://img.shields.io/badge/Gazebo-Classic-orange.svg" /> <img src="https://img.shields.io/badge/Physics-Underwater%20Hydrodynamics-00b3b3.svg" /> <img src="https://img.shields.io/badge/RL-Gym%20Environment-green.svg" /> <img src="https://img.shields.io/badge/Status-Active-success" /> </p>
🐋 Overview

Nerites Underwater Simulator is a complete ROS 2 Foxy–based simulation ecosystem for underwater robotics research, supporting AUVs, ROVs, manipulators, GPU sonar, camera sensors, and reinforcement learning environments.

Built on top of the UUV Simulator framework and extended with custom vehicle models and controllers, Nerites provides everything needed for:

⚓ Autonomous underwater robotics

🤖 Control & thrust allocation research

🧠 Reinforcement learning for underwater vehicles

🛠 Manipulation tasks with Oberon4 & Oberon7 arms

🌍 Simulation in realistic ocean and testbed environments

🚀 Key Features
🌍 Underwater Worlds

Includes all environments from UUV Simulator:

Ocean environments

Pool/tank worlds

Structured testbeds

Hydrodynamic & buoyancy plugins

Current, turbidity & noise models

🤖 Supported Vehicles
Vehicle  Description
RexROV   Heavy-duty ROV with full thruster model
RexROV + Oberon4  ROV + 4-DOF manipulator
RexROV + Oberon7  ROV + 7-DOF high-precision arm
ECA A9 AUV  Torpedo-style autonomous underwater vehicle
X300 AUV/ROV   Lightweight, configurable research vehicle

All vehicles include:

Hydrodynamics

Thruster models

Sensors

ROS 2 interfaces

Launch configurations

🎛️ Controllers

Nerites includes modular ROS 2 controllers:

NED controller

Non-NED controller

Velocity, depth, heading controllers

Station keeping

Control allocator for multi-thruster vehicles

🎥 Sensor Suite

RGB camera

GPU sonar (gpu_ray sensor)

IMU (Accel + Gyro)

Depth sensor

DVL emulation

Noisy underwater dynamics

ROS 2 Camera Info & LaserScan publishers

🧠 Gym Environment for Reinforcement Learning

A built-in RL environment compatible with:

Stable Baselines

RLlib

CleanRL

Custom PyTorch policies

Features:

reset() / step() Gym API

Randomized underwater conditions

Task variations (hovering, waypoint following, depth keeping)

Multi-vehicle support

📦 Repository Structure
nerites_sim_ws/
└── src/
    ├── Nerites/
    │   ├── uuv_simulator
    │   ├── uuv_descriptions
    │   ├── uuv_gazebo
    │   ├── uuv_gazebo_worlds
    │   ├── uuv_sensor_plugins
    │   ├── uuv_world_plugins
    │   ├── uuv_control
    │   ├── uuv_assistants
    │   ├── uuv_teleop
    │   ├── nerites
    │   └── nerites_utils
    │
    ├── eca_a9_nerites/
    ├── x300_description/
    ├── kdl_parser_py/
    └── ...

🔧 Installation
1️⃣ Install ROS 2 Foxy

https://docs.ros.org/en/foxy/Installation.html

2️⃣ Install Gazebo Classic
sudo apt install gazebo11 ros-foxy-gazebo-*

3️⃣ Clone the Simulator Workspace
git clone <your-repo-url> ~/nerites_sim_ws
cd ~/nerites_sim_ws/src

4️⃣ Install dependencies
rosdep update
rosdep install --from-paths . --ignore-src -y

5️⃣ Build
cd ~/nerites_sim_ws
colcon build --symlink-install
source install/setup.bash

▶️ Launching the Simulation
🌍 Start a world
ros2 launch uuv_gazebo_worlds ocean_world.launch.py

🤖 Spawn RexROV
ros2 launch rexrov_sim bringup.launch.py

🤖 Spawn RexROV + Oberon7 arm
ros2 launch rexrov_sim bringup.launch.py use_arm:=oberon7

🚀 Launch the ECA A9
ros2 launch eca_a9_nerites start_eca_a9.launch.py

🐋 Launch the X300
ros2 launch x300_description spawn_x300.launch.py

📡 Viewing Sensors
Camera
rqt_image_view /eca_a9/camera/image

Sonar
rviz2 -d sonar_visualization.rviz

🧠 Reinforcement Learning Example
import gym
import nerites_gym

env = gym.make("NeritesAUVEnv-v0")
obs = env.reset()

for step in range(1000):
    action = policy(obs)
    obs, reward, done, info = env.step(action)
    if done:
        obs = env.reset()

🛠 Contributing

Contributions are welcome!
Areas where help is appreciated:

New vehicle models

Improved sonar simulation

Underwater manipulation

Reinforcement learning tasks

Documentation & tutorials

Fork → Branch → Pull Request.

📄 License

This project is licensed under Apache 2.0 (or your license of choice).

📢 Citation
@misc{nerites2025,
  title  = {Nerites Underwater Simulator},
  author = {Alice S., Contributors},
  year   = {2025},
  url    = {https://github.com/<your-repo>}
}

🌊 Dive In

Nerites provides a powerful, flexible simulation platform for underwater robotics, research, AI, and RL.

Whether you're developing:

AUV autonomy

ROV manipulation

Thruster allocation

Neural controllers

RL policies

Nerites gives you the ocean — go explore it.