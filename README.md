Nerites Underwater Simulator
Motivation

The Nerites Underwater Simulator is a complete ROS 2 Foxy translation and extension of the original UUV Simulator framework, focused on:

✅ Full ROS 1 → ROS 2 Foxy migration

✅ Modern underwater vehicle simulation

✅ Reinforcement learning for underwater robotics

✅ Autonomous and teleoperated vehicle control

✅ Underwater manipulation with Oberon arms

This project ports and extends the original UUV Simulator ecosystem with:

RexROV (ROV + manipulators)

ECA A9 AUV

GRAALTech X-300 hybrid AUV/ROV

It additionally introduces a native Gym-compatible RL environment with:

base_uuv_env

Dedicated SAC training pipelines for all three vehicles

<p align="center"> <img width="600" src="images/nerites_banner.png"> </p>
Features
Underwater Worlds

All original UUV Simulator environments are fully ported to ROS 2:

Ocean worlds

Pool/tank environments

Hydrodynamic & buoyancy models

Current, turbidity, and noise plugins

Structured testing environments

Supported Vehicles
Vehicle Description
RexROV  Heavy-duty work-class ROV
RexROV + Oberon4  ROV with 4-DOF manipulator
RexROV + Oberon7  ROV with 7-DOF manipulation arm
ECA A9 AUV  Torpedo-style autonomous underwater vehicle
GRAALTech X-300 Hybrid hovering AUV/ROV
<p align="center"> <img width="650" src="images/vehicles.png"> </p>

All vehicles include:

Hydrodynamic models

Thruster allocation

ROS 2 control interfaces

Gazebo physics integration

Sensor plugins

Autonomous & RL-ready configurations

GRAALTech X-300 AUV (Hybrid AUV/ROV)

The GRAALTech X-300 is a compact hybrid autonomous/teleoperated underwater vehicle, designed for:

Precise hovering

Long-range autonomous navigation

Inspection and environmental monitoring

In Nerites, X-300 is fully integrated into ROS 2 Foxy with:

Thruster allocation

Hydrodynamics

Sensors

Full SAC reinforcement learning control

Real Vehicle Reference Specs
Property  Value
Length  ~2.22 m
Diameter  ~0.155 m
Weight  ~29 kg
Max depth 300 m
Max speed ~5 knots
Endurance Up to 14 h
Battery 24 V Li-Ion (~1200 Wh)
Buoyancy  Active buoyancy system
Trim  Internal movable mass
Actuation (Simulated)

Main rear propeller → Surge

Vertical tunnel thrusters → Heave & depth

Horizontal tunnel thrusters → Yaw & sway

Buoyancy engine → Vertical profiling

Internal movable mass → Pitch/trim stabilization

Sensors (Simulated)

RGB camera

GPU sonar

IMU

Depth / pressure

DVL

Ground-truth odometry

Controllers

Nerites includes:

NED & ENU controllers

Velocity, depth & heading controllers

Station-keeping

Thruster allocation

Teleoperation nodes

cmd_vel and autonomous mission interfaces

Sensor Suite

RGB camera

GPU sonar (gpu_ray)

IMU

Depth sensor

DVL

LaserScan

CameraInfo

<p align="center"> <img width="650" src="images/sensor_suite.png"> </p>
Reinforcement Learning Framework

Nerites provides a native Gym-style RL environment built directly on ROS 2:

base_uuv_env

Continuous control

Domain randomization

Sensor noise injection

Hydrodynamic disturbances

Supported RL Vehicles

✅ RexROV
✅ ECA A9
✅ GRAALTech X-300

Training Algorithms

Soft Actor-Critic (SAC)

Training Scripts
train_rexrov_sac.py
train_eca_a9_sac.py
train_x300_sac.py

Tasks

Hover stabilization

Depth keeping

Waypoint following

Disturbance rejection

Long-horizon navigation

Requirements

ROS 2 Foxy

Gazebo Classic 11

Python ≥ 3.8

Colcon

Installation
git clone <your-repo-url> ~/nerites_sim_ws
cd ~/nerites_sim_ws/src
rosdep update
rosdep install --from-paths . --ignore-src -y
cd ~/nerites_sim_ws
colcon build --symlink-install
source install/setup.bash

Running
Launch an underwater world
ros2 launch uuv_gazebo_worlds ocean_world.launch.py

Spawn RexROV
ros2 launch rexrov_sim bringup.launch.py

Spawn RexROV + Oberon7
ros2 launch rexrov_sim bringup.launch.py use_arm:=oberon7

Launch ECA A9
ros2 launch eca_a9_nerites start_eca_a9.launch.py

Spawn X-300
ros2 launch x300_description spawn_x300.launch.py

RL Example
import gym
import nerites_gym

env = gym.make("NeritesAUVEnv-v0")
obs = env.reset()

for step in range(1000):
    action = policy(obs)
    obs, reward, done, info = env.step(action)
    if done:
        obs = env.reset()

<p align="center"> <img width="650" src="images/rl_training.png"> </p>
Repository Structure
nerites_sim_ws/
└── src/
    ├── Nerites/
    │   ├── uuv_simulator (ROS 2 port)
    │   ├── uuv_descriptions
    │   ├── uuv_gazebo
    │   ├── uuv_gazebo_worlds
    │   ├── uuv_sensor_plugins
    │   ├── uuv_world_plugins
    │   ├── uuv_control
    │   ├── uuv_assistants
    │   ├── uuv_teleop
    │   ├── nerites_gym
    │   └── nerites_utils
    │
    ├── eca_a9_nerites/
    ├── x300_description/
    └── kdl_parser_py/

Contributing

You may contribute with:

Vehicles

Sensors

Sonar simulation

Manipulation

RL benchmarks

Documentation

Citation
@misc{nerites2025,
  title  = {Nerites Underwater Simulator (ROS 2)},
  author = {Alice S. and Contributors},
  year   = {2025},
  url    = {https://github.com/<your-repo>}
}

Final Note

Nerites is the first complete ROS 2 Foxy migration of the UUV Simulator ecosystem with native reinforcement learning support.

It provides a realistic research platform for:

AUV autonomy

ROV manipulation

Underwater control

Sensor fusion

Reinforcement learning

Now the ocean is programmable. 🌊🤖