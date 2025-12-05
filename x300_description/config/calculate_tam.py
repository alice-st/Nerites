import numpy as np
import yaml

# === Thruster positions from URDF ===
positions = {
    0: [-1.15,  0.25, 0.0],
    5: [-1.15, -0.25, 0.0],
    1: [-0.98,  0.25, 0.0],
    6: [-0.98, -0.25, 0.0],
    4: [ 0.91,  0.25, 0.0],
    9: [ 0.91, -0.25, 0.0],
    2: [-0.88,  0.25, 0.0],
    7: [-0.88, -0.25, 0.0],
    3: [ 0.81,  0.25, 0.0],
    8: [ 0.81, -0.25, 0.0]
}

# === Thruster directions from URDF ===
# directions = {
#     0: [1, 0, 0], #back_thruster
#     5: [1, 0, 0], #back_thruster
#     1: [0, 0, 1], #front_thruster_vertical
#     6: [0, 0, 1], #front_thruster_vertical
#     4: [0, 0, 1], #front_thruster_vertical
#     9: [0, 0, 1], #front_thruster_vertical
#     2: [0, 1, 0], #front_thruster_horizontal
#     7: [0, 1, 0], #front_thruster_horizontal
#     3: [0, 1, 0], #front_thruster_horizontal
#     8: [0, 1, 0]  #front_thruster_horizontal
# }

directions = {
    0: [ 1,  0,  0],  # surge
    5: [ 1,  0,  0],  # surge
    2: [ 0,  1,  0],  # heave
    7: [ 0,  1,  0],  # heave
    3: [ 0,  1,  0],  # heave
    8: [ 0,  1,  0],  # heave
    1: [ 0,  0,  1],  # sway
    6: [ 0,  0,  1],  # sway
    4: [ 0,  0,  1],  # sway
    9: [ 0,  0,  1],  # sway
}

# === Build TAM ===
thruster_ids = sorted(positions.keys())  # Ensure consistent order: [0,1,2,3,4,5,6,7,8,9]
tam = np.zeros((6, len(thruster_ids)))

for idx, tid in enumerate(thruster_ids):
    r = np.array(positions[tid])   # position vector
    d = np.array(directions[tid])  # thrust direction
    torque = np.cross(r, d)        # torque contribution
    tam[:, idx] = np.concatenate((d, torque))

# === Flatten TAM for ROS2 YAML ===
tam_flat = tam.flatten().tolist()

# === Prepare ROS 2 YAML structure ===
ros2_yaml = {
    "/**": {
        "ros__parameters": {
            "tam": tam_flat
        }
    }
}

# === Write YAML file ===
with open("x300_TAM.yaml", "w") as f:
    yaml.dump(ros2_yaml, f, default_flow_style=None, sort_keys=False)

print("TAM exported in ROS2 format to x300_tam.yaml")
