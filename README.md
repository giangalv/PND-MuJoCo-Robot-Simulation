# PND MuJoCo Robot Simulation
A comprehensive MuJoCo-based physics simulation framework for PND humanoid robots (ADAM-U, ADAM-Lite, ADAM-SP) with ROS2/DDS integration. This project enables real-time control and monitoring of multiple robot configurations through a unified interface.

## Features
* 🤖 Multi-Robot Support - Seamlessly switch between ADAM-U, ADAM-Lite, and ADAM-SP robot models
* 🔄 Real-time DDS/ROS2 Integration - Full communication stack using CycloneDDS
* 📊 Advanced Physics Simulation - MuJoCo-based accurate dynamics simulation
* 🎮 Gamepad Support - Xbox and Nintendo Switch controller support for manual control
* 🧵 Multithreaded Architecture - Separate threads for physics simulation and visualization
* 🎯 Easy Robot Switching - Simple bash script for switching between robot configurations
* 📈 Sensor Publishing - Real-time joint state, IMU, and hand pose feedback
* ⚡ Elastic Band Control - Optional virtual spring-damper for interactive control

## Prerequisites
### System Requirements
* OS: Ubuntu 20.04 LTS or later (with ROS 2 Humble recommended)
* Python: 3.10 or higher
* RAM: Minimum 4GB (8GB recommended)
* Processor: Multi-core processor recommended

 ### Required Software
 * Python 3.10+
 * pip3 (Python package manager)
 * MuJoCo physics engine
 * ROS 2 (Humble or later - optional but recommended)

## Installation
### 1. Clone the Repository
```bash
cd ~
git clone https://github.com/giangalv/pnd_mujoco_simulation.git
cd pnd_mujoco_simulation
```
### 2. Install the PND Robotics SDK
```bash
# Install the SDK in editable mode
pip3 install -e .
```
This will automatically install all dependencies:
* `mujoco` - Physics simulation engine
* `cyclonedds` - DDF communication middleware
* `numpy` - Numerical computing
* `opencv-python` - Computer vision utilities
* `pygame` - Input handling for gamepad support
### 3. Verify Installation
```bash
# Test if the SDK is properly installed
python3 -c "import pndbotics_sdk_py; print('Installation successful!')"
```
### 4. Run the Simulation
```bash
cd pnd_mujoco/simulate_python/
python3 pnd_mujoco.py
```

## Quick Start Guide
### Basic Usage
1. Start the simulation with defaul robot (ADAM-U):
```bash
cd ~/pnd_mujoco_simulation/pnd_mujoco/simulate_python/
python3 pnd_mujoco.py
```
2. Switch to a different robot:
```bash
./switch_robot.sh adam_lite
# or
./switch_robot.sh adam_sp
# or
./switch_robot.sh adam_u
```
3. Run the simulation with the new robot:
```bash
python3 pnd_mujoco.py
```

## Available Robots
Robot	    | Description	                       | Actuators	                   | Best For
ADAM-U	  | Full humanoid with dexterous hands | 19 motors + 24 hand actuators | Complete humanoid research
ADAM-Lite	| Simplified humanoid without hands	 | 23 motors	                   | Mobile manipulation
ADAM-SP	  | Special purpose configuration	     | 29 motors	                   | Advanced manipulation tasks

## Configuration
The simulation is controlled through config.py located in pnd_mujoco/simulate_python/:
```python
# Robot configuration
ROBOT = "adam_u"  # Options: "adam_u", "adam_lite", "adam_sp"
ROBOT_SCENE = "../pnd_robots/" + ROBOT + "/scene.xml"

# Simulation parameters
SIMULATE_DT = 0.001  # Simulation timestep (1ms)
VIEWER_DT = 0.01     # Viewer update rate (10ms)

# DDS/ROS2 configuration
SDK_TYPE = "ROS2"    # Options: "ROS2", "DDS"
DOMAIN_ID = 0        # DDS Domain ID
INTERFACE = "lo"     # Network interface (lo for localhost)

# Features
ENABLE_ELASTIC_BAND = True      # Enable virtual spring-damper
USE_JOYSTICK = False            # Enable gamepad input
JOYSTICK_TYPE = "xbox"          # Options: "xbox", "switch"
PRINT_SCENE_INFORMATION = True  # Print robot structure on startup

# Hand control
HANDPOSE_SRC = 0  # Hand pose source (0 or 1 for different modes)
```
## Key Configuration Options
* ROBOT - Canghe which robot model to simulate
* SDK_TYPE - Choose between ROS2 and pure DDS communication
* ENABLE_ELASTIC_BAND - Toggle virtual spring-damper for interactive control
* USE_JOYSTICK - Enable/disable gamepad support

### Usage
## Running the Simulation
```bash
# Navigate to simulation directory
cd ~/pnd_mujoco_simulation/pnd_mujoco/simulate_python/

# Run with current configuration
python3 pnd_mujoco.py
```

## Switching Robots
the project includes an easy-to-use robot switchin script:
```bash
# Switch to ADAM-Lite
./switch_robot.sh adam_lite

# Switch to ADAM-SP
./switch_robot.sh adam_sp

# Check current robot
./switch_robot.sh
```

## Project Structure
```
pnd_mujoco_simulation/
├── pndbotics_sdk_py/           # Main SDK package
│   ├── core/                   # Core communication modules
│   │   ├── channel.py          # DDS channel implementation
│   │   └── ...
│   ├── idl/                    # Interface Definition Language files
│   │   ├── pnd_adam/           # ADAM robot IDL definitions
│   │   ├── default.py          # Default message types
│   │   └── ...
│   └── utils/                  # Utility functions
│       └── thread.py           # Threading utilities
│
├── pnd_mujoco/                 # MuJoCo simulation package
│   ├── pnd_robots/             # Robot model files
│   │   ├── adam_u/             # ADAM-U robot models
│   │   │   ├── scene.xml       # Scene configuration
│   │   │   ├── adam_u.xml      # Robot URDF
│   │   │   └── assets/         # 3D models and textures
│   │   ├── adam_lite/          # ADAM-Lite models
│   │   └── adam_sp/            # ADAM-SP models
│   │
│   └── simulate_python/        # Simulation scripts
│       ├── pnd_mujoco.py       # Main simulation entry point
│       ├── config.py           # Configuration file
│       ├── pndbotics_sdk_py_bridge.py    # SDK bridge
│       ├── pndbotics_ros_bridge.py       # ROS2 bridge
│       └── switch_robot.sh     # Robot switching utility
│
├── example/                    # Example scripts
│   └── low_level/              # Low-level control examples
│
└── README.md                   # This file
```

## Known Issues & Troubleshooting
ISSUE: `ModuleNotFoundError: No module named 'pndbotics_sdk_py'`
Solutin: Reinstall the package in editable mode:
```bash
cd ~/pnd_mujoco_simulation
pip3 install -e .
```
ISSUE: `ValueError: mjParseXML: Error opening file`
Solution: Ensure you're running the script from the correct directory:
```bash
cd ~/pnd_mujoco_simulation/pnd_mujoco/simulate_python/
python3 pnd_mujoco.py
```
ISSUE: Gamepad not detected
Solutio:
1. Verify the controller is connected: `ls /dev/input/js*`
2. Give Python permission to access input devices: `sudo usermod -a -G input $USER`
3. Log out and log back in for group changes to take effect

## Dependencies
All dependencies are automatically installed with `pip3 install -e .`:
Package	Version	Purpose
mujoco	Latest	Physics simulation
cyclonedds	0.10.2+	DDS middleware
numpy	Latest	Numerical computing
opencv-python	Latest	Computer vision
pygame	Latest	Input handling

## License 
This project is licensed under the MIT License.

## Citation
```bibtex
@software{pnd_mujoco_2024,
  title={PND MuJoCo Robot Simulation},
  author={Gia Galvez},
  year={2024},
}
```
## Changelog
v1.0.1
* ✅ Multi-robot simulation support (ADAM-U, ADAM-Lite, ADAM-SP)
* ✅ DDS/ROS2 integration
* ✅ Gamepad controller support
* ✅ Real-time physics simulation
* ✅ Easy robot switching utility
