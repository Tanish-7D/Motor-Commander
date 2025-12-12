# Motor-Commander

Stepper Motor Commander using ROS 2 Jazzy messages on a Raspberry Pi 4 to control a NEMA 17 stepper motor through PWM signals.

---

## Purpose

The main goals of this system are:

1. **Hardware** — Define required components and establish communication between the computer and the Raspberry Pi 4  
2. **Installation** — Set up all software, dependencies, and the ROS 2 workspace  
3. **Usage** — Run the ROS 2 node that generates PWM signals based on proportional control  
4. **Debugging** — Verify functionality and troubleshoot motor behavior or communication issues  

---

## Hardware

The system relies on the following hardware:

- **Stepper Motor** — NEMA 17  
- **Motor Driver** — MicroStepper Driver for WASP Extruder  
- **Microcontroller** — Raspberry Pi 4 running ROS 2 Jazzy and pigpio (hardware PWM)  

<img width="350" height="300" src="https://github.com/user-attachments/assets/6547f814-3ed1-442b-a861-d817cb965ae6" />

**GPIO Pin Mapping (Raspberry Pi 4)**

| Signal | GPIO Pin |
|--------|----------|
| PULSE  | 17       |
| DIR    | 27       |
| EN     | 22       |

---

## Installation

### Prerequisites

- ROS 2 Jazzy installed and sourced  
- Python ≥ 3.10  
- pigpio library  

```bash
sudo apt install pigpio python3-pigpio -y
```

**Note:** The node automatically starts the pigpiod daemon if it is not already running. You do not need to manually start it.

### 1. Clone the Repository

```bash
cd ~

git clone https://github.com/Tanish-7D/Motor-Commander.git pi_ws

cd pi_ws
```

**Note:** This command clones the Motor-Commander repository and creates a workspace directory called `pi_ws`. If you already have a `pi_ws` directory, either delete it first or clone with a different name.

This workspace includes two ROS 2 packages:
- **motorcontrol_messages** - Custom message definitions (FlowData.msg)
- **motor_commander** - Motor control nodes

Your workspace structure will look like:

```
~/pi_ws/
├── motorcontrol_messages/
│       ├── msg/
│       │   └── FlowData.msg
│       ├── CMakeLists.txt
│       └── package.xml
├── src/   
│   └── motor_commander/
│       ├── src/
│       │   ├── bldc_pwm.cpp
│       │   ├── dualmotortest.cpp
│       │   └── stepper_pwm.cpp
│       ├── CMakeLists.txt
│       ├── LICENSE
│       └── package.xml
└── README.md
```

### 2. Install Dependencies

```bash
cd ~/pi_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build the Packages

This workspace contains two packages that need to be built:
- `motorcontrol_messages` - Provides custom message types
- `motor_commander` - Motor control executables

```bash
cd ~/pi_ws
colcon build --symlink-install
```

**Note:** The build process will discover and build both the `motorcontrol_messages` package and the nested `motor_commander` package.

### 4. Source the Workspace

```bash
echo "source ~/pi_ws/install/setup.bash" >> ~/.bashrc
source ~/pi_ws/install/setup.bash
```

## Usage

### Start pigpio (if not using daemon)

```bash
sudo systemctl start pigpiod
```

### Source the workspace

```bash
source ~/pi_ws/install/setup.bash
```

### Run the Motor Commander node

```bash
ros2 run motor_commander stepper_pwm
```

### Send PWM / velocity commands (example)

```bash
ros2 topic pub /command std_msgs/Float32 "{data: 30.0}"
```

---

## Debugging

### Check pigpio service

```bash
systemctl status pigpiod
```

If not running:

```bash
sudo systemctl restart pigpiod
```

### Check GPIO output manually

```bash
pigs w 17 1
pigs w 17 0
```

### Check ROS topics

```bash
ros2 topic list
ros2 topic echo /command
```

### View node logs

```bash
ros2 run motor_commander stepper_pwm --ros-args --log-level debug
```
---
