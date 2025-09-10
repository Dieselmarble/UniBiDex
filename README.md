# 🤖 UniBiDex: A Unified Teleoperation Framework for Robotic Bimanual Dexterous Manipulation

<div align="center">

[![arXiv](https://img.shields.io/badge/arXiv-2501.XXXXX-b31b1b.svg)](https://arxiv.org/abs/2501.XXXXX)
[![Website](https://img.shields.io/badge/🌐-Website-blue.svg)](https://dieselmarble.github.io/UniBiDex/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python 3.10+](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/release/python-380/)
[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-green.svg)](https://docs.ros.org/en/humble/)

*A unified teleoperation framework for robotic bimanual dexterous manipulation that bridges the gap between human dexterity and robotic precision through intuitive haptic leader-follower control systems*

[**Paper**](https://arxiv.org/abs/2501.XXXXX) | [**Website**](https://dieselmarble.github.io/UniBiDex/) | [**Video**](https://youtu.be/XXXXX) | [**Data**](#datasets)

</div>

---

## 🔥 Highlights

**UniBiDex** is a comprehensive teleoperation framework that enables **unified bimanual dexterous manipulation** through VR controllers or leader-follower arms. UniBiDex enables real-time, contact-rich dual-arm teleoperation by integrating heterogeneous input devices into a shared control stack with consistent kinematic treatment and safety guarantees.

### Key Features
- 🎮 **Cross-Device Consistency**: Unified teleoperation algorithm across heterogeneous input sources (VR, leader-follower arms)
- 🛡️ **Motion Smoothness & Safety**: Null-space optimization and redundancy control for robust operation with singularity avoidance
- 🎯 **Haptic Feedback**: Current-based force feedback providing enhanced manipulation precision
- 🔧 **Modular Design**: Extensible architecture for different robots and control algorithms

---

## 📁 Repository Structure

```
UniBiDex/
├── 🎮 assets/                        # 3D models and calibration files
│   ├── images/                       
│   └── urdf/                         
├── 📜 scripts/                       # Main control scripts
│   ├── leader_controller.py                       
│   ├── visualize_example.py          
│   └── calib/                        # Calibration code for the leader arm
├── 🤖 unibidex_core/                 # Core UniBiDex framework
│   ├── agents/                       
│   ├── dynamixel/                    
│   └── robots/                       
├── 🎛️ unibidex_client/               # Robot control client
│   ├── motion_control/               # Motion controllers (XArm7, grippers)
│   ├── nodes/                        # ROS2 nodes and utilities
│   ├── configs/                      
│   └── tests/                        
├── � sim/                           # Simulation environments
│   └── envs/                         
└── 🔧 third_party/                   # Third-party dependencies
    ├── DynamixelSDK/                 
    └── mujoco_menagerie/             
```

---

## 🛠️ Configuration

#### Camera Configuration (If using data recording)
```bash
# Set up camera rules for consistent device naming
cd unibidex_client
sudo cp camera_roles.rules /etc/udev/rules.d/99-camera-roles.rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Run camera view selector to assign camera roles
python -m unibidex_client.nodes.camera_view_selector
```

#### Robot and Control Configuration
Edit configuration files in `unibidex_client/configs/`:
- `bimanual.yml`: Main bimanual control parameters
- Robot-specific configs for XArm7 and grippers
- Controller gains and limits
- Safety constraints  
- Hardware interface settings

---

## 🚀 Quick Start

### Prerequisites

- **OS**: Ubuntu 22.04 (recommended)
- **Python**: ≥ 3.8
- **Dependencies**: ROS2 Humble, MuJoCo ≥ 3.0, OpenCV, NumPy, Dynamixel SDK

### Basic Setup

1. **Clone and install**
   ```bash
   git clone --recursive https://github.com/Dieselmarble/UniBiDex.git
   cd UniBiDex
   conda create -n unibidex python=3.10
   conda activate unibidex
   pip install -e .
   pip install -e third_party/DynamixelSDK/python
   cd unibidex_client && pip install -e . && cd ..
   ```

3. **Run simulation**
   ```bash
   # Start MuJoCo simulation
   python sim/envs/xarm_mujoco_sim.py
   ```

### Installation

1. **Clone the repository**
   ```bash
   git clone --recursive https://github.com/Dieselmarble/UniBiDex.git
   cd UniBiDex
   ```

2. **Set up the environment**
   ```bash
   # Create conda environment
   conda create -n unibidex python=3.10
   conda activate unibidex
   
   # Install core dependencies
   pip install -r requirements.txt
   pip install -e .
   
   # Install Dynamixel SDK
   pip install -e third_party/DynamixelSDK/python
   
   # Install UniBiDex client for robot control
   cd unibidex_client
   pip install -e .
   cd ..
   ```

3. **Configure ROS2 (if using real robots)**
   ```bash
   # Source ROS2
   source /opt/ros/humble/setup.bash
   
   # Configure workspace if needed
   # (ROS2 integration is streamlined in the new framework)
   ```
---

## 🎮 Usage

### UniBiDex Teleoperation

1. **Configure UniBiDex devices**
   ```bash
   # Calibrate UniBiDex leader arms
   Left arm: python scripts/calib/unibidex_get_offset.py unibidex_left.yml
   Right arm: python scripts/calib/unibidex_get_offset.py unibidex_right.yml
   ```

2. **Start teleoperation**
   ```bash
   # Main teleoperation script with bimanual control
   python scripts/main.py
   ```

### Robot Control with UniBiDex Client

1. **Single arm control**
   ```bash
   cd unibidex_client
   python -m unibidex_client.nodes.single_arm_control
   ```

2. **Bimanual control**
   ```bash
   cd unibidex_client
   python -m unibidex_client.nodes.unibidex_controller --config configs/bimanual.yml
   ```

### MuJoCo Simulation

1. **Start simulation environment**
   ```bash
   python sim/envs/xarm_mujoco_sim.py
   ```

### Data Collection & Replay

1. **Collect demonstration data**
   ```bash
   cd unibidex_client
   python -m unibidex_client.nodes.data_recorder
   ```

2. **Replay demonstrations**
   ```bash
   # Dummy replay 
   python -m unibidex_client.nodes.data_playback --zarr_path ./demo_data.zarr --mode print
   
   # Real robot replay
   python -m unibidex_client.nodes.data_playback --zarr_path ./demo_data.zarr --mode real --config configs/bimanual.yml
   ```

---


## 📝 Citation

If you find UniBiDex useful in your research, please cite our paper:

```bibtex
@article{li2025unibidex,
    title   = {A Unified Teleoperation Framework for Robotic Bimanual Dexterous Manipulation},
    author  = {Zhongxuan Li and Zeliang Guo and Jun Hu and David Navarro-Alarcon and Jia Pan and Hongmin Wu and Peng Zhou},
    journal = {arXiv preprint arXiv:2501.XXXXX},
    year    = {2025}
}
```

---

## 📞 Support

- **Issues**: [GitHub Issues](https://github.com/Dieselmarble/UniBiDex/issues)
- **Discussions**: [GitHub Discussions](https://github.com/Dieselmarble/UniBiDex/discussions)
- **Email**: [zhongxuan.li@connect.hku.hk](mailto:zhongxuan.li@connect.hku.hk)

---

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## 🙏 Acknowledgments

- **Inspiration**: Built upon excellent open-source robotics frameworks
- **Hardware**: Thanks to UFactory for XArm7 platform support
- **Community**: Thanks to all contributors who made this project possible
- **Funding**: Supported by research grants and industry partnerships

---

<div align="center">

**[🌐 Visit our website](https://dieselmarble.github.io/UniBiDex/) | [📄 Read the paper](https://arxiv.org/abs/2501.XXXXX) | [🎥 Watch the video](https://youtu.be/XXXXX)**

</div>
