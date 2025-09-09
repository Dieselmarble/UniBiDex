# 🤖 UniBiDex: Unified Bimanual Dexterous Manipulation

<div align="center">

[![arXiv](https://img.shields.io/badge/arXiv-2501.XXXXX-b31b1b.svg)](https://arxiv.org/abs/2501.XXXXX)
[![Website](https://img.shields.io/badge/🌐-Website-blue.svg)](https://dieselmarble.github.io/UniBiDex/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python 3.8+](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/release/python-380/)
[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-green.svg)](https://docs.ros.org/en/humble/)

*A unified teleoperation framework for robotic bimanual dexterous manipulation*

[**Paper**](https://arxiv.org/abs/2501.XXXXX) | [**Website**](https://dieselmarble.github.io/UniBiDex/) | [**Video**](https://youtu.be/XXXXX) | [**Data**](#datasets)

</div>

---

## 🔥 Highlights

**UniBiDex** is a comprehensive teleoperation framework that enables **unified bimanual dexterous manipulation** across heterogeneous input devices. Our system integrates VR headsets, leader-follower arms, and advanced control algorithms to achieve precise, safe, and efficient dual-arm manipulation.

### Key Features

- 🎮 **Universal Device Support**: Seamlessly integrate VR headsets (Meta Quest 3) and leader-follower arms (UniBiDex)
- 🤝 **Unified Control Framework**: Single control stack handles multiple input modalities with consistent behavior  
- 🛡️ **Safety-Aware Control**: Advanced inverse kinematics with collision avoidance and singularity handling
- 🔄 **Null-Space Optimization**: Exploit arm redundancy for optimal bimanual coordination
- 📱 **Haptic Feedback**: Cost-effective force feedback using motor current sensing
- 🐳 **Production Ready**: Dockerized deployment with comprehensive ROS2 integration

---

## 📁 Repository Structure

```
UniBiDex/
├── 📦 banana_teleoperation/          # VR teleoperation pipeline
│   ├── banana_teleop_client/         # Robot control client
│   ├── banana_teleop_server/         # VR pose streaming server
│   └── assets/                       # 3D models and calibration files
├── 🤖 unibidex/                      # Core UniBiDex framework
│   ├── unibidex_core/                        # UniBiDex leader-follower integration
│   ├── ros2/                         # ROS2 nodes and controllers
│   ├── scripts/                      # Control scripts and utilities
│   └── experiments/                  # Experimental configurations
├── 🤏 gripper/                       # Gripper control modules
├── 📄 docs/                          # Documentation
└── 🌐 website/                       # Project website (gh-pages)
```

---

## 🚀 Quick Start

### Prerequisites

- **OS**: Ubuntu 20.04/22.04 (recommended)
- **Python**: ≥ 3.8
- **ROS2**: Humble
- **Hardware**: Meta Quest 3, xArm7 robots, UniBiDex devices (optional)

### Installation

1. **Clone the repository**
   ```bash
   git clone --recursive https://github.com/Dieselmarble/UniBiDex.git
   cd UniBiDex
   ```

2. **Set up the environment**
   ```bash
   # Create conda environment
   conda create -n unibidex python=3.8
   conda activate unibidex
   
   # Install dependencies
   cd unibidex
   pip install -r requirements.txt
   pip install -e .
   pip install -e third_party/DynamixelSDK/python
   ```

3. **Configure ROS2 (if using real robots)**
   ```bash
   # Source ROS2
   source /opt/ros/humble/setup.bash
   
   # Build ROS2 workspace
   cd ros2
   colcon build
   source install/setup.bash
   ```

### Docker Setup (Recommended)

1. **Build Docker image**
   ```bash
   cd unibidex
   docker build . -t unibidex:latest
   ```

2. **Launch container**
   ```bash
   python scripts/launch.py
   ```

---

## 🎮 Usage

### VR Teleoperation

1. **Start the VR server** (Quest 3)
   ```bash
   cd banana_teleoperation/banana_teleop_server
   python main.py --config configs/vr_config/quest_config.yml
   ```

2. **Launch robot client**
   ```bash
   cd banana_teleoperation/banana_teleop_client
   python main.py --config configs/bimanual.yml
   ```

### Leader-Follower Teleoperation

1. **Configure UniBiDex devices** (see [UniBiDex Setup](#unibidex-setup))
   ```bash
   cd unibidex
   python scripts/configure_unibidex.py --port /dev/ttyUSB0
   ```

2. **Start teleoperation**
   ```bash
   python scripts/teleop_unibidex.py --config experiments/bimanual_unibidex.py
   ```

### Data Collection & Replay

1. **Collect demonstration data**
   ```bash
   python scripts/collect_data.py --output ./demo_data.zarr
   ```

2. **Replay demonstrations**
   ```bash
   # Simulation replay
   python scripts/data_playback.py --zarr_path ./demo_data.zarr --mode sim
   
   # Real robot replay
   python scripts/data_playback.py --zarr_path ./demo_data.zarr --mode real --config configs/bimanual.yml
   ```

---

## 🛠️ Configuration

### Hardware Setup

#### Camera Configuration
```bash
# Set up camera rules for consistent device naming
sudo cp banana_teleoperation/assets/camera_roles.rules /etc/udev/rules.d/99-camera-roles.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

#### Robot Configuration
Edit configuration files in `banana_teleoperation/banana_teleop_client/configs/`:
- `bimanual.yml`: Main bimanual control parameters
- `cameras.yml`: Camera settings and calibration
- Robot-specific configs in `banana_teleop_server/configs/robot_config/`

### Software Configuration

#### ROS2 Parameters
Key parameters in `unibidex/ros2/src/*/config/`:
- Controller gains and limits
- Safety constraints
- Hardware interface settings

---

## � Experiments

### Kitchen Tidying Task
```bash
cd unibidex/experiments
python kitchen_tidying.py --mode vr  # VR mode
python kitchen_tidying.py --mode unibidex  # UniBiDex mode
```

### Custom Tasks
1. Create experiment configuration in `experiments/`
2. Define task parameters and constraints
3. Run with either VR or UniBiDex input

---

## 📊 Datasets

We provide demonstration datasets for common bimanual manipulation tasks:

- **Kitchen Tidying**: 40 demonstrations across VR and UniBiDex modalities
- **Assembly Tasks**: Complex multi-step manipulation sequences
- **Contact-Rich Tasks**: Tasks requiring precise force control

Download datasets from [our website](https://dieselmarble.github.io/UniBiDex/) or generate your own using our data collection pipeline.

---

## 🧩 Extending UniBiDex

### Adding New Input Devices

1. **Implement device interface** in `unibidex/devices/`
2. **Create configuration file** with device parameters
3. **Register device** in the main control loop

### Custom Robot Integration

1. **Add robot model** to `banana_teleoperation/assets/`
2. **Configure kinematics** in robot config files  
3. **Implement robot interface** following existing examples

---

## 📝 Citation

If you find UniBiDex useful in your research, please cite:

```bibtex
@article{li2025unibidex,
    title   = {UniBiDex: A Unified Teleoperation Framework for Robotic Bimanual Dexterous Manipulation},
    author  = {Zhongxuan Li and Zeliang Guo and Jun Hu and David Navarro-Alarcon and Jia Pan and Hongmin Wu and Peng Zhou},
    journal = {2025 IEEE International Conference on Robotics and Biomimetics},
    year    = {2025}
}
```

---

## 🤝 Contributing

We welcome contributions! Please see [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

### Development Setup

1. **Install development dependencies**
   ```bash
   pip install -r requirements_dev.txt
   pre-commit install
   ```

2. **Run tests**
   ```bash
   pytest tests/
   ```

3. **Code formatting**
   ```bash
   black . && isort . && flake8
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

- **UniBiDex**: Built upon the excellent [GELLO framework](https://github.com/wuphilipp/gello_software)
- **ROS2**: Powered by the Robot Operating System 2
- **Contributors**: Thanks to all contributors who made this project possible

---

<div align="center">

**[🌐 Visit our website](https://dieselmarble.github.io/UniBiDex/) | [📄 Read the paper](https://arxiv.org/abs/2501.XXXXX) | [🎥 Watch the video](https://youtu.be/XXXXX)**

</div>
