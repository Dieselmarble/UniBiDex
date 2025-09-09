# 🤖 UniBiDex: Unified Bimanual Dexterous Manipulation

<div align="center">

[![arXiv](https://img.shields.io/badge/arXiv-2501.XXXXX-b31b1b.svg)](https://arxiv.org/abs/2501.XXXXX)
[![Website](https://img.shields.io/badge/🌐-Website-blue.svg)](https://dieselmarble.github.io/UniBiDex/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python 3.8+](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/release/python-380/)
[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-green.svg)](https://docs.ros.org/en/humble/)

*A unified teleoperation framework for robotic bimanual dexterous manipulation*

> **🆕 Latest Updates**: The codebase has been restructured for better modularity and maintainability. Key improvements include unified device management, streamlined configuration, and enhanced simulation support.

[**Paper**](https://arxiv.org/abs/2501.XXXXX) | [**Website**](https://dieselmarble.github.io/UniBiDex/) | [**Video**](https://youtu.be/XXXXX) | [**Data**](#datasets)

</div>

---

## 🔥 Highlights

**UniBiDex** is a comprehensive teleoperation framework that enables **unified bimanual dexterous manipulation** with haptic leader-follower devices. Our system integrates cost-effective hardware design, advanced control algorithms, and simulation environments to achieve precise, safe, and efficient dual-arm manipulation.

### Key Features

- 🎮 **Unified Leader-Follower Control**: Custom-designed UniBiDex devices provide intuitive bimanual teleoperation
- 🤝 **Bimanual Coordination**: Advanced control framework handles dual-arm manipulation with coordinated motion  
- 🛡️ **Safety-Aware Control**: Collision avoidance, singularity handling, and workspace limits
- 🔄 **Null-Space Optimization**: Exploit arm redundancy for optimal bimanual coordination
- 📱 **Haptic Feedback**: Force feedback using motor current sensing for enhanced immersion
- 🐳 **Modular Design**: Extensible architecture for different robots and control algorithms

---

## 📁 Repository Structure

```
UniBiDex/
├── 🎮 assets/                        # 3D models and calibration files
│   ├── images/                       # Project images and logos
│   └── urdf/                         # Robot URDF models
├── 📜 scripts/                       # Main control scripts
│   ├── main.py                       # Core teleoperation script
│   ├── visualize_example.py          # Visualization utilities
│   └── calib/                        # Calibration scripts and configs
├── 🤖 unibidex_core/                 # Core UniBiDex framework
│   ├── agents/                       # Agent implementations
│   ├── dynamixel/                    # Dynamixel motor control
│   └── robots/                       # Robot interface modules
├── 🎛️ unibidex_client/               # Robot control client
│   ├── motion_control/               # Motion controllers (XArm7, grippers)
│   ├── nodes/                        # ROS2 nodes and utilities
│   ├── configs/                      # Configuration files
│   └── tests/                        # Test scripts
├── � sim/                           # Simulation environments
│   └── envs/                         # MuJoCo simulation environments
└── 🔧 third_party/                   # Third-party dependencies
    ├── DynamixelSDK/                 # Dynamixel SDK
    └── mujoco_menagerie/             # MuJoCo robot models
```

---

## 🚀 Quick Start

### Prerequisites

- **OS**: Ubuntu 20.04/22.04 (recommended)
- **Python**: ≥ 3.8
- **Hardware**: XArm7 robots, UniBiDex leader devices, Robotiq grippers (optional)
- **Dependencies**: Modern C++ compiler, OpenCV, NumPy

### Basic Setup

1. **Clone and install**
   ```bash
   git clone --recursive https://github.com/Dieselmarble/UniBiDex.git
   cd UniBiDex
   conda create -n unibidex python=3.8
   conda activate unibidex
   pip install -e .
   pip install -e third_party/DynamixelSDK/python
   cd unibidex_client && pip install -e . && cd ..
   ```

2. **Test installation**
   ```bash
   # Test with mock devices (no hardware required)
   python scripts/main.py --mock --verbose
   
   # Test robot control (requires XArm7)
   cd unibidex_client && python tests/test_xarm7.py
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
   conda create -n unibidex python=3.8
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

### Docker Setup (Recommended)

1. **Build Docker image**
   ```bash
   docker build . -t unibidex:latest
   ```

2. **Launch container**
   ```bash
   # Run with hardware access for real robots
   docker run -it --privileged --net=host \
     -v /dev:/dev \
     -v $PWD:/workspace \
     unibidex:latest
   ```

---

## 🎮 Usage

### UniBiDex Teleoperation

1. **Configure UniBiDex devices**
   ```bash
   # Calibrate UniBiDex leader arms
   python scripts/calib/unibidex_get_offset.py --port /dev/ttyUSB0
   ```

2. **Start teleoperation**
   ```bash
   # Main teleoperation script with bimanual control
   python scripts/main.py --bimanual --hz 100
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

2. **Run with visualization**
   ```bash
   python scripts/visualize_example.py
   ```

### Data Collection & Replay

1. **Collect demonstration data**
   ```bash
   cd unibidex_client
   python -m unibidex_client.nodes.data_recorder
   ```

2. **Replay demonstrations**
   ```bash
   # Simulation replay
   python -m unibidex_client.nodes.data_playback --zarr_path ./demo_data.zarr --mode print
   
   # Real robot replay
   python -m unibidex_client.nodes.data_playback --zarr_path ./demo_data.zarr --mode real --config configs/bimanual.yml
   ```

---

## 🛠️ Configuration

### Hardware Setup

#### Camera Configuration
```bash
# Set up camera rules for consistent device naming
cd unibidex_client
sudo cp camera_roles.rules /etc/udev/rules.d/99-camera-roles.rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Run camera view selector to assign camera roles
python -m unibidex_client.nodes.camera_view_selector
```

#### Robot Configuration
Edit configuration files in `unibidex_client/configs/`:
- `bimanual.yml`: Main bimanual control parameters
- Robot-specific configs for XArm7 and grippers

### Software Configuration

#### UniBiDex Calibration
```bash
# Calibrate left and right UniBiDex devices
python scripts/calib/unibidex_get_offset.py --port /dev/ttyUSB0 --side left
python scripts/calib/unibidex_get_offset.py --port /dev/ttyUSB1 --side right
```

#### Control Parameters
Key parameters in `unibidex_client/configs/`:
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

### Adding New Robot Types

1. **Implement robot interface** in `unibidex_client/motion_control/`
2. **Create configuration file** in `unibidex_client/configs/`  
3. **Add URDF models** to `assets/urdf/`
4. **Register robot** in control nodes

### Custom Control Algorithms

1. **Extend base controller** in `unibidex_client/motion_control/base.py`
2. **Implement custom IK/dynamics** following existing examples
3. **Configure parameters** in YAML config files

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

- **Inspiration**: Built upon excellent open-source robotics frameworks
- **Hardware**: Thanks to UFactory for XArm7 platform support
- **Community**: Thanks to all contributors who made this project possible
- **Funding**: Supported by research grants and industry partnerships

---

<div align="center">

**[🌐 Visit our website](https://dieselmarble.github.io/UniBiDex/) | [📄 Read the paper](https://arxiv.org/abs/2501.XXXXX) | [🎥 Watch the video](https://youtu.be/XXXXX)**

</div>
