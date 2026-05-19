# Hexmind 🕷️

An advanced, AI-powered Hexapod robotics project integrating custom YOLO vision models and Arduino-based kinematics.

---

## 📸 The Hexapod

<!-- 
  REPLACE the link below with your actual image path or URL once you have a picture of your robot!
  Example: ![Hexapod Robot](images/my_hexapod.jpg)
-->
<div align="center">
  <img src="https://via.placeholder.com/800x400?text=Your+Hexapod+Robot+Picture+Here" alt="Hexapod Robot" width="100%" />
</div>

---

## 🚀 Project Overview

Hexmind combines the mechanical agility of a hexapod with the intelligence of modern computer vision. The system is split into two primary components:

1. **AI Brain (`hexapod_ai/`)**: 
   - A Python-based intelligence module running custom YOLOv8 object detection.
   - Depth estimation and real-time processing capabilities (`depth_estimator.py`, `hexapod_brain.py`).
   - Custom training pipelines and dashboard GUI for monitoring.

2. **Hardware Control (`Hexapod_Code/`)**:
   - Arduino (`.ino`) scripts handling the low-level transmitter and receiver logic.
   - Precise multi-servo coordination to translate AI commands into physical locomotion.

## 📁 Repository Structure

```text
Hexamind/
├── Hexapod_Code/            # Arduino sketches for Tx/Rx communication
│   ├── hexapod_receiver.ino
│   └── hexapod_transmitter.ino
├── hexapod_ai/              # Python vision and brain scripts
│   ├── hexapod_brain.py     # Main AI logic controller
│   ├── depth_estimator.py   # Monocular depth estimation
│   ├── gui_dashboard.py     # User interface for monitoring
│   └── train_custom.py      # Custom model training scripts
└── README.md
```

## 🛠️ Getting Started

### Prerequisites

- **Hardware**: Arduino microcontrollers, servo controllers, and a hexapod chassis.
- **Software**: Python 3.8+, Arduino IDE.

### AI Setup

Navigate to the `hexapod_ai` directory and install the necessary requirements:

```bash
cd hexapod_ai
pip install -r requirements.txt
```

### Microcontroller Setup

Flash the `hexapod_receiver.ino` to the hexapod's onboard controller and `hexapod_transmitter.ino` to your remote control module using the Arduino IDE.

## 🧠 Model Training

The `hexapod_ai` module supports training custom YOLOv8 models. Place your datasets inside the `hexapod_ai/dataset/` directory (these are excluded from Git to save space) and run the `train_custom.py` script.

---
*Created by [Divyanshu-2907](https://github.com/Divyanshu-2907)*
