# Bluehand-IsaacLab
A four-finger hand simulator for grasping based on IsaacLab

## Overview
......dummy.......

## Setup
The demo was tested on Ubuntu 22.04 with an Nvidia RTX 3070 Ti.
- **Driver version**: 525.85.12
- **CUDA**: 11.8
- **IsaacSim**: 4.2.0

## Features

### 1. Hand Gesture GUI Implementation
A simple GUI-based control for hand gestures.

#### Keyboard Shortcuts:
- **F**: Fully extend all fingers (Open Hand)
- **P**: Fully close all fingers (Power Grip)
- **1**: Toggle extension/contraction of the first finger
- **2**: Toggle extension/contraction of the second finger
- **3**: Toggle extension/contraction of the third finger
- **4**: Toggle extension/contraction of the fourth finger

### 2. Sensor & RGB-Based In-Hand Value Measurement and Classification
- Captures in-hand sensor data combined with RGB images.
- Processes sensor data for classification of grasping status.

### 3. Reinforcement Learning for Grasping Policy
- Implements reinforcement learning (RL) to train grasping strategies.
- Utilizes IsaacGym for simulation-based RL training.

## Usage
### Installation
Ensure you have the required dependencies installed:
```bash
# Install required packages
pip install -r requirements.txt
```

### Running the Hand Gesture GUI
```bash
python run_hand_gui.py
```

### Running Sensor-Based Classification
```bash
python run_sensor_classification.py
```

### Running RL Training
```bash
python run_rl_training.py
```

## Acknowledgments

## License
This project is licensed under the MIT License. See `LICENSE` for details.


