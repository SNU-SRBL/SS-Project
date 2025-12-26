# SS-Project: Hall Sensor Research

A research project for Hall Sensor developed by Jaehong Cho in 2025. This project includes sensor data acquisition, calibration, training, and real-time force visualization.

## Installation

### Step 1: Setup Conda Environment

Create a conda environment using the provided `environment.yml` file:

```bash
conda env create -f environment.yml
conda activate SS
```

### Step 2: Install PyTorch

Install PyTorch based on your system configuration. Visit [PyTorch official installation guide](https://pytorch.org/get-started/locally/) for the latest instructions.

**Example command for PyTorch 2.6.0 & CUDA 11.8:**
```bash
pip install torch==2.6.0 torchvision==0.21.0 torchaudio==2.6.0 --index-url https://download.pytorch.org/whl/cu118
```

**For CPU-only:**
```bash
pip install torch==2.6.0 torchvision==0.21.0 torchaudio==2.6.0
```

> **Note:** Choose the appropriate CUDA version for your system or use CPU-only installation.

## Hardware Setup

### Step 1: Connect Sensors

- **Hall Sensor (sensor_srbl)**: Connect to `/dev/ttyUSB0`
- **Commercial FT Sensor**: Connect to `/dev/ttyUSB1` (optional, only required for calibration)

### Step 2: Verify USB Connections

Check USB device connections using:
```bash
dmesg | grep tty
```

## Usage

### Step 1: Test Hall Sensor

Test the Hall sensor to ensure proper connection and functionality:

```bash
python sensor/sensor_srbl.py
```

### Step 2: Test Commercial FT Sensor (Optional)

If you have a commercial FT sensor connected, test it:

```bash
python sensor/sensor_FT.py
```

### Step 3: Collect Calibration Data

Collect sensor data for calibration training:

```bash
python calibration/logger.py
```

This will generate a CSV file with synchronized data from both sensors.

### Step 4: Train Calibration Model

Train the neural network model for force prediction:

```bash
python calibration/train.py
```

This will create trained model weights and scalers in the `calibration/` directory.

### Step 5: Visualize Sensor with Force Mapping

Run the real-time force visualization using VTK:

```bash
python run_vtk.py
```

You can optionally specify an STL file for the sensor model:

```bash
python run_vtk.py sensor.STL
```

## Project Structure

```
SS-Project/
├── sensor/                    # Sensor interface modules
│   ├── sensor_srbl.py        # Hall sensor driver
│   └── sensor_FT.py          # Commercial FT sensor driver
├── calibration/              # Calibration and training scripts
│   ├── logger.py             # Data logging script
│   ├── train.py              # Model training script
│   └── sensor_data_log_*.csv # Generated calibration data
├── run_vtk.py                # Real-time force visualization
└── environment.yml           # Conda environment file
```

## License

Research Project - Soft Robotics & Bionics Lab, Seoul National University
