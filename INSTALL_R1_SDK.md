# Galaxea R1 Pro SDK Installation Guide

This guide explains how to install the Galaxea R1 Pro SDK in your container.

## Prerequisites

- Ubuntu 22.04/24.04
- Python 3.10+
- pip3

## Quick Install (Automated Script)

```bash
chmod +x install_r1_sdk.sh
./install_r1_sdk.sh
```

## Manual Installation Methods

### Method 1: Install from PyPI (if publicly available)

```bash
pip install galaxea-sdk
```

### Method 2: Install from Local Package Files

If you obtained the SDK as downloadable files from Galaxea:

**For `.whl` files:**
```bash
# Copy your .whl file to the container first
pip install galaxea_sdk-X.X.X-py3-none-any.whl
```

**For `.tar.gz` files:**
```bash
pip install galaxea-sdk-X.X.X.tar.gz
```

**For `.deb` packages:**
```bash
sudo dpkg -i galaxea-sdk_X.X.X_amd64.deb
sudo apt-get install -f  # Fix any dependency issues
```

### Method 3: Install from Source

If you have the SDK source code:

```bash
# Clone or extract the SDK source
cd galaxea-sdk/

# Install in development mode
pip install -e .

# Or standard install
pip install .
```

### Method 4: Install from Private Git Repository

If Galaxea provided access to a private repository:

```bash
# Using HTTPS with token
pip install git+https://<token>@github.com/galaxea-robotics/galaxea-sdk.git

# Using SSH
pip install git+ssh://git@github.com/galaxea-robotics/galaxea-sdk.git
```

## Copying Files to Your Container

If your SDK files are on your local machine, copy them to the container:

```bash
# From your host machine (outside the container)
docker cp /path/to/galaxea-sdk.whl <container_id>:/workspace/

# Then inside the container
pip install /workspace/galaxea-sdk.whl
```

## ROS 2 Integration (Optional)

If you need ROS 2 integration with the R1 Pro:

```bash
# Install ROS 2 Humble/Jazzy packages
sudo apt-get install -y ros-${ROS_DISTRO}-ros-base

# Install R1 ROS packages if available
# These might be provided separately by Galaxea
```

## Verification

Test your installation:

```python
import galaxea
from galaxea import R1Robot

# Initialize robot connection
robot = R1Robot()
print(f"SDK Version: {galaxea.__version__}")
```

## Common Dependencies

The SDK typically requires these packages:

```bash
pip install numpy scipy matplotlib
pip install opencv-python pillow
pip install pyserial  # Serial communication
pip install zmq       # Network communication
pip install protobuf  # Message serialization
```

## Troubleshooting

### Import Error
```
ModuleNotFoundError: No module named 'galaxea'
```
→ Ensure you activated the correct virtual environment or installed to the right Python.

### Missing System Libraries
```
ImportError: libhdf5.so.10: cannot open shared object file
```
→ Install missing system libraries:
```bash
sudo apt-get install libhdf5-dev
```

### Permission Denied (Serial Port)
```
PermissionError: [Errno 13] Permission denied: '/dev/ttyUSB0'
```
→ Add user to dialout group:
```bash
sudo usermod -a -G dialout $USER
# Then logout and login again
```

## Support

For SDK-specific issues, contact Galaxea support or refer to:
- Official Galaxea documentation
- SDK release notes provided with your package
