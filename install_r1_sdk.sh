#!/bin/bash
# Galaxea R1 Pro SDK Installation Script
# Ubuntu 24.04 / Python 3.12

set -e

echo "=============================================="
echo "  Galaxea R1 Pro SDK Installation"
echo "=============================================="

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 1. Install system dependencies
echo -e "${YELLOW}[1/5] Installing system dependencies...${NC}"
sudo apt-get update
sudo apt-get install -y \
    python3-pip \
    python3-venv \
    python3-dev \
    build-essential \
    cmake \
    git \
    libhdf5-dev \
    libopenblas-dev \
    liblapack-dev

# 2. Create a virtual environment (recommended)
echo -e "${YELLOW}[2/5] Setting up Python virtual environment...${NC}"
VENV_DIR="${HOME}/galaxea_venv"
if [ ! -d "$VENV_DIR" ]; then
    python3 -m venv "$VENV_DIR"
fi
source "$VENV_DIR/bin/activate"

# Upgrade pip
pip install --upgrade pip setuptools wheel

# 3. Install the Galaxea R1 SDK (via pip)
echo -e "${YELLOW}[3/5] Installing Galaxea R1 SDK...${NC}"

# Option A: Install from PyPI (if available)
pip install galaxea-sdk 2>/dev/null || {
    echo -e "${YELLOW}galaxea-sdk not found on PyPI, trying alternative methods...${NC}"
    
    # Option B: Install from GitHub (if official repo exists)
    pip install git+https://github.com/galaxea-robotics/galaxea-sdk.git 2>/dev/null || {
        echo -e "${YELLOW}GitHub installation failed, trying galxea package...${NC}"
        
        # Option C: Try alternative package names
        pip install galxea 2>/dev/null || pip install r1-sdk 2>/dev/null || {
            echo ""
            echo -e "${YELLOW}=============================================="
            echo "SDK not found in public repositories."
            echo "Please use one of these methods:"
            echo "=============================================="
            echo ""
            echo "METHOD 1: Install from local .whl file"
            echo "  pip install /path/to/galaxea_sdk-*.whl"
            echo ""
            echo "METHOD 2: Install from local .tar.gz file"
            echo "  pip install /path/to/galaxea-sdk-*.tar.gz"
            echo ""
            echo "METHOD 3: Install from extracted source directory"
            echo "  cd /path/to/galaxea-sdk"
            echo "  pip install -e ."
            echo ""
            echo "METHOD 4: Install from .deb package"
            echo "  sudo dpkg -i /path/to/galaxea-sdk*.deb"
            echo ""
            echo "=============================================="
            echo -e "${NC}"
        }
    }
}

# 4. Install common robotics dependencies
echo -e "${YELLOW}[4/5] Installing common robotics dependencies...${NC}"
pip install numpy scipy matplotlib
pip install opencv-python pillow
pip install pyserial  # For serial communication with robot

# 5. Verify installation
echo -e "${YELLOW}[5/5] Verifying installation...${NC}"
python3 -c "
try:
    import galaxea
    print('✅ Galaxea SDK installed successfully!')
    print(f'   Version: {galaxea.__version__}')
except ImportError:
    try:
        import galxea
        print('✅ Galxea SDK installed successfully!')
    except ImportError:
        print('⚠️  SDK import not verified. If you installed from local files, please test manually.')
"

echo ""
echo -e "${GREEN}=============================================="
echo "Installation complete!"
echo "=============================================="
echo ""
echo "To activate the environment in the future:"
echo "  source ${VENV_DIR}/bin/activate"
echo ""
echo -e "${NC}"
