#!/usr/bin/env bash
set -euo pipefail

##################################################################################################################
# This script sets up the conda environment for the Python flavour of unitree_mujoco
# (simulate_python). It assumes you have set up the base HIVE environment described in
# https://github.com/Hive-Robots/core-environment, which clones unitree_sdk2_python into
# ~/repos/unitree_sdk2_python and provides Miniconda + CycloneDDS.
#
# The C++ simulator (./simulate) is not handled here — it requires unitree_sdk2 installed
# under /opt/unitree_robotics and a system-wide MuJoCo build; see readme.md.
##################################################################################################################

# ====== Configuration ======
GREEN="\033[1;32m"; BLUE="\033[1;34m"; YELLOW="\033[1;33m"; RED="\033[1;31m"; NC="\033[0m"
ENV_NAME="unitree_mujoco"
PYTHON_VERSION="3.10"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$SCRIPT_DIR"
UNITREE_SDK_DIR="${UNITREE_SDK_DIR:-$HOME/repos/unitree_sdk2_python}"

log_step() {
    echo -e "\n${BLUE}[ step ]${NC} $1"
    echo "----------------------------------------------------"
}
log_ok()   { echo -e "${GREEN}[ok]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[warn]${NC} $1"; }
log_err()  { echo -e "${RED}[error]${NC} $1"; exit 1; }

# ====== Pre-flight Check ======
if [ ! -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]; then
    log_err "Conda activation script not found at $HOME/miniconda3/etc/profile.d/conda.sh. Run the core-environment base setup first."
fi

if [ ! -d "$REPO_DIR/simulate_python" ]; then
    log_err "This script must live in the unitree_mujoco repo root (missing simulate_python/)."
fi

if [ ! -d "$UNITREE_SDK_DIR/.git" ]; then
    log_err "unitree_sdk2_python repository not found at $UNITREE_SDK_DIR. Run the core-environment base setup first, or set UNITREE_SDK_DIR."
fi

source "$HOME/miniconda3/etc/profile.d/conda.sh"

# ====== 1. Prepare Environment ======
log_step "Preparing $ENV_NAME (Py$PYTHON_VERSION)"
if conda info --envs | awk '{print $1}' | grep -qx "$ENV_NAME"; then
    log_ok "Environment exists."
else
    conda create -n "$ENV_NAME" "python=$PYTHON_VERSION" pip -y
fi
conda activate "$ENV_NAME"

# ====== 2. Python Dependencies ======
log_step "Installing simulator Python dependencies"
pip install \
    "mujoco" \
    "numpy<2" \
    "pygame" \
    "pyyaml"

# ====== 3. Unitree SDK ======
log_step "Installing unitree_sdk2_python (editable)"
# CycloneDDS 0.10.x matches the build produced by core-environment's 01_setup_base.sh.
pip install --no-deps "cyclonedds==0.10.2"
pip install -e "$UNITREE_SDK_DIR"

# ====== 4. Sanity Check ======
log_step "Verifying core imports"
python - <<'EOF'
import mujoco
import numpy as np
import pygame
from unitree_sdk2py.core.channel import ChannelFactoryInitialize  # noqa: F401

print(f"mujoco: {mujoco.__version__}")
print(f"numpy:  {np.__version__}")
print(f"pygame: {pygame.version.ver}")
print("unitree_sdk2py import OK")
EOF

log_ok "unitree_mujoco Python setup complete."
log_ok "Activate with: conda activate $ENV_NAME"
log_ok "Then run:      python simulate_python/unitree_mujoco.py"
