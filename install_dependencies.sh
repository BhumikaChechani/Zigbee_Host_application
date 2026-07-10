#!/bin/bash

# ZNP MT Host Controller - Dependency Installer Script
# This script installs the GCC compiler, Make utility, and serial diagnostic tools.

set -e

echo "=========================================================="
echo " Starting ZNP MT Host Controller Dependency Installation"
echo "=========================================================="

# Check if running as root
if [ "$EUID" -ne 0 ]; then
  echo "❌ Please run this script with sudo or as root:"
  echo "  sudo ./install_dependencies.sh"
  exit 1
fi

# Detect Package Manager and Install Dependencies
if command -v apt-get &> /dev/null; then
    echo "[+] Debian/Ubuntu system detected. Installing dependencies via apt..."
    apt-get update
    apt-get install -y build-essential usbutils picocom
elif command -v dnf &> /dev/null; then
    echo "[+] Fedora/RHEL/Rocky system detected. Installing dependencies via dnf..."
    dnf groupinstall -y "Development Tools"
    dnf install -y usbutils picocom
elif command -v yum &> /dev/null; then
    echo "[+] CentOS system detected. Installing dependencies via yum..."
    yum groupinstall -y "Development Tools"
    yum install -y usbutils picocom
else
    echo "[-] Unsupported package manager. Please manually install: gcc, make, glibc headers, usbutils, and picocom."
    exit 1
fi

# Configure system-wide udev rules for all ttyACM and ttyUSB serial devices
echo "[+] Configuring udev rules for all serial ports (/dev/ttyACM* and /dev/ttyUSB*)..."
cat << 'EOF' > /etc/udev/rules.d/99-serial.rules
KERNEL=="ttyACM[0-9]*", MODE="0666"
KERNEL=="ttyUSB[0-9]*", MODE="0666"
EOF

# Reload udev rules to apply changes immediately
echo "[+] Reloading udev rules..."
udevadm control --reload-rules
udevadm trigger

# Add current non-root user to the serial communication group as backup
# (SUDO_USER holds the username of the user who invoked sudo)
REAL_USER=${SUDO_USER:-$USER}

if [ "$REAL_USER" != "root" ]; then
    echo "[+] Configuring group permissions for user: $REAL_USER"
    if getent group dialout > /dev/null; then
        usermod -aG dialout "$REAL_USER"
        echo "[+] Added $REAL_USER to the 'dialout' group."
    elif getent group uucp > /dev/null; then
        usermod -aG uucp "$REAL_USER"
        echo "[+] Added $REAL_USER to the 'uucp' group."
    fi
fi

echo "=========================================================="
echo " SUCCESS: Installation & Configuration complete!"
echo " 🔌 IMPORTANT: If the Zigbee Coordinator is plugged in,"
echo "    please UNPLUG and PLUG it back in to apply the rules."
echo "    This allows running the app without sudo immediately!"
echo "=========================================================="
