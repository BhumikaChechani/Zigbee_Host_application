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

# Add current non-root user to the serial communication group
# (SUDO_USER holds the username of the user who invoked sudo)
REAL_USER=${SUDO_USER:-$USER}

if [ "$REAL_USER" != "root" ]; then
    echo "[+] Configuring serial port permissions for user: $REAL_USER"
    if getent group dialout > /dev/null; then
        usermod -aG dialout "$REAL_USER"
        echo "[+] Added $REAL_USER to the 'dialout' group."
    elif getent group uucp > /dev/null; then
        usermod -aG uucp "$REAL_USER"
        echo "[+] Added $REAL_USER to the 'uucp' group."
    fi
    echo "=========================================================="
    echo " SUCCESS: Installation complete!"
    echo " ⚠️  IMPORTANT: Please LOG OUT and LOG BACK IN (or reboot) for"
    echo " group permissions to take effect so you can access the serial port without sudo."
    echo "=========================================================="
else
    echo "=========================================================="
    echo " SUCCESS: Installation complete!"
    echo " Note: Since you ran this script directly as root, make sure your"
    echo " standard user account belongs to the 'dialout' or 'uucp' group."
    echo "=========================================================="
fi
