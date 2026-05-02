#!/bin/bash
set -e

echo "Setting up permissions for ISM330 IMU..."

# 1. Create udev rule to allow access to iio:device* for dialout/plugdev users
# The ISM330 on this board usually shows up as an IIO device.
# We'll make it accessible to everyone or the current group.


# Helper script path
HELPER_SCRIPT="/usr/local/bin/manage_ism330.sh"

echo "1. Creating runtime helper script at $HELPER_SCRIPT..."
sudo bash -c "cat > $HELPER_SCRIPT" <<'EOF'
#!/bin/bash
set -e

# Load module
modprobe iio-trig-hrtimer || echo "Failed to load iio-trig-hrtimer"

# Mount configfs
if ! mount | grep -q "configfs"; then
    mkdir -p /sys/kernel/config
    mount -t configfs none /sys/kernel/config
fi

# Set permissions for configfs triggers (creates dir if needed)
mkdir -p /sys/kernel/config/iio/triggers
chmod 0777 /sys/kernel/config/iio/triggers

# Instantiate I2C device if not present
# Bus 1, Address 0x6a
if [ -d "/sys/bus/i2c/devices/i2c-1" ]; then
    if [ ! -d "/sys/bus/i2c/devices/1-006a" ]; then
        echo "Instantiating ISM330..."
        echo ism330dhcx 0x6a > /sys/bus/i2c/devices/i2c-1/new_device || echo "Failed to instantiate (busy?)"
    fi
fi
EOF
sudo chmod +x $HELPER_SCRIPT

echo "2. Installing systemd service..."
SERVICE_FILE="/etc/systemd/system/ism330-setup.service"
sudo bash -c "cat > $SERVICE_FILE" <<EOF
[Unit]
Description=Setup ISM330 IMU permissions and triggers
After=network.target

[Service]
Type=oneshot
ExecStart=$HELPER_SCRIPT
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

echo "3. Updating udev rules..."
RULE_FILE="/etc/udev/rules.d/99-ism330.rules"
if [ ! -d "/etc/udev/rules.d" ]; then
    sudo mkdir -p /etc/udev/rules.d
fi

# RUN script to chmod the device attributes when it appears
sudo bash -c "cat > $RULE_FILE" <<'EOF'
ACTION=="add", SUBSYSTEM=="iio", KERNEL=="iio:device*", MODE="0666", RUN+="/bin/sh -c 'chmod -R 0777 /sys/bus/iio/devices/%k'"
EOF
# sudo chmod -R 0777 /sys/bus/iio/devices/iio\:device0
# sudo chmod -R 0777 /sys/bus/iio/devices/iio\:device1
echo "4. Reloading and applying..."
sudo systemctl daemon-reload
sudo systemctl enable ism330-setup.service
sudo systemctl start ism330-setup.service

if command -v udevadm >/dev/null 2>&1; then
    sudo udevadm control --reload-rules
    sudo udevadm trigger
fi

echo "Done! Configuration is now persistent across reboots."



echo "Done! You usually don't need to reboot, but if it doesn't work,"
echo "try unplugging/replugging the device or rebooting."
echo "Verify with: ls -l /dev/iio:device*"
