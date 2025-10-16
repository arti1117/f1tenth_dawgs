#!bin/bash

# Parameter
RULE_FILE="99-iarhs_udev.rules"

# Copying rule file
echo "Copying $RULE_FILE to /etc/udev/rules.d/"
sudo cp "$RULE_FILE" /etc/udev/rules.d/

# Reload and Trigger new rule
echo "Reloading udev rules..."
sudo udevadm control --reload

echo "Triggering udev..."
sudo udevadm trigger

echo "Udev rule applied successfully!"
