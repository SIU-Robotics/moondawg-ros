#!/bin/bash

set -e

# Check if script is run as root

if [ "$EUID" -ne 0 ]; then
    echo "Please run as root (use sudo)"
    exit 1
fi

# Check if already installed by seeing if website is already mounted
if mountpoint -q /var/www/html; then
    echo "Website appears to be already mounted at /var/www/html. Exiting."
    exit 0
fi

# Get the directory of this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Get the project root directory (one level up from scripts)
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
# Website directory
WEBSITE_DIR="$PROJECT_DIR/website"

echo "Installing nginx web server..."
apt-get update
apt-get install -y nginx

# Link /var/www/html to the website directory
rm -rf /var/www/html
mkdir -p /var/www/html
mount --bind "$WEBSITE_DIR" /var/www/html

echo "$WEBSITE_DIR /var/www/html none bind 0 0" >> /etc/fstab

# Ensure nginx can access the website directory
chown -R www-data:www-data "$WEBSITE_DIR"

systemctl enable nginx

# Get the local IP address
LOCAL_IP=$(hostname -I | awk '{print $1}')

echo "The website should be available at http://$LOCAL_IP/"
