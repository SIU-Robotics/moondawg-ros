#!/bin/bash
set -e

# Run as root
if [ "$EUID" -ne 0 ]; then
    echo "This script must be run as root. Please run with 'sudo bash $0' or as root user."
    exit 1
fi

if mountpoint -q /var/www/html; then
    echo "Website appears to be already mounted at /var/www/html. Exiting."
    exit 0
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
WEBSITE_DIR="$PROJECT_DIR/website"

echo "Installing nginx web server..."
apt-get update
apt-get install -y nginx

rm -rf /var/www/html
mkdir -p /var/www/html
mount --bind "$WEBSITE_DIR" /var/www/html

echo "$WEBSITE_DIR /var/www/html none bind 0 0" >> /etc/fstab

systemctl enable nginx

LOCAL_IP=$(hostname -I | awk '{print $1}')

echo "The website should be available at http://$LOCAL_IP/"
