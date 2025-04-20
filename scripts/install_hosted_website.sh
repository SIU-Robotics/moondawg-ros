#!/bin/bash

# Check if script is run as root
if [ "$EUID" -ne 0 ]; then
    echo "Please run as root (use sudo)"
    exit 1
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

echo "Creating symbolic link from /var/www/html to website directory..."

rm -rf /var/www/html

# Create the symbolic link
ln -sf "$WEBSITE_DIR" /var/www/html

# Ensure nginx can access the website directory
chown -R www-data:www-data "$WEBSITE_DIR"

# Test nginx configuration
nginx -t

# Restart nginx to apply changes
systemctl restart nginx
systemctl enable nginx

# Get the local IP address
LOCAL_IP=$(hostname -I | awk '{print $1}')

echo "Website installation complete!"
echo "The MoonDawg ROS website is now available at http://$LOCAL_IP/"
