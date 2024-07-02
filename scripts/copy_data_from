#!/bin/bash

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <remote_ip_address>"
    exit 1
fi

REMOTE_IP=$1
REMOTE_DIR="/home/administrator/experiment_files/*"
LOCAL_DIR="$HOME/experiment_files/"

scp -r "administrator@$REMOTE_IP:$REMOTE_DIR" "$LOCAL_DIR"

if [ $? -eq 0 ]; then
    echo "Files copied successfully."
else
    echo "An error occurred while copying files."
fi
