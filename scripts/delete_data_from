#!/bin/bash

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <remote_ip_address>"
    exit 1
fi

REMOTE_IP=$1
REMOTE_DIR="/home/administrator/experiment_files"

ssh "administrator@$REMOTE_IP" "rm -r $REMOTE_DIR"

if [ $? -eq 0 ]; then
    echo "All files in $REMOTE_DIR on $REMOTE_IP have been deleted successfully."
else
    echo "An error occurred while deleting files."
fi
