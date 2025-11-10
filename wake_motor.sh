#!/bin/bash
echo "Sending wake/enable commands..."

# Try common motor IDs with enable command
for id in 101 107 127; do
    hex_id=$(printf "%03X" $id)
    echo "Trying motor ID $id (0x$hex_id)..."
    
    # Enable command
    cansend can0 ${hex_id}#FFFFFFFFFFFFFFFC
    sleep 0.5
    
    # Read position command
    cansend can0 ${hex_id}#9200000000000000
    sleep 0.5
done

echo ""
echo "Commands sent. Check candump for responses!"
