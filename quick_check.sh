#!/bin/bash
echo "Quick Motor Check (with swapped wires)"
echo "======================================"
echo ""
echo "Listening for 10 seconds..."
echo "Power cycle your motor NOW!"
echo ""

timeout 10 candump can0

if [ $? -eq 124 ]; then
    echo ""
    echo "Timeout - no messages seen"
else
    echo ""
    echo "Got messages!"
fi

