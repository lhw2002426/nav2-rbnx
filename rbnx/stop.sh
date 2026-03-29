#!/bin/bash

# Kill by process name as fallback
pkill -f "navigation_launch" || true

exit 0