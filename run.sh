#!/bin/bash
# Helper script to run the container

# Ensure X11 socket permissions
xhost +local:docker

# Export display
export DISPLAY=:0

# Run docker-compose
docker-compose up -d

# Show logs
echo "Container started. Viewing logs (Ctrl+C to exit)..."
docker-compose logs -f
