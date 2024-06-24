#!/bin/bash

# Start the wandb sweep
sweep_id=$(wandb sweep sweep.yaml)
echo "Sweep ID: $sweep_id"

# Start the wandb agent with the sweep ID
wandb agent your-entity/fsae-car-rl/$sweep_id

# Wait for the sweep to complete
wait

# Clean up (optional)
# You can add any additional cleanup steps here, such as stopping the simulation script