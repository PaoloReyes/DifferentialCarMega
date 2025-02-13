# Paint Machine Car - README

## Overview
This repository contains the code for the car module of the automatic paint machine. The car is responsible for transporting itself along the machine to position under the corresponding container and produce the desired color.

## Features
- **Automated Motion Control**: Uses DC motors with encoders for precise movement along the track.
- **Positioning System**: Moves to specific locations under the paint containers.
- **Sensor Feedback**: Utilizes encoders for movement validation.
- **Communication**: Interfaces with the main controller via RS485 communication.

## Communication Protocol
The car module receives and sends commands via a predefined protocol:
- `MOVE POSITION SPEED` - Moves to the specified container position at the given speed.
- `STOP` - Immediately stops all movement.
- `STATUS` - Returns the current position based on encoder feedback.
- `CALIBRATE` - Runs the homing sequence to set reference positions.

## Troubleshooting
| Issue               | Possible Cause                | Solution |
|---------------------|-----------------------------|----------|
| Motor not moving   | Loose wiring, wrong pinout  | Check connections, verify pin configuration |
| Incorrect position | Calibration error           | Re-run calibration sequence |
| No response       | Communication failure       | Verify RS485 wiring and settings |


## Contributors
- **Paolo Reyes** - Lead Developer

## License
This project is licensed under the MIT License - see the LICENSE file for details.

