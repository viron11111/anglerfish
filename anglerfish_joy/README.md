#anglerfish_joy

Joy node for use with Logitech F710 controller.

Button/axes functions:

axes:
[0] : Left thumb X-axis: translate desired position along x-axis
[1] : Left thumb Y-axis: translate desired position along y-axis
[3] : Right thumb X-axis: yaw about the z-axis

buttons:
[0] :  A (green): descend desired TF by 0.1 meters
[1] :  B (red): set desired TF to Anglerfish's current position
[2] :  X (blue): ascend desired TF by 0.1 meters
[3] :  Y (yellow): make desired TF new position (Anglerfish will attempt to reach this position/orientation)
[5] :  RB: reset desired TF to 0 (pos: 0,0,0 rot: 0,0,0,1)
