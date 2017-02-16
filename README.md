# position-Interpolation
Generate interpolation files used as an input to 'animate' (program used internally at ArM lab CSU)

A file called robot.key that contains the robot joint variable positions and velocities at
key frames. File project1_parta, takes in this file to generate the file robot.ang used by animate that
contains the values of the joint variables for each frame.

A file called object.key that contains the homogeneous transformation describing an
object’s position and orientation at key frames. File project1_partb takes in this file to generate the
file object.traj that contains the homogeneous transformations for the object at every
frame.

The interpolation done by maintaining continuity in the velocity and assuming object starts at rest.
