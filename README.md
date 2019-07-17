# Arachne
Mainly 3D printed manipulator with 6 axis of freedom based on design of steward platform

**!!! Now the notes in this repository are rather assumptions that will need to be verified in practice !!!**

Platform is based on two tringle bases contected with 6 actuators. Control of these actuators is driven from motherboard.
Lower base is stationary, upper base part is movable.
Motherboard is positioned in lower triangle base. Motherboard comunicates with controller (pc) via serial line with G-code.
First version of platform is 3D printed. Next version are suppposed to be from aluminium.

# Motion system
Motion system is based on [affine transformation](https://en.wikipedia.org/wiki/Affine_transformation).
Basic system contains from seven points. Which are placed in center of platform and in points of connection with actuators.
Points are on both triangular bases. Coresponding points have same numbers.
Position of point is described as coordinates in cartesian system.
Center point is not used for calculation of motion, is used only as center point of mass.
- Can probably be used for optimalization of speed.

Every point is represented as vector [x,y,z,1].

### Travel distance
Distance between point on lower and upper bases is calculated.
- Calculation is based on usage of euclidian distance calculation in three dimensional space
- $d(a,b) = \sqrt{(a_x - b_x)^2 + (a_y - b_y)^2 + (a_z - b_z)^2}$

Affine transforation is apllied on all point of connection with actuators on upper base.
Now is calculated new distance between fixed and movable points. Diference bewtween distances in old and new position is travel distance of actuator.
By repeating this proces for every actuator is calculated one move of platform.

### Speed
Speed scaling is linear based on travel distance of actuator.
Acceleration must grow proportionally as speed.
Traveled distance is integration of actual speed of motor by time.
Example: Two motor M1 - travels 10mm, M2 - travels 30mm,
- Maximal speed is tested (before usage of device) and used for motor with the longest distance (as well as acceleration)
- M1 use three-times lower speed and acceleration becasuse of travel distance is three-timew lower
- Travel time is same for both motor, so motors cannot jam due to incorrect position

### Problems
Jams of platforms are posible when movement is not synchronized. In this state some motor can be in endposition, but motion is not done.
Main issue is when position of points in unknown. This happends when platform is powered on. Before power of platform must be homed -> this cannot be reliably secured.
Posible homing sequence is that all actuators will be moved with same speed to their home position. But there is possibility of jam.
