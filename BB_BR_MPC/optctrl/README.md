# MPC bluerov2
package to command bluerov2 using mpc


## Use case
### reference
`referencevarying.py` -> this script just sets the reference points and publishes to /optreferences topic which mpc listens. You can increase/decrease reference points via keyboard, you'll see in the script what corresponds to what. However, I suspect due to floating point issues sometimes using the keyboard for attitude commands creates instabilities (mpc glitches) so if you're gonna give attitude commands give it through argparser like below
example usage:
```bash
ros2 run optctrl refvar --x 0.0 --y 0.0 --z 5.0 --phi 0.0 --theta 0.0 --psi 0.0
```
attitude is in radians, position in meters
there is also a static reference setter which is just called 

`reference.py` , same logic except no keyboard increments.  you have to rerun the script with new waypoints. 

### mpc
The second script is just the controller, it solves a quadratic programme, the dynamics are nonlinear, however we linearize it at the current operating condition at each timestep, so it is a cheaters version of NMPC 😄
To start the controller, you just run
```bash
ros2 run optctrl mpc
```
It subscribes to odometry and references, publishes to thrusters directly