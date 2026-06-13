# optreferences

server to gradually call optreferences for the mpc in the package Bluerov2_MPC

## Use case
*IMPORTANT*: for the full use case, check `bt_policy`

T1: launch stonefish
```bash
ros2 launch br2_stonefish blueboat_bluerov_heavy.launch.py
```

T2: launch server
```bash
ros2 launch optreferences optref_act_serv.launch.py
```

T3: launch mpc
```bash
ros2 run optctrl mpc
```

### from terminal

T4: send position
```bash
ros2 action send_goal /transition_to_pose optreferences/action/TransitionToPose "{z: 2.0}"
```

or directly:
```bash
ros2 topic pub /optreferences std_msgs/msg/Float64MultiArray "{data: [0.5, 1.0, 4.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```
