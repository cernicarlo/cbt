# BT policy

Package running the BT operator for the proposed scenarios

## Use case

The use case presented in the paper.

T1(Terminal 1): launch the simulated scenario

```bash
ros2 launch br2_stonefish blueboat_bluerov_heavy_fish.launch.py
```

T2: launch the action server that accepts `transition_to_pose` request and translate them in smooth `/optreferences`

```bash
ros2 launch optreferences optref_act_serv.launch.py
```

T3: launch the action server accepting `follow_bluerov`: when requested, the blueboat will chase the bluerov:

```bash
ros2 launch optreferences bb_pos_hold_act_serv.launch.py 
```

T4: launch mpc that accepts `/optreferences` and translate them in motor commands to move the bluerov

```bash
ros2 run optctrl mpc
```

T5: launch CV module (see below*)

T6: launch the BT manager that orchestrates the mission (check this terminal to go through the whole process)

```bash
cd ~/ros_ws/
./build/bt_policy/exec_bt_manager 
```

* to activate CV, we used a venv to better manage deps. Nothing stops you from using this as a normal ros2 node.
To generate the venv:

```bash
python3 -m venv detector_venv
source detector_venv/bin/activate
pip install opencv-python torchvision torch "numpy<2"
```

to activate venv and run the script:

```bash
# assuming you are in the folder where you create venv
source detector_venv/bin/activate
cd PATH_TO_THIS_FOLDER/scripts/
python3 detect_object.py
```

this node has a service server (`/bluerov/camera/get_detected_objects`) that, when triggered, analyze the image published in `/bluerov/camera/image_color` and publishes a Detection2dArray with the output in `/bluerov/camera/detected_objects`


### inspect_offshore_base (paper BT1, default `main_bt`)

`inspect_offshore_base` is the paper's main BT. It is structurally identical to
`inspect_anodes` below, but its Leaves use the paper Fig.1 names
(`BRGoToTargetA`/`BRGoToTargetB`, `BRInspectTargetA`/`BRInspectTargetB`). The
underlying node classes are the same, registered under these paper-accordant
aliases, so the recovery flow is identical to `inspect_anodes`.

### inspect_anodes

Set `bt_policy/bt_policy/config/bt_config.yaml/main_bt:` to `inspect_anodes`
(same logic as `inspect_offshore_base`, original leaf names).

1. the BT Manager (BTM) will try to execute `inspect_anodes`: send the ROV close to the first anode to inspect it
2. `inspect_anodes` will fail sending a request: failure because of `fish_detected_at_target_pose`
3. BTM will pass `fish_detected_at_target_pose` to the `resolution_map`
4. the `resolution_map` maps `fish_detected_at_target_pose` with the BT `deocclusion_maneuver_fish` to overcome the failure
5. the `resolution_map` passes `deocclusion_maneuver_fish` to BTM
6. BTM executes `deocclusion_maneuver_fish`
7. `deocclusion_maneuver_fish` succeeds
8. BTM executes `inspect_anode_right`
9. `inspect_anode_right` succeeds
10. The ROV is ready to inspect the other anode
11. the BT Manager (BTM) will try to keep executing `inspect_anodes`: send the ROV close to the other anode to inspect it
12. `inspect_anodes` will fail sending a request: failure because of `missing_object_at_pos`
13. BTM will pass `missing_object_at_pos` to the `resolution_map`
14. the `resolution_map` maps `missing_object_at_pos` with the BT `look_around` to overcome the failure
15. the `resolution_map` passes `look_around` to BTM
16. BTM executes `look_around`
17. `look_around` succeeds
18. BTM executes `inspect_anode_left`
19. `inspect_anode_left` succeeds
20. full BT stack executed

> **Note:** the `BT DB` (the place where the manager loads the BTs) is located in `PATH_TO_WS/install/share/bt_policy/bt_xml`. If you only need to *modify* a BT already present in the DB, as long as you built the ws using `--symlink-install`, then you can make the changge inside the `bt_xml` folder in the `src`. If you didn't use `--symlink-install`, or you added a *new* BT, then you need to either operate directly inside the BT DB or rebuild the package `bt_policy`.

### Dummy Tests

### 0
```bash
ros2 service call /help_request bt_policy/srv/HelpRequest "{node_failure: 'emergency_bt1'}"
```

### 1
* find a way to make fail/success first from main (currently, it will enter an infinte loop - ok for debugging)

* T1:
```bash
ros2 run action_tutorials_cpp fibonacci_action_server
```

T2
```bash
cd ~/ros_ws/
./build/bt_policy/exec_bt_manager 
```

the fibonacci should execute first-second-first-second-first (first: `hs_test_zero.xml`; second: `hs_test_one.xml`)
