# bt_policy
This project is inspired by [btcpp_sample](https://github.com/BehaviorTree/btcpp_sample) and, hereby, compatible with version 4.X of [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) (that you must download and build for this repo to work together with ROS noetic - ).

The **CMakeLists.txt** and **package.xml** files are compatible with both ROS and ROS2.

## Installation

- Once you install ROS Humble, create a workspace (eg. `~/ros_ws`) following ROS instructions for ws creation.
- clone [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) (this work was built using this [commit](https://github.com/BehaviorTree/BehaviorTree.CPP/commit/1fcb624d4d7b9d1f357378b20ff19bdcc3853cea)) in `src` (eg. `~/ros_ws/src`)
- clone this repo in the same `src` folder (eg. `~/ros_ws/src`)
- modify `bt_policy/include/bt_policy/utils.h L14` the variable `path` with the path where is located your local `bt_policy/xml` file and rebuild <!-- TODO: assign a dynamic variable -->
- build the ws (eg. execute `colcon build` in `~/ros_ws`)
- `source ~/ros_ws/devel/setup.bash`


## For users

### start simulation

Follow instructions in `bt_policy/README.md`


### modify behavior

- add/remove the BT in `bt_policy/bt_xml` file following the convenctions
- in `bt_policy/scenario/failure.xml` file specify the reason for each failure (`Nodename AsInTheBT = reasonOfFailure`)
- in `bt_policy/scenario/solution.xml` file specify the solution for each `reasonOfFailure` (`reasonOfFailure solution = NameFileBTCamelCaseNoXMLExtension`)


## For developers

* create the node in the `bt_policy/nodes` folder [there are good examples to be inpired by in oil_structure_inspection_true_nodes]
* update the database (if series) in `bt_policy/nodes/node_utils.cpp/nodeRegistrator()` if cases with the class created
* (ignore for now) NOTE: the attributes in the bt.xml are environment variable that, if equal to the values, return automatically success; for PostSequence/Fallback, the last word after underscore says the target of the condition (true/false)
