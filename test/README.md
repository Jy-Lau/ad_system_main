## Testing
Before carrying out integration testing, please make sure to perform installation by colcon build and sourcing the workspace. To test only this package, perform ```colcon build --packages-select decision_core path_planning_pkg trajectory_control obstacle_detection custom_msg ackermann_msgs``` and ```source install/setup.bash``` is sufficient, without building other packages. Details on how to clone every depedencies repo, please refer to [installation](../README.md#installation).
### Integration Tests
1. Setup pytest tool:
```bash
pip3 install pytest
```
2. Open four separate terminal to run the nodes (make sure to source the workspace beforehand). Ctrl + C to kill the node and rerun it for each test case:
```bash
ros2 run decision_core decision_core
```
```bash
ros2 run path_planning_pkg path_planning_node
```
```bash
ros2 run trajectory_control control
```
```bash
ros2 run obstacle_detection obstacle_detection
```
3. Run the integration tests for each of the test script and extract the log message, use the following command as an example:
```bash
python3 -m pytest -s -v TC_Int004.py > TC_Int004.log
```