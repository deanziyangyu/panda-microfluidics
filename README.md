# panda-microfluidics
panda-microfluidics

Repository to Manipulate a franka robot to grasp microfluidic items 

Authors: Ziyang Yu and Eric Liang

Repository: https://github.com/deanziyangyu/panda-microfluidics

### Usage:

e to the folder using cd /home/scratchpad/franka_ros2_csc379 and run the following
commands in both terminals:
source /opt/ros/humble/setup.bash
source install/setup.bash


Launch the Docker Container, run
`ros2 launch franka_gripper gripper.launch.py robot_ip:=<YOUR IP HERE`
`ros2 run franka_ros2_csc379 run_franka_impedance_control_ros2`
for impedance control and gripper server.

Launch franka_run.py first, then followed by match_makers_gui (with the correct args specified below, such as `match_markers_gui.py -m s`). The GUI should be automatically connected to ROS node if the ports are correctly configured.

### Repository Composition

Two Main Programs: `franka_run.py` and `match_markers_gui.py`

### Franka ROS2 Node: franka_run.py

1. Control program for Franka Emika Panda Robot though ROS2.
2. Has both online and offilne modes for operation without the ROS2 container
3. Via sockets, sends
- JointPos
- Video recording (starts automaticaly if a datacollection directory is specified)
- Logging
to the GUI program.
4. Accepts {'x', 'y', 'z', 'r', 'p', 'y'} from GUI via another client sockets.


### Match Maker GUI: match_markers_gui.py

1. A OpenCV-based Webcam Server Listening on [IP] [PORT].
It uses Pyqt5 based qtthreads for realtime Concurrency of the below functionalities:
- Video capture thread
- Feature Extraction
- Homography Extraction thread
- Spatial Processing Thread (Performing ik for the given jointpos and homography)
- IP socket
- JointPos
- Video recording
- Logging
3. Via sockets, receives
- JointPos
- Video recording (starts automaticaly if a datacollection directory is specified)
- Logging
4. Sends {'x', 'y', 'z', 'r', 'p', 'y'} from GUI via another client sockets.

Usage:
    for startup without path and timestamp specified:
        `py ./match_markers_gui.py --mode s`
    for startup specifying path and timestamp:
        ```
        py ./webcam_server.py --mode v --path ./path/to/data_collection_folder/YYMMDD_HHMMSS \
            --init_time YYMMDD_HHMMSS
        ```
Shell launch cmd from other .py program:
    silent:  `Popen(["py", "./match_markers_gui.py", "--mode", "s"])`
    verbose: 
    ```
    Popen(["py", "./match_markers_gui.py",
        "--mode", "v",
        "--path", "./path/to/data_collection_folder/YYMMDD_HHMMSS",
        "--init_time", "YYMMDD_HHMMSS"])
    ```

