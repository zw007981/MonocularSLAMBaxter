# MonocularSLAMBaxter

**Contents**

* [Introduction](#Introduction)
* [Usage](#Usage)
* [Result](#Result)

## Introduction

A naive algorithm to solve monocular SLAM problem in C++ using Baxter robot.

## Usage

1. Enable the Baxter robot and open its camera.

``` 
rosrun baxter_tools enable_robot.py -e
rosrun baxter_interface joint_trajectory_action_server.py --mode position
rosrun baxter_tools camera_control.py -o left_hand_camera -r 1280x800
```

2. Save the pictures.

``` 
rosrun MonocularSLAMBaxter vision.py
```

3. Let the robot move.

``` 
rosrun MonocularSLAMBaxter move.py
```

4. Process the data collected.

Compile and execute `dense_mapping.cpp` .

## Result

Feature extraction and matching.
<img src="https://github.com/zw007981/MonocularSLAMBaxter/blob/master/features.png">

The depth image after 5 iterations.
<img src="https://github.com/zw007981/MonocularSLAMBaxter/blob/master/5.png" width="400">

The depth image after 100 iterations.
<img src="https://github.com/zw007981/MonocularSLAMBaxter/blob/master/100.png" width="400">

