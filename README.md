# Introduction-to-Robotics-DD2410-
Assignment work in KTH course- Introduction to Robotics.

## Assignment 1
This assignment allowed students to get familiarized with ros environment. The objective was to make a wall follower using Kobuki robot simulation. This was divided into three tasks:

- To make an open loop controller. 
- To make a cartesian controller (based on differential drive model) to make the robot go in a straight line
- To add a wall follower on top of cartesian controller to make the robot follow the wall.

## How to execute
Create a catkin workspace
`mkdir ~/catkin_ws && mkdir ~/catkin_ws/src`
Git clone this repo in src and then:
`catkin_make`

To execute task 2: 
`roslaunch ras_lab1_launch lab1_task2.launch`

To execute task 3:

`roslaunch ras_lab1_launch lab1_task3.launch`
