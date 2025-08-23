# tr-autonomy-training-visualization

## Introduction

In this training you will create visualizations and logging to determine the accuracy of the HuskyBot cv stack in a simulated environment.
You will use foxglove to easily visualize what is happening. Your end result might look something like the video below
<video width="800" controls>
    <source src=".md/foxglove_demo.webm" type="video/webm">
</video>

## Learning objectives

- understand the importance for good visualizations and logging in the debugging and performance analysis process
- create and incorprate a new package from scartch into a existing workspace
- work in an environment that is closer to the real TR autonomy stack, not just a isolated training workspace
- learn how to work with complex and nested msg types and custom msg types
- learn how to use the tf2 library
- introduction to the TR robot simulator  
- learn how to use rclpy and other python ros2 packages
- reinforce core ros2 skills

## Getting oriented

1. clone this repository
2. run `git submodule update --init` in the root dir of this repo

1. node to republish image frames with the bounding boxes drawn
2. node to publish the x,y,z error of solvepnp from huskybot using maniskill ground truth
3. node to publish a tf frame and use foxglove to visualize ground truth and observed measurement

import tr_messages DetWithImg
<https://docs.ros.org/en/kinetic/api/vision_msgs/html/msg/Detection2DArray.html>

make sure to launch with sim_time = true

maybe compare the ground truth to the timestamp of the detection vs the timestamp of the camera image
-> artificially adjust the timestamp of the detection to be later in time? might be weird with sim_time

Lessons to learn:
    why we use cpp on the robot? look at the tf exceptions. Even though it only takes "a few milliseconds" to broadcast the transforms, at the scales we work at that is too long

    probably fine for a quick check, but this also tells us that logging and visualizations for 
    millisecond critical code should be written in c++ and tested to see if it can keep up

![debug prints example](.md/image.png)

rename TR-Autonomy
to TR-autonomy

rm -r build/ install/ log/

foxglove consider setting fixed bounds for y axis in the plot settings instead of automatic

you might not want to select "now" as ur timestamp for measuring error. tf2 struggles a bit with real time
and since we are just using this for logging and visualization not real time control we can just say
current time - (some delay) and this will ensure our transform exists in the tf2 buffer and since we recieve a continous stream of data we will log and
visualize every data point

think deeply and be very careful about what timestamp you select for your transform.
the panels transform should match the timestamp of the camera position

## launch node
