# Introduction

This is a ROS implemention for DJI Onboard SDK based on DJI offical demo code.
And I am supporting MAVLink Now.You can use qgroundcontrol to takeoff/landing your M100 now.

# FILE STRUCTURE
I use C style C++ for this code, you could found lots of namespaces here.

# ROS

As for ros, I use services for control the aircraft, action to send mode cmd, and also topic to display information.It will support Odem for SLAM Application soon.

# MAVLINK
I am developing a full mavlink-controlable DJI with this app, you can use qgroundcontrol with UDP to control the aircraft, support display location and attitude now for controling ,now only support takeoff and landing.

Now support mavlink protcol of waypoints, you can simply use QGC for plan and flying.

# Roadmap
## Waypoint
Fully support QGroundControl style waypoint and guided mode.

## MAVLINK
Full support of common mavlink could control with QGC.

## ROS
Support for ros SLAM,like odem, and will privide some Gazebo support.

## Javascript
Using javascipt with node-js and ros-js to control DJI Onboard SDK.Will provide as internal function and also support javascipt manager.

# Acknowledgements
Thanks for help with DJI guys,they are Paul Yang, Jiahang Yin, Mingxi Wang.

# License
MIT and USING THIS SOFTWARE MEANS YOU WILL NOT LAUGH AT THE BAD ENGLISH LEVEL OF DEVELOPER :)
# DJI-Onboard-SDK-for-Mavlink
