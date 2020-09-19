# Interactive Waypoints:

This repo is inspired by http://wiki.ros.org/follow_waypoints and extends the functionality further. It allows to graphically control the list of waypoints, add/remove/change_id.. It also allows **Saving to file** and **sending to move_base** from a simple drop down menu.

<figure class="video_container">
<iframe src="https://www.youtube.com/embed/TEhuI_YelVc" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</figure>

## Description:

This package is a node that manages a waypoint server. Commands to add/delete/reorder etc are implemented as a menu items on waypoints (through rviz)

To use: add interactive waypoint description through rviz. First waypoint can be spawn through add_waypoint_topic (/initial_pose by default --> default topic of rviz pose initialisation tool)

## Subscriptions

- <add_waypoint_topic> expects PoseWithCovarianceStamped messages. Spawns waypoints at those poses. (default topic matched the rviz tool topic)
  #Params:

## Params

- add_waypoint_topic (default: /initialpose)
  See subscriptions
- connect_movebase (default: true)
  Enables move base functionality. Expects a movebase action server to be available at move_base
- enable_saving (default: true)
  Enables load/save functions.

### Notes:

- The waypoints will be in the frame of received poses

## Dependancies:

- pip install easygui. This is for file choice dialog. Should install using rosdep
- ros_msgdict: can be found [here](https://github.com/JuliusSustarevas/ros_msgdict)
