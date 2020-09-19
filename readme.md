# Description:

This package is a node that manages a waypoint server. Commands to add/delete/reorder etc are implemented as a menu items on waypoints (through rviz)/

To use: add interactive waypoint description through rviz. First waypoint can be spawn through add_waypoint_topic (/initial_pose by default --> default topic of rviz pose initialisation tool)

#Subscriptions
-<add_waypoint_topic> expects PoseWithCovarianceStamped messages. Spawns waypoints at those poses. (default topic matched the rviz tool topic)
#Params:

- add_waypoint_topic (default: /initialpose)
  See subscriptions
- connect_movebase (default: true)
  Enables move base functionality. Expects a movebase action server to be available at move_base
- enable_saving (default: false)
  Enables load/save functions.

#Notes:

- When saving/loading files the dialog default path will be within the package directory. Not ideal.
- The waypoints will be in the frame of received poses
- Something is not exactly right with how interactive marker server updates...
  #Dependancies:
- pip install easygui. This is for file choice dialog. Should install using rosdep

interactive_waypoints, inspired by http://wiki.ros.org/follow_waypoints.
adds loads more features, a dropdown box, ability to save/load files etc.
