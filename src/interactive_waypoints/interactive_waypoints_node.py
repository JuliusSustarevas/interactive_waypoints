#!/usr/bin/env python
import rospy
from waypoint_list import *

rospy.init_node("interactive_waypoints")
pose_topic = rospy.get_param(
    "~add_waypoint_topic", "/initialpose")
srv_name = rospy.get_param("~waypoint_server_name", "waypoint_srv")
add_movebase_action = rospy.get_param(
    "~movebase_action", True)
add_saveload = rospy.get_param(
    "~saveload_action", True)

# create and setup server
wl = InteractiveWaypointList(srv_name)
rospy.Subscriber(pose_topic, PoseWithCovarianceStamped,
                 lambda msg: wl.append(Waypoint(msg)))

# Add actions
if add_movebase_action:
    mb_action = WaypointMoveBaseAction(
        rospy.get_param("~move_base_namespace", "/move_base"))
    wl.attach_action("goto", mb_action.goto_action, mb_action.is_connected)
    wl.attach_action("gotoall", mb_action.gotoall_action,
                     mb_action.is_connected)
    wl.attach_action("cancel goto", mb_action.cancel_goals_action,
                     mb_action.is_connected)
if add_saveload:
    sl_action = WaypointSaveLoadAction()
    wl.attach_action("save", sl_action.saveToPath, sl_action.is_connected)
    wl.attach_action("load", sl_action.loadFromPath,
                     sl_action.is_connected)

rospy.spin()
