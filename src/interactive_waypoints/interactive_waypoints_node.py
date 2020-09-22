#!/usr/bin/env python
import rospy
from waypoint_list import *
from waypoint import Waypoint
from waypoint_actions import *
from geometry_msgs.msg import PoseStamped

rospy.init_node("interactive_waypoints")
pose_topic = rospy.get_param(
    "~add_waypoint_topic", "/initialpose")
add_movebase_action = rospy.get_param(
    "~movebase_action", True)
add_saveload = rospy.get_param(
    "~saveload_action", True)


def pwcs2ps(pwcs_msg):
    msg = PoseStamped()
    msg.header = pwcs_msg.header
    msg.pose = pwcs_msg.pose.pose
    return msg


# create and setup server
wl = InteractiveWaypointList()
rospy.Subscriber(pose_topic, PoseWithCovarianceStamped,
                 lambda msg: wl.append(Waypoint(pwcs2ps(msg))))

# Add actions
if add_movebase_action:
    mb_action = WaypointMoveBaseAction(
        rospy.get_param("~move_base_namespace", "/move_base"))
    wl.attach_menu_action("goto", mb_action.goto_action,
                          mb_action.is_connected)
    wl.attach_menu_action("gotoall", mb_action.gotoall_action,
                          mb_action.is_connected)
    wl.attach_menu_action("cancel goto", mb_action.cancel_goals_action,
                          mb_action.is_connected)
if add_saveload:
    sl_action = WaypointSaveLoadAction()
    wl.attach_menu_action("save", sl_action.saveToPath, sl_action.is_connected)
    wl.attach_menu_action("load", sl_action.loadFromPath,
                          sl_action.is_connected)

rospy.spin()
