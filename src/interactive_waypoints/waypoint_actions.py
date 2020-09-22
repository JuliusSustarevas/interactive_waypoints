#!/usr/bin/env python
import rospy
import rosparam
import actionlib
import functools
import threading
import easygui
import os
from rospy_message_converter import message_converter
from copy import deepcopy
from interactive_markers import *
from visualization_msgs.msg import *
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray
from std_msgs.msg import Empty
from ros_msgdict import msgdict


def on_own_thread(func):
    """[Decorator for exectuting a function on its own thread]

    Args:
        func ([function]): [function to be executed]
    """
    @functools.wraps(func)
    def _exec(*args, **kwargs):
        thread = threading.Thread(target=lambda: func(*args, **kwargs))
        thread.start()

    return _exec


def _movebase_command(func):
    """[Decorator for catching movebase  not being able to connect]

    Args:
        func ([function]): [Fucntion being wrapped]

    Returns:
        [Depends]: [function output]
    """
    @functools.wraps(func)
    def exec_command(self, *args, **kwargs):
        try:
            return func(*args, **kwargs)
        except Exception as e:
            rospy.logerr(
                "Calling Movebase failed, maybe movebase is down?: {}".format(e))
            self._mb_connected = False
    return exec_command


class WaypointMoveBaseAction(object):

    def __init__(self, move_base_nm="/move_base"):
        self._mb_connected = False
        # Timer for monitoring conenction
        self.mb_timer = rospy.Timer(rospy.Duration(
            5), lambda event: self.ping_move_base)
        self._mb_client = actionlib.SimpleActionClient(
            move_base_nm, MoveBaseAction)

    def ping_move_base(self):
        if not self._mb_client.wait_for_server(timeout=rospy.Duration(3.0)):
            rospy.loginfo_throttle(30,
                                   "Waypoint lost move_base connection" if self._mb_connected else "Failed to connect to movebase")
        else:
            if not self._mb_connected:
                rospy.loginfo('Waypoint follower Connected to move_base.')
            self._mb_connected = True

    def is_connected(self):
        """[getted for mb connected status]

        Returns:
            [Bool]: [_mb_connected]
        """
        return self._mb_connected

    @_movebase_command
    def _send_goal_pose(self, pose):
        goal = MoveBaseGoal()
        goal.target_pose = pose
        self._mb_client.send_goal(goal)
        self._mb_client.wait_for_result()

    @_movebase_command
    def cancel_goals_action(self, waypoint_list, waypoint_name):
        self._mb_client.cancel_all_goals()

    @on_own_thread
    def goto_action(self, waypoint_list, waypoint_name):
        wp = waypoint_list.get_wp(waypoint_name)
        if self._mb_connected:
            self._send_goal_pose(wp.get_pose().pose)
        else:
            rospy.logerr(
                "Attempting to invoke move_base, but client is not connected")

    @on_own_thread
    def gotoall_action(self, waypoint_list, waypoint_name):
        if self._mb_connected:
            waypoints = deepcopy(waypoint_list.get_list())
            for wp in waypoints:
                self._send_goal_pose(wp.get_pose().pose)
        else:
            rospy.logerr(
                "Attempting to invoke move_base, but client is not connected")


class WaypointSaveLoadAction(object):

    @staticmethod
    def get_fileopen_path():
        return easygui.fileopenbox(
            default="", msg="Select yaml file to load", title="Load Path", filetypes=["*.yaml", "Config files"])

    @staticmethod
    def get_filesave_path():
        return easygui.filesavebox(
            default="path1", msg="Create save file", title="Save Path", filetypes=["*.yaml", "Config files"])

    @on_own_thread
    def saveToPath(self, waypoint_list, waypoint_name):
        """[Save waypoint_list to file using simple easygui]

        Args:
            waypoint_list ([WaypointList]): [Instance of WaypointList]
            waypoint_name ([str]): [unused]
        """
        waypoint_list.saveToPath(
            self.get_filesave_path(), "Path"+str(rospy.Time.now()))

    @on_own_thread
    def loadFromPath(self, waypoint_list, waypoint_name):
        """[Reset waypoint list and load up from file]

        Args:
            waypoint_list ([WaypointList]): [Instance of WaypointList]
            waypoint_name ([str]): [unused]
        """
        waypoint_list.loadFromPath(self.get_fileopen_path())

    def is_connected(self):
        """[Returns connected status]

        Returns:
            [Bool]: [_mb_connected]
        """
        return True
