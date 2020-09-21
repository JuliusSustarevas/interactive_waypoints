#!/usr/bin/env python
import rospy
import rosparam
import actionlib
from copy import deepcopy
from waypoint import Waypoint
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray
from std_msgs.msg import Empty
from rospy_message_converter import message_converter
from ros_msgdict import msgdict


class Singleton(type):
    _instances = {}

    def __call__(cls, *args, **kwargs):
        """[Singleton metaclass definition]
        """
        if Singleton._instances.get(cls, None) is None:
            Singleton._instances[cls] = super(
                Singleton, cls).__call__(*args, **kwargs)
        return Singleton._instances[cls]


# InteractiveMarkerServer and MenuHandler are turned into singletons to reduce passing around of references.
class InteractiveWaypointServer(InteractiveMarkerServer, object):
    __metaclass__ = Singleton


class WaypointMenuHandler(MenuHandler, object):
    __metaclass__ = Singleton


class WaypointList(object):

    def __init__(self):
        """[Basic waypoint list. This means a list + server to hold the markers in. Maintains waypoint text as list index
        Note that this class is intended to mirror the internal state of the waypoint server. i.e. if waypoint is in this list, its on the server, if its removed from here, its removed from server.
        Note that this is Waypoint Class Agnostic]

        Args:
            WaypointClass (Class): [Class used to instantiate new waypoints etc.]
            srv_name (str, optional): [description]. Defaults to "Interactive_waypoints".
        """
        # Create server
        self._server = InteractiveWaypointServer(
            rospy.get_param("~waypoint_server_name", "waypoint_srv"))

        self._wp_list = []

    def get_list(self):
        """[Return list of Waypoint objs]

        Returns:
            [List]: [list of Waypoints]
        """
        return self._wp_list

    def get_wp(self, iid):
        """[Return waypoint, given index or name]

        Args:
            iid ([Str,int]): [waypoint name or index]

        Returns:
            [Waypoint]: [waypoint]
        """
        if type(iid) == str:
            for ii in range(self.len()):
                if self._wp_list[ii].get_name() == iid:
                    return self._wp_list[ii]
            rospy.logerr("Cannot find waypoint named {}".format(iid))
        elif type(iid) == int:
            return self._wp_list[iid]
        else:
            rospy.logerr("Cannot index with {}".format(type(iid)))

    def append(self, wp):
        """[Append the list with this waypoint.]

        Args:
            wp ([Waypoint]): [waypoint]
        """
        self.insert(self.len(), wp)

    def insert(self, idx, wp):
        """[Insert Waypoint wp to position idx in WaypointList and upload it to server]

        Args:
            idx ([int]): [description]
            wp ([Waypoint]): [description]
        """
        self._wp_list.insert(idx, wp)
        wp._upload()  # upload itself unto the server
        self.reset_text()

    def remove(self, wp):
        """[ Remove Waypoint to WaypointList]

        Args:
            wp ([Waypoint,str,int]): [waypoint, its name or index]
        """
        if isinstance(wp, int) or isinstance(wp, str):  # if id or name passed
            wp = self.get_wp(wp)
        self.pop(wp)  # pop without return

    def clearall(self):
        """[Remove all waypoints]
        """
        for _ in range(self.len()):
            self.pop(self.get_wp(self.len()-1))

    def pop(self, wp=None):
        """[Pop Waypoint out of the list]

        Args:
            wp ([type],optional): [Return this wp. If no wp given, pops last wp. ] Defaults to None

        Returns:
            [Waypoint]: [waypoint]
        """
        if not wp:  # if no wp, then last wp
            wp = self.get_wp(len(self._wp_list)-1)
        if isinstance(wp, str) or isinstance(wp, int):  # if id or name passed
            wp = self.get_wp(wp)
        wp.remove()  # removes from server
        self._wp_list.remove(wp)  # removes from list
        self.reset_text()  # resets text of all WPs
        return wp

    def len(self):
        """[Return length of list]

        Returns:
            [int]: [length of list]
        """
        return len(self._wp_list)

    def reset_text(self):
        """[Maintain consistent text ids for all waypoints]
        """
        for ii in range(self.len()):
            self._wp_list[ii].set_text(str(ii+1))

    def save_to_msg(self):
        """[Convert waypoint into a restorable ROS message. This is done for file saving convenience.]
        Returns:
            [PoseStamped]: [ros message such that output of this function given to from_msg() would recreate the waypoint]
        """
        pose_array = PoseArray()
        if self.len() == 0:
            return pose_array

        for wp in self.get_list():
            msg = wp.save_to_msg()
            pose_array.header.frame_id = msg.header.frame_id
            pose_array.poses.append(msg.pose)

        return pose_array

    def load_from_msg(self, msg):
        """[Load from PoseArray]

        Args:
            msg ([type]): [description]
        """
        self.clearall()
        frame_id = msg.header.frame_id
        wp_pose = PoseStamped()
        wp_pose.header.frame_id = frame_id
        for pose in msg.poses:
            wp = Waypoint()
            wp_pose.pose = pose
            wp.load_from_msg(deepcopy(wp_pose))
            self.append(wp)


class InteractiveWaypointList(WaypointList):

    def __init__(self):
        """[ Interactive Waypoint list: Extends WaypointList; is a list of interactive markers/poses/data  represented by an instance of Waypoint; implements right click drop down menu for actions; tracks waypoints in order; Implements attaching menu actions to be performed on a waypoint or on a list]

        Args:
            srv_name ([Str]): [Interactive Marker server name]
        """
        WaypointList.__init__(self)
        self._menu_handler = WaypointMenuHandler()
        # Menu items:
        self._menu_handler.insert(
            "Duplicate", callback=lambda feedback: self.duplicate(self.get_wp(feedback.marker_name)))
        self._changeid_handle = self._menu_handler.insert("Change ID")
        self._menu_handler.insert(
            "Delete", callback=lambda feedback: self.remove(feedback.marker_name))
        self._menu_handler.insert(
            "clearall", callback=lambda feedback: self.clearall())
        # action timers
        self.action_state_timers = []

    def attach_action(self, menu_item_name, exec_cb, check_cb):
        """[Attach a menu item and associated callback to be offered on right click menu]

        Args:
            menu_item_name ([str]): [item name displayed in drop down menu]
            exec_cb (function): [Function with args: WaypointList, waypoint_name]
            check_cb (function): [Function taking no args, returns True or False if action is available or not]
        """

        menu_item = self._menu_handler.insert(
            menu_item_name, callback=lambda feedback: exec_cb(self, feedback.marker_name))

        def check_status(_):
            self._menu_handler.setVisible(menu_item, check_cb())
            self._menu_handler.reApply(self._server)
            self._server.applyChanges()

        self.action_state_timers.append(
            rospy.Timer(rospy.Duration(1.0), check_status))

    def _updateMenu(self):
        """[Deletes all change_id menu entries and updates with new ones. Text is taken straight from the waypoints]
        """
        del(self._menu_handler.entry_contexts_[
            self._changeid_handle].sub_entries[:])
        for ii in range(self.len()):
            self._menu_handler.insert(
                self.get_wp(ii).get_text(), parent=self._changeid_handle, callback=lambda feedback: self.changeID(feedback.marker_name, int(self._menu_handler.entry_contexts_[
                    feedback.menu_entry_id].title)-1))
        for wp in self._wp_list:
            self._menu_handler.apply(self._server, wp.get_name())
            self._menu_handler.reApply(self._server)
            self._server.applyChanges()

    def changeID(self, wp_old, new_id):
        """[Change the ID of a waypoint. Means to pop and then insert back in a different place]

        Args:
            wp_old ([Waypoint, str, int]): [waypoint , its name or index]
            new_id ([int]): [new index]
        """
        if not isinstance(wp_old, Waypoint):
            wp_old = self.get_wp(wp_old)
        self._wp_list.remove(wp_old)
        self.insert(new_id, wp_old)

    def duplicate(self, wp):
        """[Duplicate copies the orientation/pose of the given point.  and creates a new waypoint.]

        Args:
            wp ([Waypoint]): [waypoint]
        """
        self.append(wp.duplicate())

    def insert(self, idx, wp):
        """[see :func:`~waypoint_list.WaypointList.insert`]
        """
        WaypointList.insert(self, idx, wp)
        self._updateMenu()

    def pop(self, wp):
        """see :func:`~waypoint_list.WaypointList.pop`"""
        wp = WaypointList.pop(self, wp)
        self._updateMenu()
        return wp

    def saveToPath(self, fullfilename):
        if not fullfilename:
            rospy.logerr("Cannot save, no Filename given")
            return
        rospy.loginfo(fullfilename)
        rospy.loginfo("Saving waypoitns to: "+fullfilename)
        # clearall local params
        msg = self.save_to_msg(self)
        msgdict.msgdict2yaml(msg, fullfilename)

    def loadFromPath(self, fullfilename):
        if not fullfilename:
            rospy.logerr("Cannot Load, no Filename given")
            return
        rospy.loginfo("Loading waypoitns from: "+fullfilename)
        msg = msgdict.yaml2msgdict(fullfilename)
        self.load_from_msg(msg)
