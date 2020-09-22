#!/usr/bin/env python
import os
from copy import deepcopy
import rospy
import rospkg
import waypoint_list
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from geometry_msgs.msg import PoseWithCovarianceStamped,  PoseStamped, Quaternion
from ros_msgdict import msgdict

# default params
params = msgdict.yaml2msgdict(rospkg.RosPack().get_path(
    "interactive_waypoints")+"/res/waypoint_params.yaml")


def normalizeQuaternion(quaternion_msg):
    norm = quaternion_msg.x**2 + quaternion_msg.y**2 + \
        quaternion_msg.z**2 + quaternion_msg.w**2
    s = norm**(-0.5)
    quaternion_msg.x *= s
    quaternion_msg.y *= s
    quaternion_msg.z *= s
    quaternion_msg.w *= s
    return quaternion_msg


class WaypointMarker(InteractiveMarker, object):

    def __init__(self, marker_type="mesh"):
        """[InteractiveMarker for Waypoints. Moves in SE2, can be a sphere or a flag. Has a red arrow]

        Args:
            marker_type (str, optional): [marker type  "mesh" or "sphere"]. Defaults to "mesh".
        """
        InteractiveMarker.__init__(self)
        self.scale = params["control_scale"].data
        self._setup_markers()
        self._setup_controls()

    def _setup_controls(self):
        """[Sets up the SE2 controls. Add markers onto the planar one]
        """
        self.planar_control = InteractiveMarkerControl()
        self.planar_control.always_visible = True
        self.planar_control.orientation = normalizeQuaternion(
            Quaternion(0, 1, 0, 1))
        self.planar_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        self.controls.append(self.planar_control)
        self.planar_control.markers = self.markers

        self.rotation_control = InteractiveMarkerControl()
        self.rotation_control.orientation = normalizeQuaternion(
            Quaternion(0, 1, 0, 1))
        self.rotation_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        self.controls.append(self.rotation_control)

    def _setup_markers(self, marker_type="mesh"):
        """[Setup all visual componenets]

        Args:
            marker_type (str, optional): [mesh/sphere for the prefered marker type. ]. Defaults to "mesh".
        """
        self.markers = []
        if marker_type == "mesh":
            self.mesh_marker = Marker()
            self.mesh_marker.type = self.mesh_marker.MESH_RESOURCE
            self.mesh_marker.mesh_resource = params["flag_mesh"].data
            self.mesh_marker.scale = params["marker_mesh_scale"]
            self.mesh_marker.color = params["marker_color"]
            self.markers.append(self.mesh_marker)
        elif marker_type == "sphere":
            self.sphere_marker = Marker()
            self.sphere_marker.type = self.sphere_marker.SPHERE
            self.sphere_marker.scale = params["marker_sphere_scale"]
            self.sphere_marker.color = params["marker_color"]
            self.markers.append(self.sphere_marker)
        else:
            rospy.logwarn("Marker Type not supported")

        self.text_marker = Marker()
        self.text_marker.type = self.text_marker.TEXT_VIEW_FACING
        self.text_marker.scale = params["text_scale"]
        self.text_marker.color = params["text_color"]
        self.text_marker.text = "N/A"
        self.markers.append(self.text_marker)

        self.arrow_marker = Marker()
        self.arrow_marker.type = self.arrow_marker.ARROW
        self.arrow_marker.scale = params["arrow_scale"]
        self.arrow_marker.color = params["red_arrow_color"]
        self.markers.append(self.arrow_marker)


class Waypoint(object):

    def __init__(self, pose=None):
        """[Waypoint is a basic 'location' + 'interactive marker' implementaiton. It supports highlighting, displaying text etc. Any Waypoint should be sufficiently initialised by simply pose and pointer to server]

        Args:
            pose ([PoseWithCovarianceStamped, PoseStamped]) (optional): [Pose associated with the waypoint, if none is specified, waypoint is not uploaded to server]
        """
        self._iw_srv = waypoint_list.InteractiveWaypointServer()
        self._name = "Waypoint_" + \
            str(rospy.Time.now())  # Makes every wp unique

        self._int_marker = WaypointMarker()
        self._int_marker.name = self._name
        if pose:
            self.load_from_msg(pose)

    def duplicate(self):
        """[Create a new waypoint using the data of this waypoint. e.g. Duplicate]

        Returns:
            [type]: [description]
        """
        return self.__class__(self.get_pose())

    def _update(self):
        """[Update the pose with new one from rviz]
        """
        self._int_marker.pose = deepcopy(self._iw_srv.get(self._name).pose)

    def _upload(self):
        """[Overwrites the interactive marker from internal state to the server]
        """
        self._iw_srv.insert(self._int_marker, self._processFeedback)
        rospy.logdebug("Uploading: {}".format(self.get_name()))
        self._iw_srv.applyChanges()

    def _processFeedback(self, feedback):
        """[Gets executed anytime marker is interacted with on rviz]

        Args:
            feedback ([InteractiveMarkerFeedback]): [feedback msg from rviz]
        """
        self._update()
        self._iw_srv.applyChanges()

    # Getters
    def get_name(self):
        """[Get waypoint name]

        Returns:
            [Str]: [Waypoint Name]
        """
        return self._name

    def get_text(self):
        """[Get waypoint text]

        Returns:
            [Str]: [Waypoint text]
        """
        return self._int_marker.text_marker.text

    def get_pose(self):
        """[Get current pose associated to this waypoint]

        Returns:
            [PoseStamped]: [current pose associated to this waypoint]
        """
        pose = PoseStamped()
        pose.header.frame_id = self._int_marker.header.frame_id
        pose.pose = deepcopy(self._int_marker.pose)
        return pose

    # Setters
    def set_highlight(self, hglt):
        """[Set highlighting]

        Args:
            hglt ([Bool]): [if True, waypoint is highlighted]
        """
        if hglt:
            self._int_marker.markers[0].color = params["marker_color_hl"]
        else:
            self._int_marker.markers[0].color = params["marker_color"]
        self._upload()

    def set_text(self, txt):
        """[Set the text on the marker]

        Args:
            txt ([Str]): [text]
        """
        self._int_marker.text_marker.text = txt
        self._upload()

    def set_pose(self, msg):
        """[Set the pose associated with this waypoint and update the waypoint]

        Args:
            pose ([PoseStamped]): [pose associated with this waypoint]
        """
        self._int_marker.header.frame_id = msg.header.frame_id
        self._int_marker.pose = msg.pose
        self._int_marker.pose.position.z = 0.1
        self._upload()

    def remove(self):
        """[Remove this waypoint from server]
        """
        self._iw_srv.erase(self._name)
        self._iw_srv.applyChanges()

    def save_to_msg(self):
        """[Convert waypoint into a restorable ROS message. This is done for file saving convenience.]
        Returns:
            [PoseStamped]: [ros message such that output of this function given to from_msg() would recreate the waypoint]
        """
        return self.get_pose()

    def load_from_msg(self, msg):
        """[Setup waypoint data from associated message]

        Args:
            msg ([PoseStamped]): [pose associated with this waypoint]
        """
        if isinstance(msg, PoseWithCovarianceStamped):
            pose = msg.pose.pose
        elif isinstance(msg, PoseStamped):
            pose = msg.pose
        else:
            rospy.logerr(
                "Cannot create Waypoint from {}".format(msg.__class__))
        self.set_pose(msg)
        self._upload()
