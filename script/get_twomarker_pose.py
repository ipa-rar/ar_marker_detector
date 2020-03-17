#!/usr/bin/env python
import rospy
import sys
import tf
import tf2_ros
import std_msgs.msg
import geometry_msgs.msg
import tf2_geometry_msgs
from geometry_msgs.msg import Pose
from ar_track_alvar_msgs.msg import AlvarMarkers
from ar_marker_detector.srv import getMarkerPose, getMarkerPoseResponse
worldFrame = "world"

class TwoMarkerDetector(object):

    def __init__(self):
        self.marker_pose = Pose()
        self.tf_buffer = tf2_ros.Buffer(
            rospy.Duration(1200.0))  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def ar_marker_cb(self, msg, args):
        marker_id = 0
        for marker in msg.markers:
            if marker.id == marker_id:  # check marker.id on /ar_pose_marker topic
                marker.pose.header.frame_id=marker.header.frame_id
                self.marker_pose=self.transform_pose(
                   marker.pose, worldFrame)
                   
    def transform_pose(self, pose, targetFrame):
        self.transform = self.tf_buffer.lookup_transform(targetFrame,
                                        pose.header.frame_id,  # source frame
                                        rospy.Time(0),        # at first available time
                                        rospy.Duration(1.0))  # wait up to 1 second
        return tf2_geometry_msgs.do_transform_pose(pose, self.transform).pose

    def getpickPose_cb(self,request): #pick marker pose Service call back
        pickMarkerID = request.ar_marker_id
        self.ar_pose_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.ar_marker_cb, pickMarkerID)
        resp = getMarkerPoseResponse()
        resp.pose = self.marker_pose
        self.marker_pose = Pose()
        return resp

    def getplacePose_cb(self,request): #place marker pose Service call back
        placeMarkerID = request.ar_marker_id
        self.ar_pose_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.ar_marker_cb, placeMarkerID)
        resp = getMarkerPoseResponse()
        resp.pose = self.marker_pose
        self.marker_pose = Pose()
        return resp

def main():
    rospy.init_node('TwoMarkerDetector')
    marker_detect=TwoMarkerDetector() 

    get_pick_pose = rospy.Service('/getPickPose', getMarkerPose, marker_detect.getpickPose_cb)
    get_place_pose = rospy.Service('/getPlacePose', getMarkerPose, marker_detect.getplacePose_cb)

    rospy.spin()

if __name__ == "__main__":
    main()
