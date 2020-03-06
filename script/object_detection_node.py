#!/usr/bin/env python
import rospy
import sys
import tf
import tf2_ros
import geometry_msgs.msg
import tf2_geometry_msgs
from ar_track_alvar_msgs.msg import AlvarMarkers


class object_detection_node(object):

    def __init__(self):
        rospy.loginfo("subscribing ar_pose_marker")
        self.tf_buffer = tf2_ros.Buffer(
            rospy.Duration(1200.0))  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.ar_pose_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.ar_marker_cb)

    def ar_marker_cb(self):
        rospy.loginfo("inside ar_marker callback")
        global marker_pose
        for marker in msg.markers:
            if marker.id == marker_id:  # check marker.id on /ar_pose_marker topic
                self.marker.pose.header.frame_id=marker.header.frame_id
                self.marker_pose=self.transform_pose(
                    self.marker.pose, world_frame)

    def transform_pose(self, pose, target_frame):
        self.transform=self.tf_buffer.lookup_transform(target_frame,
                                        pose.header.frame_id,  # source frame
                                        # at first available time
                                        rospy.Time(0),
                                        rospy.Duration(1.0))  # wait up to 1 second
        return tf2_geometry_msgs.do_transform_pose(pose, transform).pose






def main():
    rospy.init_node('object_detection_node')
    object_detect=object_detection_node()
    rospy.spin()

if __name__ == "__main__":
    main()
