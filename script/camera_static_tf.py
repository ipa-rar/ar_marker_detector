#!/usr/bin/env python
import rospy
import sys
import tf 
import tf2_ros
import geometry_msgs.msg

class camera_static_tf(object):
    def __init__(self):
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.static_transformStamped = geometry_msgs.msg.TransformStamped()

    def tf2_broadcaster(self):
        self.static_transformStamped.header.stamp = rospy.Time.now()
        self.static_transformStamped.header.frame_id = "world"      #parent frame is world here      
        self.static_transformStamped.child_frame_id = sys.argv[1]   #child frame is camera here

        self.static_transformStamped.transform.translation.x = float(sys.argv[2])
        self.static_transformStamped.transform.translation.y = float(sys.argv[3])
        self.static_transformStamped.transform.translation.z = float(sys.argv[4])

        quat = tf.transformations.quaternion_from_euler(
                   float(sys.argv[5]),float(sys.argv[6]),float(sys.argv[7]))
        self.static_transformStamped.transform.rotation.x = quat[0]
        self.static_transformStamped.transform.rotation.y = quat[1]
        self.static_transformStamped.transform.rotation.z = quat[2]
        self.static_transformStamped.transform.rotation.w = quat[3]

        self.broadcaster.sendTransform(self.static_transformStamped)


def main():
    rospy.init_node('camera_static_tf')
    static_node = camera_static_tf()
    static_node.tf2_broadcaster()
    rospy.spin()

if __name__ == "__main__":
    main()