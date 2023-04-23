#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
import tf.transformations as tft
import numpy as np

# Create a publisher to publish the tag pose 
pub = rospy.Publisher('tag2place', PoseStamped, queue_size=10)


def tag_detections_callback(tag_detections):
    # Loop through all detected tags
    for detection in tag_detections.detections:
        # Extract tag information
        tag_id = detection.id
        tag_pose = detection.pose.pose.pose
        tag_size = detection.size

        # Convert the apriltag detection to a PoseStamped message in the camera_link frame
        tagIDstr = "camera_link"
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = tagIDstr
        pose_msg.pose.position.x = tag_pose.position.x
        pose_msg.pose.position.y = tag_pose.position.y 
        pose_msg.pose.position.z = tag_pose.position.z

        pose_msg.pose.orientation.x = tag_pose.orientation.x
        pose_msg.pose.orientation.y = tag_pose.orientation.y
        pose_msg.pose.orientation.z = tag_pose.orientation.z
        pose_msg.pose.orientation.w = tag_pose.orientation.w

        rospy.loginfo(pose_msg)
        pub.publish(pose_msg)



def talker():
    # Initialize the node
    rospy.init_node('tagplacenode', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    # Subscribe to the tag_detections topic to get AprilTag detections
    rospy.Subscriber('tag_detections', AprilTagDetectionArray, tag_detections_callback)

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
