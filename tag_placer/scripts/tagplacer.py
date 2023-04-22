#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
import tf.transformations as tft
import numpy as np

pub = rospy.Publisher('tag2place', PoseStamped, queue_size=10)


def tag_detections_callback(tag_detections):
    # Loop through all detected tags
    for detection in tag_detections.detections:
        tag_id = detection.id
        tag_pose = detection.pose.pose.pose
        #rospy.loginfo(tag_pose)
        tag_size = detection.size
        # print("Detected tag with ID:", tag_id)
        # print("Tag pose:")
        # print(tag_pose)
        # print("Tag size:", tag_size)

        tagID=1
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

        # # apply a 90 degree rotation about the x axis
        # q = tft.quaternion_from_euler(tft.pi / 2, 0, 0)
        # R = np.array([[0,1,0,0], [0,0,1,0], [1,0,0,0], [0,0,0,1]])
        
        # q = tft.quaternion_from_matrix(R)
        # print(np.shape(q))
        # # # get a quaternion of the og pose
        # q_old = [pose_msg.pose.orientation.x, pose_msg.pose.orientation.y,
        #          pose_msg.pose.orientation.z, pose_msg.pose.orientation.w]
        # print(np.shape(q_old))
        # q_new = tft.quaternion_multiply(q_old, q)

        # pose_msg.pose.orientation.x = q_new[0]
        # pose_msg.pose.orientation.y = q_new[1]
        # pose_msg.pose.orientation.z = q_new[2]
        # pose_msg.pose.orientation.w = q_new[3]

        

        rospy.loginfo(pose_msg)
        pub.publish(pose_msg)



def talker():
    rospy.init_node('tagplacenode', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    rospy.Subscriber('tag_detections', AprilTagDetectionArray, tag_detections_callback)

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
