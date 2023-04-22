#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
# from tf2_ros import TransformListener, TransformStamped, Buffer, tf2_geometry_msgs
import tf2_ros
import tf2_geometry_msgs


rospy.init_node('tagtfplacenode', anonymous=True)
pub = rospy.Publisher('tagtf2place', PoseStamped, queue_size=10)
buffer = tf2_ros.Buffer()

tflistener = tf2_ros.TransformListener(buffer)

def tag_tfs_callback(tag2place):
    tag_pose = tag2place.pose
    
    # transformed_pose = tflistener.transformPose('map',tag_pose)
    # trans = buffer.lookup_transform('map', tag2place.header.frame_id, rospy.Time())
    trans = buffer.lookup_transform('map', 'camera_link', rospy.Time())
    
    transform = tf2_ros.TransformStamped()

    transform.transform.translation.x = trans.transform.translation.x
    transform.transform.translation.y = trans.transform.translation.y
    transform.transform.translation.z = trans.transform.translation.z
    transform.transform.rotation.x = trans.transform.rotation.x
    transform.transform.rotation.y = trans.transform.rotation.y
    transform.transform.rotation.z = trans.transform.rotation.z
    transform.transform.rotation.w = trans.transform.rotation.w

    pose_transformed = tf2_geometry_msgs.do_transform_pose(tag2place, transform)
    pose_transformed.header.frame_id = 'map'
    
    rospy.loginfo(pose_transformed)
    pub.publish(pose_transformed)    
    
    # tagIDstr = "map"
    # pose_msg = PoseStamped()
    # pose_msg.header.frame_id = tagIDstr
    # pose_msg.pose.position.x = transformed_pose.position.x
    # pose_msg.pose.position.y = transformed_pose.position.y
    # pose_msg.pose.position.z = transformed_pose.position.z
    # pose_msg.pose.orientation.x = transformed_pose.orientation.x
    # pose_msg.pose.orientation.y = transformed_pose.orientation.y
    # pose_msg.pose.orientation.z = transformed_pose.orientation.z
    # pose_msg.pose.orientation.w = transformed_pose.orientation.w

    # rospy.loginfo(pose_msg)
    # pub.publish(pose_msg)



def talker():
    
    rate = rospy.Rate(10) # 10hz
    rospy.Subscriber('tag2place', PoseStamped, tag_tfs_callback)

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
