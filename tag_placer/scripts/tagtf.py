#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
# from tf2_ros import TransformListener, TransformStamped, Buffer, tf2_geometry_msgs
import tf2_ros
import tf2_geometry_msgs


# Initialize the node
rospy.init_node('tagtfplacenode', anonymous=True)
# Create a publisher to publish the tag pose in the map frame
pub = rospy.Publisher('tagtf2place', PoseStamped, queue_size=10)
buffer = tf2_ros.Buffer()

tflistener = tf2_ros.TransformListener(buffer)

def tag_tfs_callback(tag2place):
    tag_pose = tag2place.pose
    
    # Convert the apriltag detection (camera frame) to a PoseStamped message in the map frame
    trans = buffer.lookup_transform('map', 'camera_link', rospy.Time())
    
    # Create a transform message
    transform = tf2_ros.TransformStamped()

    transform.transform.translation.x = trans.transform.translation.x
    transform.transform.translation.y = trans.transform.translation.y
    transform.transform.translation.z = trans.transform.translation.z
    transform.transform.rotation.x = trans.transform.rotation.x
    transform.transform.rotation.y = trans.transform.rotation.y
    transform.transform.rotation.z = trans.transform.rotation.z
    transform.transform.rotation.w = trans.transform.rotation.w

    # Transform the tag pose from the camera frame to the map frame
    pose_transformed = tf2_geometry_msgs.do_transform_pose(tag2place, transform)
    pose_transformed.header.frame_id = 'map'
    
    # Publish the tag pose in the map frame
    rospy.loginfo(pose_transformed)
    pub.publish(pose_transformed)    


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
