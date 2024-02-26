#!/usr/bin/env python
import rospy
import tf
from gazebo_msgs.msg import LinkStates

# Global variable where the object's pose is stored
pose = None
input_linkname = None
message = None


def get_links_gazebo(link_states_msg):
    # Call back to retrieve the object you are interested in
    global input_linkname
    global pose
    global message
    poses = {"world": link_states_msg.pose[0]}  # get world link
    message = link_states_msg


if __name__ == "__main__":
    rospy.init_node("gazebo2tfframe")

    # Create TF broadcaster -- this will publish a frame give a pose
    tfBroadcaster = tf.TransformBroadcaster()
    # SUbscribe to Gazebo's topic where all links and objects poses within the simulation are published
    rospy.Subscriber("gazebo/link_states", LinkStates, get_links_gazebo)
    rospy.loginfo("Spinning")
    global pose
    global message
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        for name in rospy.get_param("piece_names"):
            global input_linkname
            input_linkname = name
            if message is not None:
                for link_idx, link_name in enumerate(message.name):
                    modelname = link_name.split("::")[0]
                    if input_linkname == modelname:
                        pose = message.pose[link_idx]
                        if pose is not None:
                            print(input_linkname)
                            pos = pose.position
                            ori = pose.orientation
                            rospy.loginfo(pose)
                            # Publish transformation given in pose
                            tfBroadcaster.sendTransform(
                                (pos.x, pos.y, pos.z - 0.93),
                                (ori.x, ori.y, ori.z, ori.w),
                                rospy.Time.now(),
                                input_linkname,
                                "world",
                            )
                            rate.sleep()
    rospy.spin()
