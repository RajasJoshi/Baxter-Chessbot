#!/usr/bin/env python
import rospy
import tf
from gazebo_msgs.msg import LinkStates

class GazeboLinkStates:
    def __init__(self):
        self.pose = None
        self.input_linkname = None
        self.message = None
        self.link_poses = {}

    def get_links_gazebo(self, link_states_msg):
        self.message = link_states_msg
        self.link_poses = dict(zip(link_states_msg.name, link_states_msg.pose))

    def run(self):
        rospy.init_node("gazebo2tfframe")

        tfBroadcaster = tf.TransformBroadcaster()
        rospy.Subscriber("gazebo/link_states", LinkStates, self.get_links_gazebo)
        rospy.loginfo("Spinning")

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            for name in rospy.get_param("piece_names"):
                self.input_linkname = name
                pose = self.link_poses.get(self.input_linkname)
                if pose is not None:
                    pos = pose.position
                    ori = pose.orientation
                    tfBroadcaster.sendTransform(
                        (pos.x, pos.y, pos.z - 0.93),
                        (ori.x, ori.y, ori.z, ori.w),
                        rospy.Time.now(),
                        self.input_linkname,
                        "world",
                    )
                    rate.sleep()
        rospy.spin()

if __name__ == "__main__":
    GazeboLinkStates().run()