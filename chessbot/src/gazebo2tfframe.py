#!/usr/bin/env python
import rospy
import tf
from gazebo_msgs.msg import LinkStates


class Gazebo2TFFrame:
    def __init__(self):
        self.poses = {}
        self.tfBroadcaster = tf.TransformBroadcaster()
        rospy.init_node("gazebo2tfframe")
        rospy.Subscriber("gazebo/link_states", LinkStates, self.get_links_gazebo)
        self.rate = rospy.Rate(10)

    def get_links_gazebo(self, link_states_msg):
        for link_idx, link_name in enumerate(link_states_msg.name):
            self.poses[link_name.split("::")[0]] = link_states_msg.pose[link_idx]

    def run(self):
        while not rospy.is_shutdown():
            for name in rospy.get_param("piece_names"):
                pose = self.poses.get(name)
                if pose is not None:
                    pos = pose.position
                    ori = pose.orientation
                    self.tfBroadcaster.sendTransform(
                        (pos.x, pos.y, pos.z - 0.93),
                        (ori.x, ori.y, ori.z, ori.w),
                        rospy.Time.now(),
                        name,
                        "world",
                    )
                    self.rate.sleep()
        rospy.spin()


if __name__ == "__main__":
    node = Gazebo2TFFrame()
    node.run()
