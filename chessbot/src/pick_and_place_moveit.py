#!/usr/bin/env python
import sys
import copy
import tf2_ros
import rospy
import baxter_interface
import moveit_commander
import numpy as np

from std_msgs.msg import (
    Empty,
)

from geometry_msgs.msg import (
    Pose,
    Point,
    Quaternion,
)


states = [
    "INIT",
    "FIRST",
    "SECOND",
    "THIRD",
    "FOURTH",
    "FIFTH",
    "SIXTH",
    "SEVENTH",
    "EIGHTH",
]
current_state_index = 0


class PickAndPlaceMoveIt(object):
    def __init__(self, limb, hover_distance=0.15, verbose=True):
        self._last_known_pose = None
        self._limb_name = limb  # string
        self._hover_distance = hover_distance  # in meters
        self._verbose = verbose  # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        rospy.loginfo("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        rospy.loginfo("Enabling robot... ")

        self._robot = moveit_commander.RobotCommander()
        # This is an interface to one group of joints.  In our case, we want to use the "right_arm".
        # We will use this to plan and execute motions
        self._group = moveit_commander.MoveGroupCommander(limb + "_arm")

    def update_pose(self):
        current_pose = self._limb.endpoint_pose()
        self._last_known_pose = Pose(
            position=Point(
                x=current_pose["position"].x,
                y=current_pose["position"].y,
                z=current_pose["position"].z,
            ),
            orientation=Quaternion(
                x=current_pose["orientation"].x,
                y=current_pose["orientation"].y,
                z=current_pose["orientation"].z,
                w=current_pose["orientation"].w,
            ),
        )

    def move_to_start(self, start_pose=None):
        rospy.loginfo("Moving the {0} arm to start pose...".format(self._limb_name))
        self.gripper_open()

        # Get the current pose
        self.update_pose()
        current_pose = self._last_known_pose

        # Define the waypoints
        waypoints = [current_pose, start_pose]

        # Compute the path
        (plan, fraction) = self._group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step  # jump_threshold
        )

        # Execute the plan
        self._group.execute(plan)

        rospy.sleep(1.0)
        rospy.loginfo("Running. Ctrl-c to quit")

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr(
                "No Joint Angles provided for move_to_joint_positions. Staying put."
            )

    def gripper_open(self):
        rospy.sleep(1.0)
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        rospy.sleep(1.0)
        self._gripper.close()
        rospy.sleep(1.0)

    def _approach(self, pose):
        self.update_pose()
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance

        # Get the current pose
        current_pose = self._last_known_pose

        # Define the waypoints
        waypoints = [current_pose, approach]

        # Compute the path
        (plan, fraction) = self._group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step  # jump_threshold
        )

        # Execute the plan
        self._group.execute(plan)

    def _retract(self):
        self.update_pose()
        current_pose = self._last_known_pose
        ik_pose = Pose()
        ik_pose.position.x = current_pose.position.x
        ik_pose.position.y = current_pose.position.y
        ik_pose.position.z = current_pose.position.z
        ik_pose.orientation.x = current_pose.orientation.x
        ik_pose.orientation.y = current_pose.orientation.y
        ik_pose.orientation.z = current_pose.orientation.z
        ik_pose.orientation.w = current_pose.orientation.w

        # Increase the number of waypoints for a smoother path
        waypoints = [ik_pose] + [
            self._create_waypoint(ik_pose, z)
            for z in np.linspace(
                ik_pose.position.z, ik_pose.position.z + self._hover_distance, num=20
            )
        ]

        # Compute the path
        (plan, fraction) = self._group.compute_cartesian_path(waypoints, 0.01, 0.0)
        self._group.execute(plan)

    def _servo_to_pose(self, pose):
        self.update_pose()

        pose.position.z += self._hover_distance
        increment = delta = self._hover_distance / 10

        # start with the current pose
        waypoints = [self._last_known_pose]
        wpose = copy.deepcopy(pose)

        # Create waypoints using a list comprehension
        waypoints += [
            self._create_waypoint(wpose, waypoints[0].position.z - increment)
            for increment in np.arange(delta, self._hover_distance + 0.01, delta)
        ]

        (plan, fraction) = self._group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold
        self._group.execute(plan)

    def _create_waypoint(self, wpose, z):
        wpose = copy.deepcopy(wpose)
        wpose.position.z = z
        return wpose

    def pick(self, pose):
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose)
        self._servo_to_pose(pose)
        self.gripper_close()
        # retract to clear object
        self._retract()

    def place(self, pose):
        # servo above pose
        self._approach(pose)
        self._servo_to_pose(pose)
        self.gripper_open()
        # retract to clear object
        self._retract()


def handle_state(
    pnp,
    tfBuffer,
    overhead_orientation,
    pieceposition,
    state_name,
    transform_name,
    position_name,
):
    rospy.loginfo("Current state: %s", state_name)

    trans = tfBuffer.lookup_transform("base", transform_name, rospy.Time())
    block_pose_pick = Pose(
        position=Point(
            x=trans.transform.translation.x,
            y=trans.transform.translation.y,
            z=trans.transform.translation.z,
        ),
        orientation=overhead_orientation,
    )
    rospy.loginfo("\nPicking...")
    pnp.pick(block_pose_pick)

    block_pose_place = Pose(
        position=Point(
            x=pieceposition[position_name][0],
            y=pieceposition[position_name][1],
            z=trans.transform.translation.z,
        ),
        orientation=overhead_orientation,
    )
    rospy.loginfo("\nPlacing...")
    pnp.place(block_pose_place)

    return True


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("ik_pick_and_place_moveit")

    # Wait for the All Clear from emulator startup
    rospy.wait_for_message("/robot/sim/started", Empty)

    limb = "left"
    hover_distance = 0.15  # meters
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    pieceposition = rospy.get_param("piece_target_position_map")

    # An orientation for gripper fingers to be overhead and parallel to the obj
    overhead_orientation = Quaternion(
        x=-0.0249590815779, y=0.999649402929, z=0.00737916180073, w=0.00486450832011
    )
    # NOTE: Gazebo and Rviz has different origins, even though they are connected. For this
    # we need to compensate for this offset which is 0.93 from the ground in gazebo to
    # the actual 0, 0, 0 in Rviz.
    homepose = Pose(
        position=Point(x=0.7, y=0.150, z=0.35), orientation=overhead_orientation
    )
    pnp = PickAndPlaceMoveIt(limb, hover_distance)
    # Move to the desired starting angles

    while not rospy.is_shutdown():
        global current_state_index

        current_state = states[current_state_index]

        pnp.move_to_start(homepose)
        rospy.loginfo("Moved to start position")

        if current_state == "INIT":

            current_state_index = (current_state_index + 1) % len(states)

        elif current_state == "FIRST":

            if handle_state(
                pnp,
                tfBuffer,
                overhead_orientation,
                pieceposition,
                "FIRST",
                "p4",
                "34",
            ):
                current_state_index = (current_state_index + 1) % len(states)

        elif current_state == "SECOND":

            if handle_state(
                pnp,
                tfBuffer,
                overhead_orientation,
                pieceposition,
                "SECOND",
                "P5",
                "45",
            ):
                current_state_index = (current_state_index + 1) % len(states)

        elif current_state == "THIRD":

            if handle_state(
                pnp,
                tfBuffer,
                overhead_orientation,
                pieceposition,
                "THIRD",
                "n6",
                "25",
            ):
                current_state_index = (current_state_index + 1) % len(states)

        elif current_state == "FOURTH":

            if handle_state(
                pnp,
                tfBuffer,
                overhead_orientation,
                pieceposition,
                "FOURTH",
                "N6",
                "55",
            ):
                current_state_index = (current_state_index + 1) % len(states)

        elif current_state == "FIFTH":

            if handle_state(
                pnp,
                tfBuffer,
                overhead_orientation,
                pieceposition,
                "FIFTH",
                "p5",
                "35",
            ):
                current_state_index = (current_state_index + 1) % len(states)

        elif current_state == "SIXTH":

            if handle_state(
                pnp,
                tfBuffer,
                overhead_orientation,
                pieceposition,
                "SIXTH",
                "P6",
                "46",
            ):
                current_state_index = (current_state_index + 1) % len(states)

        elif current_state == "SEVENTH":

            if handle_state(
                pnp,
                tfBuffer,
                overhead_orientation,
                pieceposition,
                "SIXTH",
                "n1",
                "32",
            ):
                current_state_index = (current_state_index + 1) % len(states)

        elif current_state == "EIGHTH":

            if handle_state(
                pnp,
                tfBuffer,
                overhead_orientation,
                pieceposition,
                "SIXTH",
                "N1",
                "52",
            ):
                current_state_index = (current_state_index + 1) % len(states)

    return 0


if __name__ == "__main__":
    sys.exit(main())
