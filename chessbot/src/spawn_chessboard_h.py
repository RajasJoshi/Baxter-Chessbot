#!/usr/bin/env python
import rospy, tf, rospkg
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
from copy import deepcopy

from pick_and_place_moveit import PickAndPlaceMoveIt

# http://wiki.ros.org/simulator_gazebo/Tutorials/ListOfMaterials
hover_distance = 0.1  # meters
limb = "left"


def spawn_model(model_name, model_xml, pose):
    try:
        spawnSdf = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        spawnSdf(model_name, model_xml, "/", pose, "world")
    except rospy.ServiceException as e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))


def load_model(model_path):
    with open(model_path, "r") as f:
        return f.read().replace("\n", "")


def main():
    rospy.init_node("spawn_chessboard")
    rospy.wait_for_service("gazebo/spawn_sdf_model")

    # Table
    modelPath = rospkg.RosPack().get_path("baxter_sim_examples") + "/models/"
    tableXml = load_model(modelPath + "cafe_table/model.sdf")
    tablePose = Pose(position=Point(x=0.78, y=0.3, z=0.0))
    spawn_model("cafe_table", tableXml, tablePose)

    # ChessBoard
    orient = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
    boardPose = Pose(Point(0.3, 0.55, 0.78), orient)
    frameDist = 0.025
    modelPath = rospkg.RosPack().get_path("chessbot") + "/models/"
    boardXml = load_model(modelPath + "chessboard/model.sdf")
    spawn_model("chessboard", boardXml, boardPose)

    # Baxter
    pnp = PickAndPlaceMoveIt(limb, hover_distance)
    overhead_orientation = Quaternion(
        x=-0.0249590815779, y=0.999649402929, z=0.00737916180073, w=0.00486450832011
    )
    homepose = Pose(
        position=Point(x=0.7, y=0.150, z=0.35), orientation=overhead_orientation
    )
    pnp.move_to_start(homepose)

    # Add chesspieces into the simulation
    originPiece = 0.03125

    piecesXml = dict()
    listPieces = "rnbqkpRNBQKP"
    for each in listPieces:
        piecesXml[each] = load_model(modelPath + each + ".sdf")
    boardSetup = [
        "r*b*k*n*",
        "*p*p*p*p",
        "********",
        "********",
        "********",
        "********",
        "P*P*P*P*",
        "*N*Q*B*R",
    ]
    # boardSetup = ["r******r", "", "**k*****", "", "", "******K*", "", "R******R"]

    piecePositionMap = dict()
    pieceNames = []

    # hard coded position for spawning chess pieces
    pieceSpawnLoc = deepcopy(boardPose)
    pieceSpawnLoc.position.x = 0.6
    pieceSpawnLoc.position.y = 0.6
    pieceSpawnLoc.position.z = 0.8

    # Load all SDF files at once
    piecesXml = {
        piece: open(modelPath + piece + ".sdf", "r").read().replace("\n", "")
        for piece in listPieces
    }

    # Calculate row and column positions outside the inner loop
    row_positions = [
        boardPose.position.x + frameDist + originPiece + row * (2 * originPiece)
        for row in range(8)
    ]
    col_positions = [
        boardPose.position.y - 0.55 + frameDist + originPiece + col * (2 * originPiece)
        for col in range(8)
    ]

    # Create one Pose object before the loop
    pose = deepcopy(boardPose)

    for row, each in enumerate(boardSetup):
        for col, piece in enumerate(each):
            # Update the position of the Pose object
            pose.position.x = row_positions[row]
            pose.position.y = col_positions[col]
            pose.position.z += 0.018

            piecePositionMap[str(row) + str(col)] = [
                pose.position.x,
                pose.position.y,
                pose.position.z - 0.93,
            ]  # 0.93 to compensate Gazebo RViz origin difference

            if piece in piecesXml:

                spawn_model("%s%d" % (piece, col), piecesXml[piece], pieceSpawnLoc)

                # Create Pose objects outside the loop
                pick_pose = Pose(orientation=overhead_orientation)
                place_pose = Pose(orientation=overhead_orientation)

                if piece in listPieces:
                    pieceNames.append("%s%d" % (piece, col))

                    # Store the place position in a variable to avoid repeated dictionary lookups
                    place_position = piecePositionMap[str(row) + str(col)]

                    # Update the positions of the Pose objects
                    pick_pose.position = Point(
                        pieceSpawnLoc.position.x,
                        pieceSpawnLoc.position.y,
                        place_position[2] - 0.015,
                    )
                    place_pose.position = Point(
                        place_position[0], place_position[1], place_position[2] + 0.008
                    )

                    # Pick and place the chess piece
                    pnp.pick(pick_pose)
                    pnp.place(place_pose)

                    # Move to Home Pose
                    pnp.move_to_start(homepose)

    rospy.set_param("board_setup", boardSetup)  # Board setup
    rospy.set_param("list_pieces", listPieces)  # List of unique pieces
    rospy.set_param(
        "piece_target_position_map", piecePositionMap
    )  # 3D positions for each square in the chessboard
    rospy.set_param("piece_names", pieceNames)  # Pieces that will be part of the game
    rospy.set_param(
        "pieces_xml", piecesXml
    )  # File paths to Gazebo models, i.e. SDF files


if __name__ == "__main__":
    main()
