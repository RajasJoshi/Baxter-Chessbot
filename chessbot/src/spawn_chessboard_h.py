#!/usr/bin/env python
import sys, rospy, tf, rospkg
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
from copy import deepcopy

from pick_and_place_moveit import PickAndPlaceMoveIt
from gazebo_msgs.msg import LinkStates

# http://wiki.ros.org/simulator_gazebo/Tutorials/ListOfMaterials
hover_distance = 0.1  # meters
limb = "left"

if __name__ == '__main__':
    rospy.init_node("spawn_chessboard")
    rospy.wait_for_service("gazebo/spawn_sdf_model")

    
    srvCall = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    
    # Table
    modelPath = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
    tableXml = ''
    with open(modelPath + "cafe_table/model.sdf", "r") as tableFile:
        tableXml = tableFile.read().replace('\n', '')

    tablePose=Pose(position=Point(x=0.78, y=0.3, z=0.0))
    try:
        spawnSdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        spawnSdf("cafe_table", tableXml, "/", tablePose, "world")
    except rospy.ServiceException as e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
    
    
    # ChessBoard
    orient = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
    boardPose = Pose(Point(0.4, 0.55, 0.78), orient)
    frameDist = 0.025
    modelPath = rospkg.RosPack().get_path('chessbot')+"/models/"
    
    with open(modelPath + "chessboard/model.sdf", "r") as f:
        boardXml = f.read().replace('\n', '')

    # Add chessboard into the simulation
    print srvCall("chessboard", boardXml, "", boardPose, "world")

    # Baxter
    pnp = PickAndPlaceMoveIt(limb, hover_distance)
    overhead_orientation = Quaternion(x=-0.0249590815779, y=0.999649402929, z=0.00737916180073, w=0.00486450832011)
    homepose = Pose(
        position=Point(x=0.7, y=0.150, z=0.35), orientation=overhead_orientation
    )
    pnp.move_to_start(homepose)
    
    # Add chesspieces into the simulation
    originPiece = 0.03125

    piecesXml = dict()
    listPieces = 'rnbqkpRNBQKP'
    for each in listPieces:
        with open(modelPath + each+".sdf", "r") as f:
            piecesXml[each] = f.read().replace('\n', '')

    boardSetup = ['rnbqkbnr', 'pppppppp', '********', '********', '********', '********', 'PPPPPPPP', 'RNBQKBNR']
    # boardSetup = ['r******r', '', '**k*****', '', '', '******K*', '', 'R******R']

    piecePositionMap = dict()
    pieceNames = []

    # hard coded position for spawning chess pieces
    pieceSpawnLoc = deepcopy(boardPose)
    pieceSpawnLoc.position.x = 0.6
    pieceSpawnLoc.position.y = 0.6
    pieceSpawnLoc.position.z = 0.8

    # Create the service proxy outside the loop
    spawnSdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    # Load all SDF files at once
    piecesXml = {piece: open(modelPath + piece + ".sdf", "r").read().replace('\n', '') for piece in listPieces}

    for row, each in enumerate(boardSetup):
        for col, piece in enumerate(each):
            pose = deepcopy(boardPose)
            pose.position.x = boardPose.position.x + frameDist + originPiece + row * (2 * originPiece)
            pose.position.y = boardPose.position.y - 0.55 + frameDist + originPiece + col * (2 * originPiece)
            pose.position.z += 0.018
            piecePositionMap[str(row)+str(col)]  = [pose.position.x, pose.position.y, pose.position.z-0.93] #0.93 to compensate Gazebo RViz origin difference

            if piece in piecesXml:
                try:
                    # spawn Piece
                    spawnSdf("%s%d" % (piece, col), piecesXml[piece], "/", pieceSpawnLoc, "world")
                except rospy.ServiceException as e:
                    rospy.logerr("Spawn SDF service call failed: {0}".format(e))

            if piece in listPieces:
                pieceNames.append("%s%d" % (piece,col))

                place_pose = piecePositionMap[str(row)+str(col)] 
                # Pick chess piece from hard coded position on table
                pnp.pick(Pose(position=Point(pieceSpawnLoc.position.x, pieceSpawnLoc.position.y , place_pose[2] - 0.015), orientation=overhead_orientation))
                # Place the chess piece at the defined position
                pnp.place(Pose(position=Point(place_pose[0], place_pose[1], place_pose[2] + 0.008), orientation=overhead_orientation))
                # Move to Home Pose
                pnp.move_to_start(homepose)

    rospy.set_param('board_setup', boardSetup) # Board setup
    rospy.set_param('list_pieces', listPieces) # List of unique pieces
    rospy.set_param('piece_target_position_map', piecePositionMap) # 3D positions for each square in the chessboard
    rospy.set_param('piece_names', pieceNames) # Pieces that will be part of the game
    rospy.set_param('pieces_xml', piecesXml) # File paths to Gazebo models, i.e. SDF files