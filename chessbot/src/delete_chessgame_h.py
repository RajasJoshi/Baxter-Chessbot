#!/usr/bin/env python
import rospy
from gazebo_msgs.srv import *
from geometry_msgs.msg import *

if __name__ == "__main__":
    rospy.init_node("delete_chessboard")
    rospy.wait_for_service("gazebo/delete_model")

    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

    list_pieces = rospy.get_param("list_pieces")
    board_setup = rospy.get_param("board_setup")

    # Create a list of piece names using a list comprehension
    piece_names = [
        "%s%d" % (piece, col)
        for row in board_setup
        for col, piece in enumerate(row)
        if piece in list_pieces
    ]

    # Delete each piece using the service proxy
    for piece_name in piece_names:
        delete_model(piece_name)

    delete_model("cafe_table")
    delete_model("chessboard")
