# Baxter Chessbot

Baxter Chessbot is a project that uses the Baxter Robot from Rethink Robotics to play a game of chess. The robot is capable of recognizing the state of the game, making its own moves, and physically moving the pieces on the board.

## Features

- **Chess Engine Integration**: The project integrates with a chess engine to determine the best move to make based on the current state of the game.
- **Computer Vision**: The robot uses computer vision to recognize the state of the chess board and the positions of the pieces.
- **Physical Interaction**: The robot can physically pick up and move the chess pieces on the board.

## Dependencies

- ROS (Robot Operating System)
- Baxter SDK
- OpenCV
- Python-Chess

## Installation

1. Install ROS and the Baxter SDK on your system.
2. Clone this repository into your catkin workspace.
3. Install the required Python packages: `pip install opencv-python python-chess`.
4. Build your catkin workspace with `catkin_make`.
5. Source your workspace's `setup.bash` file: `source devel/setup.bash`.

## Usage

1. Run the Baxter Chessbot script with `rosrun your_package_name chessbot.py`.
2. The robot will start a new game of chess. You can make your move on the physical board, and the robot will recognize your move and make its own move in response.

## Contributing

Contributions are welcome! Please read the [contributing guidelines](CONTRIBUTING.md) before getting started.

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details.