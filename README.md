# Goaliebot
A robot that blocks a ball from passing a goal line

## To Run the Robot
### On Turtlebot
Once remoted in to the turtlebot, set the ROS_MASTER_URI and ROS_IP variables in the ~/.bashrc file to the turtlebot's IP address
Then run the following:
`source ~/.bashrc`
`roslaunch turtlebot3_bringup turtlebot3_robot.launch`

### On Workstation
After placing the goaliebot folder in the src directory of your catkin workspace, change the ROS_MASTER_URI and ROS_IP values in the devel/setup.bash file of your catkin workspace to be the turtlebots IP and the workstations IP respectively. Then, in the root directory of your catkin workspace, run:
`source devel/setup.bash`
`rosrun goaliebot move_turtlebot.py`

## To Run in Simulation
After placing the goaliebot folder in the src directory of your catkin workspace, change the ROS_MASTER_URI and ROS_IP values in the devel/setup.bash file of your catkin workspace to both be the IP of your workstation.
To start the gazebo simulation, run the following:
`roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch`
This assumes that you already have ROS properly installed.
In the simulation, place a ball in front of the turtlebot. Be sure that the ball is more than 4.5 squares away from the turtlebot. This will ensure that the turtlebot will not immediately detect the ball once you run the script.
In a new terminal, run the following command:
`rosrun goaliebot move_turtlebot.py`
Now apply a force to the ball that it travels in the general direction of the turtlebot.
The turtlebot should detect the ball and move to a position to stop the ball.