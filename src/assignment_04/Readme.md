# Assignment 4: Mobile Base with Differential Drive, Camera and Hokuyo 

The objective of this assignment is to control the mybot, using PID, to accelerate when there is no obstacle in the front and should decelerate when near the
corner or there is an obstacle in the front. 

The mybot should move through a ractrack created in gazebo as fast as possible

## Gazebo World
The track launch file is mybot_world.launch and should be run from AUE893_Bolduc/src/mybot_gazebo/worlds using the command:
roslaunch mybot_gazebo mybot_world.launch

![alt tag](https://github.com/bolduc-drew/AUE893_Bolduc/blob/master/src/assignment_04/raceTrack.png)

## mybot PID Controller
The PID script is turtlebot_app.py and should be run from AUE893_Bolduc/src/assignment_04/scripts using the command:

### First cd into proper directory:
cd AUE893_Bolduc/src/assignment_04/scripts

### Then Execute Command:
python turtlebot_app.py

Place obstacles, such as a cube and cabinet (seen below), in the map to test obstacle avoidance
![alt tag](https://github.com/bolduc-drew/AUE893_Bolduc/blob/master/src/assignment_04/raceTrack_obstacles.png)
