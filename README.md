# Assignment 1: Robot Cleaner

The objective of these files is to simulate a Roomba vacuum cleaner. The turtle moves in a pattern to get maximum coverage of a 10mx10m room


## turtlesim_spiral_cleaner.launch
The launch file turtlesim_spiral_cleaner.launch causes the turtlesim to cover the room in a spiral pattern as shown below:

![alt tag](https://github.com/bolduc-drew/AUE893_Bolduc/blob/master/Screenshot%20from%202017-02-02%2004:23:17.png)

## turtlesim_grid_cleaner.launch 
The launch file turtlesim_grid_cleaner.launch causes the turtlesim to cover the room in a grid pattern as shown below:

![alt tag](https://github.com/bolduc-drew/AUE893_Bolduc/blob/master/Screenshot%20from%202017-02-02%2004:29:31.png)

## rosbag 2017-02-02-04-37-24.bag
The rosbag file 2017-02-02-04-37-24.bag was recorded using the following method:



roscore


In a new terminal window


rosrun turtlesim turtlesim_node

In a new terminal window


rosrun turtlesim turtle_teleop_key

In a new terminal window


rosbag record -a



and should be ran using:


roscore

In a new terminal window


rosrun turtlesim turtlesim_node

In a new terminal window


rosbag play 2017-02-02-04-37-24.bag

![alt tag](https://github.com/bolduc-drew/AUE893_Bolduc/blob/master/Screenshot%20from%202017-02-02%2004:38:40.png)

## ten_squares_grid_cleaner.launch
To run the grid cleaning simulation in gazebo, first the ten_squares_gazebo_world.launch file should be ran followed by the ten_squares_grid_cleaner.launch file
