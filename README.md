# rrt-ros
Implementation of a rapidly expanding random trees algorithm for ROS (Robot Operating System).

## Prerequisites
You need to have installed [ROS - Robot Operating System](http://www.ros.org/) in your system.

## Running and visualization
Clone the rrt-ros repository in to your preferable workspace. Then compile it using catkin as follows.

`git clone https://github.com/Malintha/rrt-ros` 

`cd rrt-ros`

`catkin_make`

Source the setup file of the rrt-ros to make the rrt-planning package visible to ros.

`source devel/setup.bash` 

Run roscore in another terminal.

`roscore`

Then run rrt node as follows.

`rosrun rrt-planning rrt`

You will be now promped to run rviz for visualization. Then run Rviz in another terminal.

`rosrun rviz rviz`

Add a marker listener to the world and enjoy the simulation!

![screenshot from 2017-10-03 14-40-21](https://user-images.githubusercontent.com/3253761/31142612-cf1bb6a2-a848-11e7-8dbe-23b005accf1e.png)
