# README

## Backgroud
This project implements two nodes based on ROS, recognizes the position of the color block in the camera field of view, and realizes Uart communication.

## Install
 The project is based on the ros operating system, and the ros environment needs to be built first.
 
 $\blacktriangleright$ **Step one:  Creat workspace**
`mkdir -p ~/catkin_ws/src`
`cd ~/catkin_ws/src`
`catkin_init_workspace`

$\blacktriangleright$**Step two: Compile workspace**
`cd ~/catkin_ws/
catkin_make`

$\blacktriangleright$**Step three: Set environment variables**
`source devel/setup.bash`

$\blacktriangleright$**Step four: Creat Rosbag**
```
cd ~/catkin_ws/src
catkin_creat_pkg test_pkg std_msgs rospy roscpp
```

$\blacktriangleright$**Step five： Compole Rosbag**
```
cd ~/catkin_ws
catkin_make
source ~catkin_ws/devel/setup.bash
```

## Run
&#10084;  **Step one: Start the master of ros：**
```
roscore
```

&#10084; **Step two: Start the camera:**
```
roslaunch cam cam.launch
```

&#10084; **Step three: Start two node:**
```
rosrun cam color.py
rosrun cam uart_catch_car
```

