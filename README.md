# Package for ARIAC

Detailed report containing details can be found in the current folder tree.

This package requires:
* ROS Melodic
* Gazebo 9.15+
* ARIAC and its dependencies 
* C++


- Clone the package into the workspace and build the package using,
```
$ catkin build group5_rwa4
```

- Source the workspace and then run,
```
$ roslaunch group5_rwa4 ariac.launch 
```

- In a seperate terminal run:
```
$ rosrun group5_rwa4 My_node
```


