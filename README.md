# Multi-Robot-System-
The idea is to implement an object prioritization problem.  The robot is programmed to select and pick objects from a set of available objects on the map on priority. On top of that optimize the minimum time and send the control signal to the existing MPC problem to reach desired coordinates on the map.

# Problem statement
To sort some objects, there are many methods: previously manually, and now it goes in the direction of automation. Based on different criteria, the objects are identified and classified into some sectors. In this work, the color of the objects is used to differentiate the objects. An optimization problem is formed to make Lego robots take the objects with the same color and put them in a standard predefined destination.

# Overall Description (One robot)
The purpose of this work is to provide a comprehensive and automated solution for the object sorting process. Robot Operating Systems (ROS), Python, and C++ are used for this goal.
only one robot in this phase is doing the process. Firstly, the robot is in the initial position, then the camera is detecting April-Tags to define the position of the robot. After this, the camera also detects the colors to define the coordinates of different objects, then the program calculates the distances between the robot and each object. After calculating the distances and multiplying by weights, the object to be taken is determined and then the robot drives to it taking it to the desired station. The camera repeats the scanning cycle to check if there are any other objects, and if yes, the same steps are done again to deliver it to its destination depending on the color. Finally, when there is no other remaining object left, the robot comes back again to the initial position.

# Visualization
The screen is divided into three windows. The first top left one represents the image processing file and in which we can see three parameters with each object. The parameters are a weighted distance of the object from the robot which is updating continuously over time. The second parameter is the object coordinates in the frame and the third parameter shows the coordinates for the robot itself in the frame.
The second window on the top right corner is a terminal window. The third window is showing the rviz and on rviz we can see the red dots indicating the robot’s location and the purple one indicating the next location of the robot, the coordinates of this location are the coordinates of the objects to be caught and sorted. It can be visualized in the video attached at the end of the report.
![basic idea ](https://github.com/mbilalfaroq/Multi-Robot-System-/assets/75878830/eb5e7f1d-2472-40fb-9553-f224901b9772)
