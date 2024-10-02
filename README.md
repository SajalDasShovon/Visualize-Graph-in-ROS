# Visualize-Graph-in-ROS


Create a ROS package that will visualize y = e^(Bt) * sin (t)


1. Create a New ROS Package:
Create a package called sin_exp_eqn with dependencies on rospy and std_msgs.


cd ~/catkin_ws/src
catkin_create_pkg sin_exp_eqn rospy std_msgs
cd ..
catkin_make
source devel/setup.bash


2. Write the ROS Node Script:
Create a Python script named sin_publisher.py &  exp_publisher.py inside the scripts folder of your package:


cd ~/catkin_ws/src/sin_exp_eqn
mkdir scripts
touch scripts/sin_publisher.py
chmod +x scripts/sin_publisher.py


touch scripts/exp_publisher.py
chmod +x scripts/exp_publisher.py






3. Implement the Code for sin_publisher.py:









3. Implement the Code for exp_subscriber.py:

3. Build and Run the Node
Build the Package:
Go back to the workspace and build your package:


cd ~/catkin_ws
catkin_make
source devel/setup.bash

Run the Node:
Run your node with the command:

rosrun sin_exp_eqn exp_subscriber.py
rosrun sin_exp_eqn sin_publisher.py

4. Visualize Using rqt_graph or rqt_plot
Launch rqt_graph:
Open a new terminal and run:
bash
Copy code
rqt_graph
Launch rqt_plot:


5. Build and Run the Node with roslaunch

Create launch file



6. Run multiple Nodes using roslaunch
roslaunch sin_exp_eqn launchfile.launch

6. Save data with rosbag

rosbag record -O sin_exp_data.bag /sin_topic /exp_topic

Will save as sin_exp_data.bag file




7. Sow data with bagpy
Create readbag.py file to see the readbag data










We can run this code in CPP too.

