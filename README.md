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
Python

#!/usr/bin/env python3


import rospy
from std_msgs.msg import Float64
import math


def sin_publisher():
   rospy.init_node('sin_publisher', anonymous=True)
   pub = rospy.Publisher('sin_topic', Float64, queue_size=10)
   rate = rospy.Rate(10) 
  
   start_time = rospy.get_time() 


   while not rospy.is_shutdown():
       current_time = rospy.get_time() 
       t = current_time - start_time   # Calculate elapsed time since start
       sin_value = math.sin(t)         #  sin(t)
       rospy.loginfo(f"Publishing sin(t) at time {t:.2f}s: {sin_value}")
       pub.publish(sin_value)
       rate.sleep()


if __name__ == '__main__':
   try:
       sin_publisher()
   except rospy.ROSInterruptException:
       pass








3. Implement the Code for exp_subscriber.py:
#!/usr/bin/env python3


import rospy
from std_msgs.msg import Float64
import math


B = 1.0  # constant B value
start_time = None 


def callback(sin_value):
   global start_time
   if start_time is None:
       start_time = rospy.get_time() 
 
   current_time = rospy.get_time()  
   t = current_time - start_time    
   exp_value = math.exp(B * t)       # Calculate e^(Bt)
   output_y = exp_value * sin_value.data  # Output
 
   # Publish the calculated values of y and e^(Bt)
   y_publisher.publish(output_y)
   exp_publisher.publish(exp_value)


   # Log the values
   rospy.loginfo(f"Time: {t:.2f}s, sin(t): {sin_value.data}, e^(Bt): {exp_value}, y: {output_y}")


def exp_subscriber():
   global y_publisher, exp_publisher
   rospy.init_node('exp_subscriber', anonymous=True)


   # Initialize the publishers for the calculated y value and e^(Bt)
   y_publisher = rospy.Publisher('/calculated_y', Float64, queue_size=10)
   exp_publisher = rospy.Publisher('/exp_topic', Float64, queue_size=10)


   # Subscriber to the /sin_topic
   rospy.Subscriber('/sin_topic', Float64, callback)
   rospy.spin()


if __name__ == '__main__':
   try:
       exp_subscriber()
   except rospy.ROSInterruptException:
       pass

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

<launch>
 <!-- Launch the sin publisher node -->
 <node pkg="sin_exp_eqn" type="sin_publisher.py" name="sin_publisher" output="screen" />


 <!-- Launch the exp subscriber node -->
 <node pkg="sin_exp_eqn" type="exp_subscriber.py" name="exp_subscriber" output="screen" />
</launch>











6. Run multiple Nodes using roslaunch
roslaunch sin_exp_eqn launchfile.launch

6. Save data with rosbag

rosbag record -O sin_exp_data.bag /sin_topic /exp_topic

Will save as sin_exp_data.bag file




7. Sow data with bagpy
Create readbag.py file to see the readbag data


import bagpy
from bagpy import bagreader
import pandas as pd


b = bagreader('sin_exp_data.bag')


print(b.topic_table)


sin_topic_csv = b.message_by_topic('/sin_topic')
exp_topic_csv = b.message_by_topic('/exp_topic')
calculated_y_csv = b.message_by_topic('/calculated_y')


df_sin = pd.read_csv(sin_topic_csv)
df_exp = pd.read_csv(exp_topic_csv)
df_y = pd.read_csv(calculated_y_csv)


# Print the first 5 rows of the dataframe
print('df_sin',df_sin.head())
print('df_exp',df_exp.head())
print('df_y',df_y.head())















We can run this code in CPP too.

Now let’s dive into this.

The sin_publisher.py will be like this



#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <cmath>


int main(int argc, char **argv)
{
   // Initialize ROS node
   ros::init(argc, argv, "sin_publisher");
   ros::NodeHandle nh;


  
   ros::Publisher sin_pub = nh.advertise<std_msgs::Float64>("sin_topic", 10);


  
   ros::Rate rate(10);


  
   double start_time = ros::Time::now().toSec();


   while (ros::ok())
   {
      
       double current_time = ros::Time::now().toSec();
       double t = current_time - start_time;


      
       double sin_value = std::sin(t);


       //publish the sine value
       ROS_INFO("Publishing sin(t) at time %.2fs: %f", t, sin_value);


       std_msgs::Float64 msg;
       msg.data = sin_value;
       sin_pub.publish(msg);


       rate.sleep();
   }




   return 0;
}










& the  sin_publisher.py will be like this


#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <cmath>  // for exp




ros::Publisher y_publisher;
ros::Publisher exp_publisher;
double B = 1.0;  // constant B value
double start_time = -1.0; 


void callback(const std_msgs::Float64::ConstPtr& sin_value)
{
   if (start_time < 0)
   {
       // Initialize start_time only on the first callback
       start_time = ros::Time::now().toSec();
   }


   double current_time = ros::Time::now().toSec();
   double t = current_time - start_time;  // elapsed time


   double exp_value = std::exp(B * t);  //  e^(Bt)
   double output_y = exp_value * sin_value->data;  //  y
   // messages to publish
   std_msgs::Float64 y_msg;
   std_msgs::Float64 exp_msg;


   y_msg.data = output_y;
   exp_msg.data = exp_value;


   //  calculated values of y and e^(Bt)
   y_publisher.publish(y_msg);
   exp_publisher.publish(exp_msg);


   // Log the values
   ROS_INFO("Time: %.2fs, sin(t): %f, e^(Bt): %f, y: %f", t, sin_value->data, exp_value, output_y);
}


int main(int argc, char **argv)
{
   // Initialize the ROS node
   ros::init(argc, argv, "exp_subscriber");
   ros::NodeHandle nh;


   // Initialize the publishers for  calculated y value and e^(Bt)
   y_publisher = nh.advertise<std_msgs::Float64>("/calculated_y", 10);
   exp_publisher = nh.advertise<std_msgs::Float64>("/exp_topic", 10);


   ros::Subscriber sin_subscriber = nh.subscribe("/sin_topic", 10, callback);


   ros::spin();


   return 0;
}



