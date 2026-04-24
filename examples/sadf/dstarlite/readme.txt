./scenario_filter.py top.xml self_report top_scenario.xml



export TURTLEBOT3_MODEL=burger

roslaunch turtlebot3_gazebo my_turtlebot3_zigzag_cylinder.launch


topic publiser
/scan [sensor_msgs/LaserScan]
/odom [nav_msgs/Odometry]

topic subscriber: 

/cmd_vel [geometry_msgs/Twist]



g++ -I. -I/home/mohammad/ForSyDe-SystemC/src -I/opt/ros/noetic/include -L/opt/ros/noetic/lib -Wl,-rpath=/opt/ros/noetic/lib main.cpp -o main -lroscpp -lrosconsole -lrostime -lroscpp_serialization -lsystemc -lm -std=c++17 -DFORSYDE_INTROSPECTION -DFORSYDE_SELF_REPORTING