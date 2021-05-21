There are three files in this folder:
    - teleoperation.cpp contains a skeleton for a C++ program, and a function to get the key press, which should (hopefully work for your OS). If it doesn't, please email Michael with specific failure modes.
    - teleoperation.py contains a skeleton for a Python program, and a function to get the key press, which should (hopefully work for your OS). If it doesn't, please email Michael with specific failure modes.
    - test_node_standin.py is a ROS node that you can run, which will advertise the service you need to call to indicate that the Turtlebot3 has reached its goal location. 

For the teleoperation files, you'll need to just take the functions for getting the keypress and use them in your ROS node.  
For the test_node_standin, put it in your package, make sure that it is executable ("sudo chmod +x test_node_standin.py" is the command on Linux), build your package, then run it with rosrun <package_name> test_node_standin.py
The node will advertise the /csci_5551/goal_reached service, which you can call.
