# ya_robot_manipulator
Ya robot manipulator ros2 packages and other software

Software architecture:
1. Accounting system (as givven, we are not develop this stage)
2. Starage optimisation software - issues the tascs to the robot to manipulate in the storage. Here we could correct the API. In general it says us what to do - pick the container, put the item in the container, etc. It stores all the data about the items and their addresses. Here we could discuss the API.
3. ROS2 robot control system. This is what we are focused on! It says the robot how to do the tasks issued from the  level 2. I think it should be built on the actions - get to the address, get the box out of the cabinet, get the item out of the box etc. So actually we need to have a set of actions to accomplish each step and a way to combine these actions into more complex action. It should contain a bridge node to communicate with the level 2 (MQTT, web sockets etc.) and to be a client to the server/servers which will perform the action.
4. Hardware interface - controll actual hardware to move and read the position data.
