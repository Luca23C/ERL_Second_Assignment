Experimental Robot Laboratory - Second assignment
==================
Andrea Scorrano 6463777   
Luca Cornia 6467737   
Irina Grishkovets 7879818

**N.B: Please switch to the *consegna* branch after cloning the repository** 

Develop a ROS package that lets a mobile robot endowed with a camera and a laser scanner move the robot to
the waypoint corresponding to the lowest ID.
The four waypoints are:


WP 0: x = -7.0, y =-1.5

WP 1: x = -3.0, y = -8.0

WP 2: x = 6.0, y = 2.0

WP 3: x = 7.0, y = -5.0

## How to Run the code
In order to start the whole simulation, open three terminals.
To run the simulation of the enviroment, it's needed to write le following line:

```

ros2 launch robot_urdf gazebo_assignment.launch

```
To run the navigation sistem run on a new terminal:
```
ros2 launch nav2_bringup navigation_launch.py 
 
```

To run the launch file containing the domain and the slam toolbox sistem run: 

```
ros2 launch planning planning_launch.py 
```

To run all the nodes  move_cmd, detect_cmd, move_to_lowest_id, service_node:
```
ros2 launch planning action_nodes_launch.py 

```

In order to generate the plan open a new terminal and run this:
```
ros2 run plansys2_terminal plansys2_terminal

```
Than, set the instances,the predicate of the proble and the goal:
```
set instance r1 robot
set instance w0 waypoint
set instance w1 waypoint
set instance w2 waypoint
set instance w3 waypoint
set instance center waypoint
set instance m marker

set predicate (at r1 center)

set predicate (connected center w0)
set predicate (connected w0 w1)
set predicate (connected w1 w2)
set predicate (connected w2 w3)

set goal (and (done r1 w0 w1 w2 w3))
get plan

```
Once the plan is generated type "run" on this terminal


## Plan generated by plansys2 ##
```
0	(move r1 center w0)	10
10.001	(detect r1 w0 m)	10
10.002	(move r1 w0 w1)	10
20.003	(detect r1 w1 m)	10
20.004	(move r1 w1 w2)	10
30.005	(detect r1 w2 m)	10
30.006	(move r1 w2 w3)	10
40.007	(detect r1 w3 m)	10
50.008	(move_to_lowest_id r1 w0 w1 w2 w3 m)	10
```
## Component Diagram of the whole system ##

![componentDiagram](https://github.com/user-attachments/assets/88d465c2-7234-4967-b66e-1d87f24f34c5)

### Move Node ###
The node implements a ROS2 action called `MoveAction` to navigate a robot to predefined waypoints using the Plansys2 framework. 
It tracks the current position, calculates the distance to the goal, and sends navigation commands to the Nav2 action server while providing action status updates and feedback.

### Detect Node ###
This node defines a ROS2 action node `Detect` that interacts with a service (`/get_map_data`) to retrieve map data and updates a matrix with marker information (ID, x, y). 
It publishes the updated matrix to a topic and provides progress feedback, simulating a detection process.

## Detection service Node ##
The service node `detect_service_node` is located within the *ros_2_aruco* package.   
This node allow us to activate the task for detecting the aruco marker. Once the marker is detected the z-angular velocity is set to zero and we save the id of the marker and the x and y position of the robot.
Lastly, this service will provide this information to the `Detect` node.


### Move to the lowest id Node ###
This ROS2 node receives a matrix from a topic, selects the row with the lowest ID, extracts the coordinates, and sends the robot to that waypoint using `nav2_msgs::action::NavigateToPose`. 
It manages the robot's state with continuous feedback and position updates. Improvements could include thread safety, configurable parameters, and more robust validations.

## Results ##
In this section, it is possible to see the entire simulation running.   
Here we can see a robot that navigate into the environment by using SLAM algorithm. The robot does not know a priori the environment, but it knows the waypoints were are located the four marker.   
We can see that the robot visit all the four waypoint and detect the aruco markers. Once this objective is achieved, it goes towards the marker with the lowest id.

https://github.com/user-attachments/assets/b7a744b9-a100-432d-8726-78049850707d

## Problems encountered and possible improvements ##
During our test we noticed that exactly the same simulation works better on the native linux instead of docker or virtual machine.   
Additionally, the SLAM algorithm for the navigation sometimes does not work properly.To improve this aspect, the use of another type of navigation algorithm like bug0 could provide a constant result, leading to a more stable and consistent achievement of the objectives.