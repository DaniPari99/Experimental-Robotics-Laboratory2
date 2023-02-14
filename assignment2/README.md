# Assignment 2

------------------------------------------

Parisi Daniele Martino 4670964

[code documentation](https://danipari99.github.io/Experimental-Robotics-Laboratory2/)

The assignment involves a robot deployed in an indoor environment for surveillance purposes. The robot’s objective is to visit the different locations and stay there for some times. The assignment is the logical continuation of the previous one (look [here](https://github.com/DaniPari99/Experimental-Robotics-Laboratory/tree/main/assignment_1)). Now the objective is the same, but the previous software architecture must be integrated with the new one in order to actually move an urdf robot model in a simulation environment.
The assignment can be divided in 2 parts:
* **Detect the all 7 markers:** initially the robot is fixed in a location in which is plenty of 7 markers in all the directions. The markers must be all detected in order to provide environment informations to the robot for creating the topological map of the environment.
* **Navigation in the environment** Once all the 7 markers are detected and the topological map is built, the robot is ready to navigate in the environment through the locations in order to patrol the whole environment area 

## Software architecture
The following figure shows the software architecture of the assignment:


![components_diagram drawio](https://user-images.githubusercontent.com/62515616/218784887-80b0e851-5e2a-4e44-b100-43566e0d8475.png)


As we can see we have 8 nodes, but there is a sixth file: the ```helper.py``` which is not in the figure, because it is not a node, but only a simple python file usefull for the ```assignment_fsm.py``` node. I implemented only 3 of them:

* **assignment_fsm**: is the node which implements the Finite State Machine which drives the robot through the locations of the map according to the stimuli. It uses external functions provided by the ```helper.py``` node. It is interfaced with aRMOR server in order to make request for modifying the load ontology or for making queries on it. It is also in contact with the ```battery_state``` in order to lnow the state of the battery every moment. It is subscribed to the ```/robot/joint_states``` through gazebo in order to know the position of the joints in every moment. It is in contact with ```detect_marker``` node in order to subscribe to a topic for retrieving locations' informations. Finally it is also in contact to move_base through an action server for sending the goal to move_base and retrieving informations about the state of the goal.
* **battery_state**: is the node which simulates the battery behaviour. The battery is recharged in a randome time belonging to the range [40, 60] seconds and run out in a random time belonging to the range [180, 240] seconds. This node publishes on the topic ```/state/battery_low``` the state of the battery: it is a boolean which is **True** if the battery is low and **False** otherwise.
* **marker_server**: it is a server which implements the ```RoomInformation.srv```. It receives from a client a request of type **int32** corresponding to the detected marker id and sends to the client a response which contains 4 fields: a **string** with the name of the location, two **float32** with the coordinates (x, y) of the center of the location and a **list** which contains the **connections** of that locations.
* **detect_marker**: the node is capable of performing a markers detecting algorithm which makes the robot's arm doing 2 entire 360 degrees loop: the first to detect the markers at the bottom of its field of view and the second to detect the markers at the top. The motion of the arm is performed by publishing in to 3 different topics ```/robot/joint(i)_position_controller/command``` used for publishing the goal configuration of the i-th joint. This node is also subscribed to a gazebo topic ```/robot/camera1/image_raw``` for detecting the marker's ID. It is the client node which sends requests (the id of the marker detected) to the server ```marker_server``` node and receives responses about locations informations. The node then sends to the ```assignment_fsm``` node a custom message of type ```/info/rooms``` which contains no more than the ```RoomInformation.srv``` response. These informations are used by the ```assignment_fsm``` node for building the topological map of the environment.

## Temporal diagram
For sake of completeness the following figure shows the temporal diagram of the software architecture:

![sw_architecture drawio (1)](https://user-images.githubusercontent.com/62515616/218324264-359559c8-71f5-4bc5-9d5f-73743a5d3620.png)

The diagram shows that the ```battery_state```is always in contact with the ```assignment_fsm``` node through a pub/sub approach, indeed it makes the system going back to the ```assignment_fsm``` immediately in order to make the robot recharging. The same for ```detect_marker``` node wich is in contact with ```assignment_fsm``` node through a pub/sub approach in order for the ```assignment_fsm``` to be always aware about the publishing locations informations.
The other software components, which are servers, are always active, but they work only if a client sends a request to them. ```assignment_fsm``` sends requests to armor in order to build the topological map at the beginning and then to update it and to do query on it. ```detect_marker``` instead sends requests to ```marker_server``` node in order to retrieve location informations for sending them to ```assignment_fsm```.

## States diagram
The following figure shows the states diagram of the Finite State Machine of the robot behaviour:

![Diagramma senza titolo drawio-2-3](https://user-images.githubusercontent.com/62515616/202918060-40c54de6-60bf-485f-a580-f060d253ae70.png)

As we can see we have 4 states:
* ```WAIT```: this state is just executed at the beginning and it waits until the ontology map to be loaded. As soon as the ontology is loaded, the transition **loaded** is trigguered.
* ```SLEEP```: this state is executed in order to recharge the battery of the robot. As soon as the battery goes high, the transition **rested** is trigguered.
* ```DECIDE```: this state is executed in order to decide the next location to be visited. As soon as the next location is chosen, the transition **decided** is trigguered, instead if the battery goes low, the transition **tired** is trigguered.
* ```VISIT```: this state is executed in order to visit the chosen location. As soon as the chosen location is visited, the transition **visited** is trigguered, instead if the battery goes low, the transition **tired** is trigguered.

For sake of completeness and robustness I also implemented the so called transitions loop: the transition which remains in the current state whenever they are trigguered.

## Custom messages and services used
* **InfoRoom.msg**: it reproduces the object room, with the name, the coordinates of the center and the connections
* **RoomConnections.msg**: it reproduces the connections of the rooms through the associated door
* **RoomInformations.srv**: a service whose request is the id number of the marker and the response instead is the location informations related to that marker.

## Installation and running

You need to have aRMOR package cloned in the SRC folder of your workspace, if you don't have follow the steps in the following link:

[aRMOR installation](https://github.com/EmaroLab/armor/issues/7)

Now you need to modify the armor_api folder by adding in the ```armor_manipulation_client``` node the following function:

```
def disj_all_inds(self,ind_list):
        try:
            res = self._client.call('DISJOINT', 'IND', '', ind_list)

        except rospy.ServiceException as e:
            raise ArmorServiceCallError("Service call failed upon adding individual {0} to class {1}: {2}".format(ind_name, class_name, e))

        except rospy.ROSException:
            raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")

        if res.success:
            return res.is_consistent
        else:
            raise ArmorServiceInternalError(res.error_description, res.exit_code)
```
Where ```ind_list``` is the list of all individuals of the ontology.

I did this modification, because I faced some problems with the pre-existed function ```disj_inds_of_class(self, class_name)``` of ```armor_manipulation_client.py```.

### How to run
In order to run the application with a launch file you need to install ```x-term``` with the following steps:
```
sudo apt-get update
sudo apt-get -y install xterm
```
Clone the repository in the src folder of your workspace by running:
```
git clone https://github.com/DaniPari99/Experimental-Robotics-Laboratory2.git
```
Change the directory in order to be in the src/assignment2/scripts and run:
```
chmod +x assignment_fsm.py
chmod +x helper.py
chmod +x battery_state.py
```
Build the ROS workspace by running  ```catkin_make``` from your root folder

Finally you can run the launch file just typing on the terminal:
```
roslaunch assignment2 assignment.launch
```
## Robot model

![Schermata 2023-02-12 alle 18 41 35](https://user-images.githubusercontent.com/62515616/218327487-59b971bb-b243-425a-84e2-8fd5d1406d27.png)

The robot urdf model adopted is a simple 4-wheels mobile robot equipped with an hokuyo laser sensor and a 3 degrees of freedom arm composed by a rotating base connected to 2 other links through revolute joints. A camera is attached on the end effector of the arm in order to detect the aruco markers.
The JointStateController interface is used to control the arm's movement, and the PID values have been manually adjusted.

## Video demonstration



https://user-images.githubusercontent.com/62515616/218697303-2599fdde-4ae1-4cc1-a90e-4185a3e3f152.mp4



In this video we can see the 2 most important phases of the robot's behavior:

**1)** Initially the robot is fixed at the position (x = -6.0, y = 11.0) and from the camera of ```Rviz``` we can see that are performed 2 entire loops in order to detect all the 7 markers. From the ```assignment_fsm``` terminal we can see the print of the correspondent location informations for each marker. After the seventh marker is detected the video is switched from Rviz to Gazebo in order to see the robot's movement through the locations.


**2)** In the second phase we can see the robot initially looking for a way of out from the initial location. It requires sometime to exit from it, because it has to build the map of the environment and localize it within the environment. Once it has understood how to do, it reach the recharging station ready for patrolling the other 6 locations.

## environment
The environment is shown in the following figure:

![Schermata 2023-02-12 alle 19 04 06](https://user-images.githubusercontent.com/62515616/218328759-55b079fe-d056-4e08-b437-3eaeb2ffd994.png)

This is the Gazebo 3D simulation environment which is represented in the [worlds](https://github.com/DaniPari99/Experimental-Robotics-Laboratory2/tree/main/assignment2/worlds) directory. 

It is composed by 4 rooms, 2 corridors and a recharging room where the robot is spawned at the beginning. actually the robot starts in an additional room directly connected to the recharging room 'E', where it finds 7 markers which he has to detect in order to collect informations about the 7 locations of the topological map. I sligthly modified the Gazebo file of the environment: I changed the color of the Aruco markers from green to white for increasing the contrast, in order to facilitate the detecting operation of the robot. 

## Working hypothesis
* The robot at the beginning is spawned in the starting location (x = -6.0, y = 11.0). It remains in this position until all the 7 markers are detected and the topological map built.
* The robot moves the first time always to the recharging station.
* After that the robot can move in a new location chosen with the following priority algorithm:
```
    if there are urgent reachable rooms:
      move to the most urgent
    else:
      move to a corridor
```
* Once the robot reaches the center of the chosen location, it stops for the time needed to accomplish an entire 360 degrees loop to declare the rocation visited.
* If the battery goes low, it stops every operation it was doing  in order to point to the recharging location and wait the time to have a full charge.

## System's limitations
* The detecting markers algorithm does not work every time, but with a good percentage higher than 80%
* The battery simulation mechanism is designed simply to a continuing charging and running out without focus on the moment in which the timer of the battery starts. But I chose 2 different timers: one for the recharging time which is lower [40, 60] seconds which makes the robot coming back at least to the recharging station in time wherever it is; the second one for the running out which is higher [180, 240] seconds.

## Possible improvements
* the detecting markers algorithm can be improved in order to reach all the 7 markers at any time, maybe by performing upper and down loops until all the 7 markers are detected
* The battery simulation mechanism can be improved in order to start the recharging timer only when the robot has reached the recharging station. This can be done by subscribing the assignment_fsm node to ```/odometry``` topic in such a way that when the robot position is equal to the coordinates of the recharging station, a global variable is set to True, otherwise to False. When the global variable is true the timer can start, instead if the global variable was False, the timer has to wait until the global variable becomes True (recharging station reached).

## Authors and contacts

[Parisi Daniele Martino](https://github.com/DaniPari99) (4670964)

s4670964@studenti.unige.it
