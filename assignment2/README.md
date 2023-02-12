# Assignment 2

------------------------------------------

Parisi Daniele Martino 4670964

(Documentation link).......................................................................................................

The assignment involves a robot deployed in an indoor environment for surveillance purposes. The robot’s objective is to visit the different locations and stay there for some times. The assignment is the logical continuation of the previous one (look [here](https://github.com/DaniPari99/Experimental-Robotics-Laboratory/tree/main/assignment_1)). Now the objective is the same, but the previous software architecture must be integrated with the new one in order to actually move an urdf robot model in a simulation environment.
The assignment can be divided in 2 parts:
* **Detect the all 7 markers:** initially the robot is fixed in a location in which is plenty of 7 markers in all the directions. The markers must be all detected in order to provide environment informations to the robot for creating the topological map of the environment.
* **Navigation in the environment** Once all the 7 markers are detected and the topological map is built, the robot is ready to navigate in the environment through the locations in order to patrol the whole environment area 

## Software architecture
The following figure shows the software architecture of the assignment:

![sw_architecture drawio](https://user-images.githubusercontent.com/62515616/218319207-ad86e999-6711-47d6-a86c-64f35287d74c.png)

As we can see we have 5 nodes, but there is a sixth file: the ```helper.py``` which is not in the figure, because it is not a node, but only a simple python file usefull for the ```assignment_fsm.py``` node.

* **assignment_fsm**: is the node which implements the Finite State Machine which drives the robot through the locations of the map according to the stimuli. It uses external functions provided by the ```helper.py``` node.
* **battery_state**: is the node which simulates the battery behaviour. The battery is recharged in a randome time belonging to the range [40, 60] seconds and run out in a random time belonging to the range [180, 240] seconds. This node publishes on the topic ```/state/battery_low``` the state of the battery: it is a boolean which is **True** if the battery is low and **False** otherwise.
* **armor**: is a server already implemented which is used by the ```assignment_fsm``` through the ```helper``` node for doing manipulations or queries on the ontology.
* **marker_server**: it is a server which implements the ```RoomInformation.srv```. It receives from a client a request of type **int32** corresponding to the detected marker id and sends to the client a response which contains 4 fields: a **string** with the name of the location, two **float32** with the coordinates (x, y) of the center of the location and a **list** which contains the **connections** of that locations.
* **detect_marker**: the node is capable of performing a markers detecting algorithm which makes the robot's arm doing 2 entire 360 degrees loop: the first to detect the markers at the bottom of its field of view and the second to detect the markers at the top. It is the client node which sends requests (the id of the marker detected) to the server ```marker_server``` node and receives responses about locations informations. The node then sends to the ```assignment_fsm``` node a custom message of type ```/info/rooms``` which contains no more than the ```RoomInformation.srv``` response. These informations are used by the ```assignment_fsm``` node for building the topological map of the environment.

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
Finally you can run the launch file just typing on the terminal:
```
roslaunch assignment2 assignment.launch
```
## Robot model

![Schermata 2023-02-12 alle 18 41 35](https://user-images.githubusercontent.com/62515616/218327487-59b971bb-b243-425a-84e2-8fd5d1406d27.png)

The robot urdf model adopted is a simple 4-wheels mobile robot equipped with an hokuyo laser sensor and a 3 degrees of freedom arm composed by a rotating base connected to 2 other links through revolute joints. A camera is attached on the end effector of the arm in order to detect the aruco markers.
The JointStateController interface is used to control the arm's movement, and the PID values have been manually adjusted.

## Video demonstration



## Working hypothesis and environment

![Schermata 2023-02-12 alle 19 04 06](https://user-images.githubusercontent.com/62515616/218328759-55b079fe-d056-4e08-b437-3eaeb2ffd994.png)

