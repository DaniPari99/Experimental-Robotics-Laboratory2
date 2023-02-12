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
