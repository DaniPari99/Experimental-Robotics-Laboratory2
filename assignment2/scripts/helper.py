#! /usr/bin/env python
"""
.. module:: helper
   :platform: Unix
   :synopsis: Python module for helping the finite stete machine module

.. moduleauthor:: Daniele Martino Parisi

Subscribes to:
    'state/battery_low': where battery_state publishes the battery state

Clients:
        :'controller': for implementing the visit of a room

        :'client': for connecting with Armor server

The node implements a class which contains a lot of functions that can help the finite state machine node in performing its task.
"""

import rospy
import random
import time
import itertools

from armor_api.armor_client import ArmorClient
from os.path import dirname, realpath
from std_msgs.msg import Bool
from assignment_1.srv import *
from threading import Lock
from actionlib import SimpleActionClient

from std_msgs.msg import Float64

from assignment2.msg import RoomConnection, InfoRoom

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

path = dirname(realpath(__file__))
path = path + "/../"
global room_list, door_list, door_list_result, list_ind, client, ontology_loaded, base_link_config
room_list = []
door_list = []
door_list_result = []
#ind_list = []
list_ind = []
ontology_loaded = False
location_info = []
base_link_config = 0.0

client = ArmorClient("test5","ontology5")



# Initializing with buffered manipulation and reasoning
client.utils.load_ref_from_file(path + "topological_map.owl", "http://bnc/exp-rob-lab/2022-23", True, "PELLET", True, False)

next_location_obj = 0

LOG_TAG = 'behaviour'

# A class to simplify the implementation of a client for ROS action servers. It is used by the `InterfaceHelper` class.
class ActionClientHelper:
    # Class constructor, i.e., class initializer. Input parameters are:
    #  - `service_name`: it is the name of the server that will be invoked by this client.
    #  - `action_type`: it is the message type that the server will exchange.
    #  - `done_callback`: it is the name of the function called when the action server completed its computation. If
    #     this parameter is not set (i.e., set to `None`), then only the `self._done_callback` function will be
    #     called when the server completes its computation.
    #  - `feedback_callback`: it is the name of the function called when the action server sends a feedback message. If
    #    this parameter is not set (i.e., set to `None`), then only the `self._feedback_callback` functions will be
    #    called when the server sends a feedback message.
    #  - `mutex`: it is a `Lock` object synchronised with the `done_callback` and `feedback_callback`. If it is not set
    #    (i.e., set to `None`), then a new mutex instance is considered. Set this variable if you want to extends the
    #    synchronization with other classes.
    def __init__(self, service_name, action_type, done_callback=None, feedback_callback=None, mutex=None):
        # Initialise the state of this client, i.e.,  `_is_running`, `_is_done`, and `_results`.
        self.reset_client_states()
        # Set the name of the server to be invoked.
        self._service_name = service_name
        # Get or create a new mutex.
        if mutex is None:
            self._mutex = Lock()
        else:
            self._mutex = mutex
        # Instantiate a simple ROS-based action client.
        self._client = SimpleActionClient(service_name, action_type)
        # Set the done and feedback callbacks defined by the class using this client.
        self._external_done_cb = done_callback
        self._external_feedback_cb = feedback_callback
        # Wait for the action server to be alive.
        self._client.wait_for_server()

    # Start the action server with a new `goal`. Note this call is not blocking (i.e., asynchronous performed).
    def send_goal(self, goal):
        # A new goal can be given to the action server only if it is not running. This simplification implies that
        # within the ROS architecture no more than one client can use the same server at the same time.
        if not self._is_running:
            # Start the action server.
            self._client.send_goal(goal,
                                   done_cb=self._done_callback,
                                   feedback_cb=self._feedback_callback)
            # Set the client's states.
            self._is_running = True
            self._is_done = False
            self._results = None
        else:
            warn_msg = 'Warning send a new goal, cancel the current request first!'
            rospy.logwarn(self.tag_log(warn_msg, LOG_TAG))

    # Stop the computation of the action server.
    def cancel_goals(self):
        # The computation can be stopped only if the server is actually computing.
        if self._is_running:
            # Stop the computation.
            self._client.cancel_all_goals()
            # Reset the client's state.
            self.reset_client_states()
        else:
            warn_msg = 'Warning cannot cancel a not running service!'
            rospy.logwarn(self.tag_log(warn_msg, LOG_TAG))

    # Reset the client state variables stored in this class.
    def reset_client_states(self):
        self._is_running = False
        self._is_done = False
        self._results = None

    # This function is called when the action server send some `feedback` back to the client.
    def _feedback_callback(self, feedback):
        # Acquire the mutex to synchronise the computation concerning the `feedback` message with the other nodes of the architecture.
        self._mutex.acquire()
        try:
            # Eventually, call the method provided by the node that uses this action client to manage a feedback.
            if self._external_feedback_cb is not None:
                self._external_feedback_cb(feedback)
            # Uncomment below to log information.
            # rospy.loginfo(anm.tag_log(f'`{self._service_name}` action server provide feedback: {feedback}.', LOG_TAG))
        finally:
            # Realise the mutex to (eventually) unblock ROS-based thread waiting on the same mutex.
            self._mutex.release()

    # This function is called when the action server finish its computation, i.e., it provides a `done` message.
    def _done_callback(self, status, results):
        # Acquire the mutex to synchronise the computation concerning the `done` message with the other nodes of the architecture.
        self._mutex.acquire()
        try:
            # Set the client's state
            self._is_running = False
            self._is_done = True
            self._results = results
            # Eventually, call the method provided by the node that uses this action client to manage a result.
            if self._external_done_cb is not None:
                self._external_done_cb(status, results)
            # Uncomment below to log information.
            # log_msg = f'`{self._service_name}` done with state `{self._client.get_state_txt()}` and result: {results}.'
            # rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        finally:
            self._mutex.release()

    # Get `True` if the action server finished is computation, or `False` otherwise.
    # Note that use this method should do it in a `self._mutex` safe manner.
    def is_done(self):  # they should be mutex safe
        return self._is_done

    # Get `True` if the action server is running, or `False` otherwise.
    # A note that use this method should do it in a `self._mutex` safe manner.
    def is_running(self):
        return self._is_running

    # Get the results of the action server, if any, or `None`.
    def get_results(self):
        if self._is_done:
            return self._results
        else:
            log_err = f'Error: cannot get result for `{self._service_name}`.'
            rospy.logerr(self.tag_log(log_err, LOG_TAG))
            return None

        # Function used to label each log with a producer tag.
    def tag_log(self, msg, producer_tag):
        return '@%s>> %s' % (producer_tag, msg)



class HelperInterface:
    """
    class for implementing the helper interface.
    """

    def __init__(self):
         """
         Constructor function of the HelperInterface class.

         Args:
            None

         Returns:
            None
         """
         self.mutex = Lock()

         self.reset_states()

         # Define the MoveBase client
         self.movebase_client = ActionClientHelper('move_base', MoveBaseAction)

         # Define the callback associated with the battery low ROS subscribers.
         rospy.Subscriber('state/battery_low', Bool, self.BatteryLowCallback)

    def reset_states(self):
        """
        Function to reset the battery state.

        Args:
            None

        Returns:
            None
        """
        self._battery_low = True

    def BatteryLowCallback(self, msg):
        """
        Function callback called whenever a new data is published in the topic battery_low by the node battery_state.

        Args:
            msg(Bool): the battery state

        Returns:
            None
        """
        # Acquire the mutex to assure the synchronization with the other subscribers and action clients (this assure data consistency).
        self.mutex.acquire()
        try:
            # Get the battery level and set the relative state variable encoded in this class.
            self._battery_low = msg.data
        finally:
            # Release the mutex to eventually unblock the other subscribers or action servers that are waiting.
            self.mutex.release()

    def is_battery_low(self):
        """
        Function called by the finite state machine node to get the battery state that concerns the battery level.
        The returning value will be `True` if the battery is low, `False` otherwise.

        Args:
            None

        Returns:
            _battery_low(Bool): The state of the battery
        """
        return self._battery_low


    def LoadOntology(self):
        """
        Function called by the finite state machine node for loading the map of the ontology.

        Args:
            None

        Returns:
            None
        """
        global room_list, door_list_result, list_ind, client, ontology_loaded

        client.utils.mount_on_ref()
        client.utils.set_log_to_terminal(True)

        # Set the initial robot position
        client.manipulation.add_objectprop_to_ind('isIn', "Robot1", "E")

        # Subscribe to the /info/room topic to get informations about the ontology
        rospy.Subscriber("/info/rooms", InfoRoom, self.LoadOntologyCallback)

        # Loop for checking if the load ontology function has finished publishing connections
        while not rospy.is_shutdown():

                # if the informations about all the 7 locations arrived
                if ontology_loaded is True:

                    # do the disjoint
                    print("disjoint")
                    client.manipulation.disj_all_inds(list_ind)
                    print("add data prop")

                    # add the visitedAt timestamps
                    for location in room_list:
                        client.manipulation.add_dataprop_to_ind("visitedAt", location, "Long", str(int(time.time())))

                    # add the 'now' timestamp to the robot
                    print("add starting robot time")
                    client.manipulation.add_dataprop_to_ind("now", "Robot1", "Long", str(int(time.time())))

                    # add the urgency threshold to the robot
                    print("add urgency threshold")
                    client.manipulation.add_dataprop_to_ind("urgencyThreshold", "Robot1", "Long", '200')

                    # update the ontology
                    client.utils.apply_buffered_changes()
                    client.utils.sync_buffered_reasoner()

                    break

    def LoadOntologyCallback(self, msg):

        """
        Function called whenever a message of type 'InfoRoom' is published in the topic "/info/rooms"

        Args:
            msg(InfoRoom): the informations about the locations of the environment

        Returns:
            None
        """

        global room_list, door_list_result, list_ind, client, ontology_loaded, location_info

        # if the length of the name of the room is less than 2
        # this because sometimes the robot detects wrongly the markers ids resulting to a warning message response which exceeds the 2 letters length
        if len(msg.room) <= 2:
            location_info.append(msg)

            # if the length of the name of the room is greater than 0
            # this for avoiding to read empty messages
            if len(msg.room) > 0:
                room_list.append(msg.room)
                print(room_list)

            # create the list of doors, but with duplicates
            for connection in msg.connections:
                for j in range(len(room_list)):
                    door_list.append(connection.through_door)

            # create the list of doors, but without duplicates
            for i in door_list:
                if i not in door_list_result:
                    door_list_result.append(i)
            print(door_list_result)

            #merge the list of doors with the list of locations
            list_ind = room_list + door_list_result
            print(list_ind)

            # add the current location read to the LOCATION class
            client.manipulation.add_ind_to_class(msg.room, "LOCATION")

            # if the room arrived
            if msg.room:
                for connection in msg.connections:

                    # Add the connections to the ontology
                    client.manipulation.add_objectprop_to_ind("hasDoor", msg.room, connection.through_door)
                    client.manipulation.add_objectprop_to_ind("hasDoor", connection.connected_to, connection.through_door)

            else:
                # all the 7 markers informations arrived and so the ontology is completely loaded
                print("7th marker reached")
                ontology_loaded = True

    def FindNewLocObj(self, next_loc):
        """
        Function called to retrieve an object room that matches with the name of the next location chosen

        Args:
            next_loc(string): the next location which robot must visit

        Returns:
            location_info[i](obj): an object with the all informations about the rooms
        """
        global location_info
        for i in range(len(location_info)):
            if location_info[i].room == next_loc:
                return location_info[i]

    def SendGoal(self, new_loc):
        """
        Function called to send the goal to the action server

        Args:
            next_loc(string): the next location which robot must visit

        Returns:
            None
        """
        global location_info, next_location_obj

        next_location_obj = self.FindNewLocObj(new_loc)

        # Set the goal for the desired room and send it
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = next_location_obj.x
        goal.target_pose.pose.position.y = next_location_obj.y
        goal.target_pose.pose.orientation.w = 1

        self.movebase_client.reset_client_states()

        self.movebase_client.send_goal(goal)


    def TurnBaseLink(self):
        """
        Function called to do an entire loop of the arm_base_joint

        Args:
            None

        Returns:
            None
        """
        global base_link_config
        #publisher to the first joint topic
        arm_base_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size = 10)
        #subscriber to the first joint topic
        rospy.Subscriber("/robot/joint1_position_controller/command", Float64, self.TurnBaseLinkCallback)
        joint1_cmd = Float64()
        joint1_cmd = 0.0
        counter = 0

        # if the configuration of the first joint is greater than 3.14, so hopefully at 6.28 radians put the final goal to 0.0 in order to accomplish an entire 360 degrees loop
        if base_link_config > 3.14:
            joint1_cmd = 0.0
        # else, so the configuration of the first joint is less than 3.14, so hopefully at 0.0 radians put the final goal to 6.28 in order to accomplish an entire 360 degrees loop
        else:
            joint1_cmd = 6.28

        #while counter different from -1
        while counter != -1:
            #leave 35 seconds to accomplish an entire loop
            if counter < 35:
                arm_base_pub.publish(joint1_cmd)
                counter += 1
            # 35 seconds are passed and so set counter to -1 in order to exit from the while loop
            else:
                counter = -1

            rospy.sleep(1)

    def TurnBaseLinkCallback(self, msg):
        """
        Function called whenever a message of type Float64 is published in the topic "/robot/joint1_position_controller/command" in order to monitorate the current configuration of the first joint

        Args:
            msg(Float64): the current configuration of first joint

        Returns:
            None
        """
        global base_link_config
        base_link_config = msg.data

    def MoveRobot(self, new_loc):
        """
        Function called by the finite state machine node for moving the robot from the current location to the chosen one.
        In this function are updated the timestamps in order to query correctly the urgent location for the next queries

        Args:
            new_loc(string): location chosen in the 'decide' state of the finite state machine node

        Returns:
            None
        """
        #global client
        # save the prev position of the robot
        old_loc = client.query.objectprop_b2_ind("isIn", "Robot1")
        # delete the useless characters
        old_loc = old_loc[0]
        old_loc = old_loc[:len(old_loc)-1]
        old_loc = old_loc[32:]

        # query for retrieving the previous 'now'
        robot_prev_now = client.query.dataprop_b2_ind("now", "Robot1")

        # delete the useless characters
        robot_prev_now = robot_prev_now[0]
        robot_prev_now = robot_prev_now[:len(robot_prev_now)-11]
        robot_prev_now = robot_prev_now[1:]

        # query for retrieving the previous 'visitedAt'
        loc_prev_visitedAt = client.query.dataprop_b2_ind("visitedAt", new_loc)

        # delete the useless characters
        loc_prev_visitedAt = loc_prev_visitedAt[0]
        loc_prev_visitedAt = loc_prev_visitedAt[:len(loc_prev_visitedAt)-11]
        loc_prev_visitedAt = loc_prev_visitedAt[1:]

        # manipulation for changing the current location of the robot with the new one
        client.manipulation.replace_objectprop_b2_ind("isIn", "Robot1", new_loc, old_loc)

        # manipulation for changing the previous timestamp 'now' with the new one
        client.manipulation.replace_dataprop_b2_ind("now", "Robot1", 'Long', str(int(time.time())), robot_prev_now)

        # manipulation for changing the previous timestamp visitedAt with the new one
        client.manipulation.replace_dataprop_b2_ind("visitedAt", new_loc, 'Long', str(int(time.time())), loc_prev_visitedAt)

        # Update the reasoner after the manipulations
        client.utils.apply_buffered_changes()
        client.utils.sync_buffered_reasoner()

    def SetReachableLoc(self):
        """
        Function called by the Choose function in order to set the reachable location by deleting the location 'E'.

        Args:
            None

        Returns:
            reachable_loc(string): the list of reachable locations
        """
        #global client
        # query for knowing which adiacent locations can be reached
        reachable_loc = client.query.objectprop_b2_ind("canReach", "Robot1")

        # cancel the useless characters
        for i in range(len(reachable_loc)):
            reachable_loc[i] = reachable_loc[i][:len(reachable_loc[i])-1]
            reachable_loc[i] = reachable_loc[i][32:]

        # remove the location E from the list of the reachable, because we will go in E only if the battery goes low
        for i in reachable_loc.copy():
            if i == "E":
                reachable_loc.remove(i)
                break

        print("reachable locations")
        print(reachable_loc)
        return reachable_loc

    def SetUrgentLoc(self):
        """
        Function called by the Choose function in order to set the urgent locations by deleting the location 'E' and the 2 corridors.

        Args:
            None

        Returns:
            urgent_loc(string): the list of urgent locations
        """
        #global client
        # query for knowing urgent locations
        urgent_loc = client.query.ind_b2_class("URGENT")
        for i in range(len(urgent_loc)):
            urgent_loc[i] = urgent_loc[i][:len(urgent_loc[i])-1]
            urgent_loc[i] = urgent_loc[i][32:]

        for i in urgent_loc.copy():

            if i == "C1":
                urgent_loc.remove(i)
                continue
            if i == "C2":
                urgent_loc.remove(i)
                continue
            if i == "E":
                urgent_loc.remove(i)
                continue

        print("urgent locations are:")
        print(urgent_loc)

        return urgent_loc

    def SetCorridorLoc(self):
        """
        Function called by the Choose function in order to set the corridors.

        Args:
            None

        Returns:
            corridor_loc(string): the list of corridors
        """
        #global client
        corridor_loc = client.query.ind_b2_class("CORRIDOR")
        for i in range(len(corridor_loc)):
            corridor_loc[i] = corridor_loc[i][:len(corridor_loc[i])-1]
            corridor_loc[i] = corridor_loc[i][32:]

        return corridor_loc

    def SetRoomLoc(self):
        """
        Function called by the Choose function in order to set the Rooms.

        Args:
            None

        Returns:
            room_loc(string): the list of rooms
        """
        #global client
        room_loc = client.query.ind_b2_class("ROOM")
        for i in range(len(room_loc)):
            room_loc[i] = room_loc[i][:len(room_loc[i])-1]
            room_loc[i] = room_loc[i][32:]

        return room_loc

    def Choose(self):
        """
        Function called by the finite state machine to choose the next location due to the reachable locations,
        the urgent ones and knowing that the priority is: firstly we see
        the reachable urgent locations, then the corridors and if we have
        more than one possibility I choose rondomly from a list of possibilities.

        Args:
            None

        Returns:
            choice(string): the location chosen
        """
        reachable_loc = self.SetReachableLoc()
        urgent_loc = self.SetUrgentLoc()
        corridor_loc = self.SetCorridorLoc()
        room_loc = self.SetRoomLoc()
        possible_choices = []

        # check if among reachable locations there are some urgent ones.
        # In this case the possible choices are set
        for i in range(len(reachable_loc)):
            for j in range(len(urgent_loc)):
                if reachable_loc[i] == urgent_loc[j]:
                    possible_choices = possible_choices + [reachable_loc[i]]

        # in case that in the previous check the list of possible choices
        # is still empty (there are not reachable rooms among the urgent ones)
        # the list of possible choices is filled by the corridors
        if len(possible_choices) == 0:
            for i in range(len(reachable_loc)):
                for j in range(len(corridor_loc)):
                    if reachable_loc[i] == corridor_loc[j]:
                        possible_choices = possible_choices + [reachable_loc[i]]

        # in case that in the previous checks the list of possible choices
        # is still empty (there are not neither reachable rooms among
        # the urgent ones nor corridors among the reachable ones) the list of
        # possible choices is filled by the reachable rooms (not urgent)
        if len(possible_choices) == 0:
            for i in range(len(reachable_loc)):
                for j in range(len(room_loc)):
                    if reachable_loc[i] == room_loc[j]:
                        possible_choices = possible_choices + [reachable_loc[i]]

        print("possible choices are: ")
        print(possible_choices)

        # choose a random location among the list of the possible choices
        choice = random.choice(possible_choices)
        print("Decision:" + choice)
        return choice

    def VisitLocation(self, room_chosen):
        """
        Function called by the finite state machine to visit the location chosen.

        Args:
            room_chosen(string): room chosen to be visited

        Returns:
            None
        """
        self.controller_client()

    def controller_client(self):
        """
        Function called by the visit location function in order to call the controller server for simulating the random motion of the robot in a room.

        Args:
            None

        Returns:
            None
        """
        #global client
        rospy.wait_for_service('controller')
        try:
            controller = rospy.ServiceProxy('controller', Controller)
            resp1 = controller()
            return resp1.received
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def CheckRobotPosition(self):
        """
        Function called by the finite state machine node in order to know in which location the robot is.

        Args:
            None

        Returns:
            robot_position(string): the current robot position
        """
        #global client
        robot_position = client.query.objectprop_b2_ind("isIn", "Robot1")

        robot_position = robot_position[0]
        robot_position = robot_position[:len(robot_position)-1]
        robot_position = robot_position[32:]

        return robot_position
