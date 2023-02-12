#!/usr/bin/env python
"""
.. module:: controller
   :platform: Unix
   :synopsis: Python module for simulating the random movement of the robot in a location

.. moduleauthor:: Daniele Martino Parisi

Servers:
    /controller

The node simulate the random robot motion in the location reached by only waste time
"""
import rospy
import random
import time

from assignment_1.srv import Controller


def MoveRandomCallBack(req):
    """
    Function callback called whenever the client sends a request to visit a location.

    Args:
            req(int): request of the service

    Returns:
            feedback(int): response feedback
    """
    print("visiting")
    time.sleep(1)
    feedback = 1
    return feedback

def my_controller_server():
    """
    Function used for initializing the server 'my_controller_server'

    Args:
            none

    Returns:
            none
    """
    rospy.init_node('my_controller_server')
    s = rospy.Service('controller', Controller, MoveRandomCallBack)
    print("Service ready.")
    rospy.spin()

if __name__ == "__main__":
     my_controller_server()
