#!/usr/bin/env python
"""
.. module:: battery_state
   :platform: Unix
   :synopsis: Python module for simulating the behaviour of the battery state and publish the state on a topic

.. moduleauthor:: Daniele Martino Parisi

Publisher:
    /publisher: publish on /state/battery_low topic

The node simulate the behaviour of the battery state and publish the state on topic /state/battery_low

"""

import threading
import random
import rospy

from std_msgs.msg import Bool

LOG_TAG = 'robot-state'
recharging = False

class BatteryState:
    """
    class for implementing the helper interface.

    Methods
    -------------

    __init__(self)

        Constructor function of the BatteryState class.

        Parameters:
            None

        Returns:
            None

    _is_battery_low(self)

        Function that publishes changes of battery levels. This method runs on a separate thread.

        Parameters:
            None

        Returns:
            None

    _random_battery_notifier(self, publisher)

        Function to publish when the battery change state (i.e., high/low) based on a random
        delay within the interval [`self._random_battery_time[0]`, `self._random_battery_time[1]`).
        The message is published through the `publisher` input parameter and is a
        boolean value, i.e., `True`: battery low, `False`: battery high.

        Parameters:
            publisher: publisher variable for publishing on the topic /state/battery_low

        Returns:
            None

    _print_info(self, msg)

        Function for printing the state of the battery in the terminal.

        Parameters:
            msg(string): the message to be printed:

        Returns:
            None

    """

    def __init__(self):

        # Initialise this node.
        rospy.init_node('robot-state', log_level=rospy.INFO)
        # Initialise battery level.
        self._battery_low = False
        # Initialise randomness, if enabled.
        self._randomness = rospy.get_param('test/random_sense/active', True)
        if self._randomness:
            self._random_battery_low_time = rospy.get_param('test/random_sense/battery_low_time', [180.0, 240.0])
            self._random_battery_high_time = rospy.get_param('test/random_sense/battery_high_time', [40.0, 60.0])
            log_msg1 = (f'Random-based battery low notification active: the battery change state (i.e., low/high) with a '
                       f'delay in the range of [{self._random_battery_low_time[0]}, {self._random_battery_low_time[1]}) seconds.')

            log_msg2 = (f'Random-based battery low notification active: the battery change state (i.e., low/high) with a '
                       f'delay in the range of [{self._random_battery_high_time[0]}, {self._random_battery_high_time[1]}) seconds.')
            print(log_msg1, "\n")
            print(log_msg2, "\n")

        # Start publisher on a separate thread.
        th = threading.Thread(target=self._is_battery_low)
        th.start()

    def _is_battery_low(self):
        """
        Function that publishes changes of battery levels. This method runs on a separate thread.

        Args:
            None

        Returns:
            None

        """
        # Define a `lathed` publisher to wait for initialisation and publish immediately.
        publisher = rospy.Publisher('state/battery_low', Bool, queue_size=1, latch=True)

        self._random_battery_notifier(publisher)

    def _random_battery_notifier(self, publisher):
        """
        Function to publish when the battery change state (i.e., high/low) based on a random
        delay within the interval [`self._random_battery_time[0]`, `self._random_battery_time[1]`).
        The message is published through the `publisher` input parameter and is a
        boolean value, i.e., `True`: battery low, `False`: battery high.

        Args:
            publisher: publisher variable for publishing on the topic /state/battery_low

        Returns:
            None
        """
        delay = 0  # Initialised to 0 just for logging purposes.
        while not rospy.is_shutdown():
            # Publish battery level.
            publisher.publish(Bool(self._battery_low))
            # Log state.
            if self._battery_low:
                recharging = True
                log_msg = f'Robot got low battery after {delay} seconds.'
            else:
                log_msg = f'Robot got a fully charged battery after {delay} seconds.'
                recharging = False
            self._print_info(log_msg)
            # Wait for simulate battery usage.
            delay_recharging = random.uniform(self._random_battery_low_time[0], self._random_battery_low_time[1])
            delay_unrecharging = random.uniform(self._random_battery_high_time[0], self._random_battery_high_time[1])

            if recharging == True:
                rospy.sleep(delay_unrecharging)

            else:
                rospy.sleep(delay_recharging)


            # Change battery state.
            self._battery_low = not self._battery_low

    def _print_info(self, msg):
        """
        Function for printing the state of the battery in the terminal.
        Args:
            msg(string): the message to be printed:

        Returns:
            None
        """
        if self._randomness:
            print(msg)


if __name__ == "__main__":
    # Instantiate the node manager class and wait.
    BatteryState()
    rospy.spin()
