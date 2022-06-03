#!/usr/bin/env python3

# Imports
import pigpio
import sys
import time
import math
import random

from infrared import Infrared
from ultrasound import Ultrasound


class HardwareBot:
    CAR_LENGTH = 0.135
    PWM_MAX = 255
    PWM_FREQUENCY = 1000

    INTERSECTION_OBSTACLE_DISTANCE = 0.15
    STREET_OBSTACLE_DISTANCE = 0.3

    # Motor pins
    MTR1_LEGA = 7
    MTR1_LEGB = 8
    MTR2_LEGA = 5
    MTR2_LEGB = 6

    MOTOR_LEGS = [MTR1_LEGA, MTR1_LEGB, MTR2_LEGA, MTR2_LEGB]

    # Connect to the GPIO, prepare and clear the pins
    def __init__(self):
        self.searching = True

        # Prepare the GPIO connetion (to command the motors).
        print("Setting up the GPIO...")

        # Initialize the connection to the pigpio daemon (GPIO interface).
        self.io = pigpio.pi()
        if not self.io.connected:
            print("Unable to connection to pigpio daemon!")
            sys.exit(0)

        # Set up the four pins as output (commanding the motors).
        for leg in HardwareBot.MOTOR_LEGS:
            self.io.set_mode(leg, pigpio.OUTPUT)

        # Prepare the PWM.  The range gives the maximum value for 100%
        # duty cycle, using integer commands (1 up to max).
        for leg in HardwareBot.MOTOR_LEGS:
            self.io.set_PWM_range(leg, HardwareBot.PWM_MAX)

        # Set the PWM frequency to 1000Hz
        for leg in HardwareBot.MOTOR_LEGS:
            self.io.set_PWM_frequency(leg, HardwareBot.PWM_FREQUENCY)

        # Clear pins
        self.clear_pins()
        
        # Setup hardware channels
        self.infrared = Infrared(self.io)
        self.ultrasound = Ultrasound(self.io)

    def clear_pins(self):
        for leg in HardwareBot.MOTOR_LEGS:
            self.io.set_PWM_dutycycle(leg, 0)

    def get_ir_states(self):
        return self.infrared.get_ir_states()

    def get_forward_distance(self):
        return self.ultrasound.get_distances(1)

    # Clear the pins, disconnect the interface, and cancel the callbacks in the ultrasound
    def shutdown(self):
        self.clear_pins()
        self.io.stop()
        self.ultrasound.shutdown()

    



