#!/usr/bin/env python3
#
#   motordemo.py
#
#   This shows how to interface with the GPIO (general purpose I/O)
#   pins and how to drive the PWM for the motors.  Please use as an
#   example, but change to suit the weekly goals.
#

# Imports
import pigpio
import sys
import time

# Define the motor pins.
MTR1_LEGA = 7
MTR1_LEGB = 8

MTR2_LEGA = 5
MTR2_LEGB = 6


#
#   Main
#
if __name__ == "__main__":

    ############################################################
    # Prepare the GPIO connetion (to command the motors).
    print("Setting up the GPIO...")
    
    # Initialize the connection to the pigpio daemon (GPIO interface).
    io = pigpio.pi()
    if not io.connected:
        print("Unable to connection to pigpio daemon!")
        sys.exit(0)

    # Set up the four pins as output (commanding the motors).
    io.set_mode(MTR1_LEGA, pigpio.OUTPUT)
    io.set_mode(MTR1_LEGB, pigpio.OUTPUT)
    io.set_mode(MTR2_LEGA, pigpio.OUTPUT)
    io.set_mode(MTR2_LEGB, pigpio.OUTPUT)

    # Prepare the PWM.  The range gives the maximum value for 100%
    # duty cycle, using integer commands (1 up to max).
    io.set_PWM_range(MTR1_LEGA, 255)
    io.set_PWM_range(MTR1_LEGB, 255)
    io.set_PWM_range(MTR2_LEGA, 255)
    io.set_PWM_range(MTR2_LEGB, 255)
    
    # Set the PWM frequency to 1000Hz.  You could try 500Hz or 2000Hz
    # to see whether there is a difference?
    io.set_PWM_frequency(MTR1_LEGA, 1000)
    io.set_PWM_frequency(MTR1_LEGB, 1000)
    io.set_PWM_frequency(MTR2_LEGA, 1000)
    io.set_PWM_frequency(MTR2_LEGB, 1000)

    # Clear all pins, just in case.
    io.set_PWM_dutycycle(MTR1_LEGA, 0)
    io.set_PWM_dutycycle(MTR1_LEGB, 0)
    io.set_PWM_dutycycle(MTR2_LEGA, 0)
    io.set_PWM_dutycycle(MTR2_LEGB, 0)

    print("GPIO ready...")

    ############################################################
    # Forward/Backward
    print("Driving Motor 1 one way, stopping, then reversing...")
    io.set_PWM_dutycycle(MTR1_LEGA, 170)
    io.set_PWM_dutycycle(MTR1_LEGB,   0)
    time.sleep(1)

    io.set_PWM_dutycycle(MTR1_LEGA,   0)
    io.set_PWM_dutycycle(MTR1_LEGB,   0)
    time.sleep(1)
    
    io.set_PWM_dutycycle(MTR1_LEGA,   0)
    io.set_PWM_dutycycle(MTR1_LEGB, 170)
    time.sleep(1)

    io.set_PWM_dutycycle(MTR1_LEGA,   0)
    io.set_PWM_dutycycle(MTR1_LEGB,   0)
    time.sleep(1)


    ############################################################
    # Ramping up/down.  Up to 254 (not 255).
    print("Ramping Motor 1 up, down - four times...")

    for i in range(4):
        for pwmlevel in [50, 100, 150, 200, 254, 200, 150, 100, 50, 0]:
            print("Motor 1 Leg A = %3d" % pwmlevel)
            io.set_PWM_dutycycle(MTR1_LEGA, pwmlevel)
            time.sleep(1)

    ############################################################
    # Turn Off.
    # Note the PWM still stay at the last commanded value.  So you
    # want to be sure to set to zero before the program closes.  else
    # your robot will run away...
    print("Turning off...")

    # Clear the PINs (commands).
    io.set_PWM_dutycycle(MTR1_LEGA, 0)
    io.set_PWM_dutycycle(MTR1_LEGB, 0)
    io.set_PWM_dutycycle(MTR2_LEGA, 0)
    io.set_PWM_dutycycle(MTR2_LEGB, 0)
    
    # Also stop the interface.
    io.stop()
