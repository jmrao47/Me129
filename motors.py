#!/usr/bin/env python3

# Imports
import pigpio
import sys
import time


class Motors:
    # Define the motor pins.
    MTR1_LEGA = 7
    MTR1_LEGB = 8
    MTR2_LEGA = 5
    MTR2_LEGB = 6

    MOTOR_LEGS = [MTR1_LEGA, MTR1_LEGB, MTR2_LEGA, MTR2_LEGB]

    PWM_MAX = 255
    PWM_FREQUENCY = 1000

    # Connect to the GPIO, prepare and clear the pins
    def __init__(self):
        # Prepare the GPIO connetion (to command the motors).
        print("Setting up the GPIO...")

        # Initialize the connection to the pigpio daemon (GPIO interface).
        self.io = pigpio.pi()
        if not self.io.connected:
            print("Unable to connection to pigpio daemon!")
            sys.exit(0)

        # Set up the four pins as output (commanding the motors).
        for leg in Motors.MOTOR_LEGS:
            self.io.set_mode(leg, pigpio.OUTPUT)

        # Prepare the PWM.  The range gives the maximum value for 100%
        # duty cycle, using integer commands (1 up to max).
        for leg in Motors.MOTOR_LEGS:
            self.io.set_PWM_range(leg, Motors.PWM_MAX)

        # Set the PWM frequency to 1000Hz
        for leg in Motors.MOTOR_LEGS:
            self.io.set_PWM_frequency(leg, Motors.PWM_FREQUENCY)

        # Clear pins
        self.clear_pins()

    def clear_pins(self):
        for leg in Motors.MOTOR_LEGS:
            self.io.set_PWM_dutycycle(leg, 0)

    # Clear the pins and disconnect the interface.
    def shutdown(self):
        self.clear_pins()
        self.io.stop()

    # Sets all four pins
    def set(self, leftdutycycle, rightdutycycle):
        # Return if values out of range
        if abs(leftdutycycle) > 1:
            print(f'Left PWM {leftdutycycle} out of range.')
            return

        if abs(rightdutycycle) > 1:
            print(f'Right PWM {leftdutycycle} out of range.')
            return

        self.clear_pins()

        # Determine left and right legs based on direction
        left_leg = Motors.MTR1_LEGA if leftdutycycle > 0 else Motors.MTR1_LEGB
        right_leg = Motors.MTR2_LEGA if leftdutycycle > 0 else Motors.MTR2_LEGB

        self.io.set_PWM_dutycycle(left_leg, abs(leftdutycycle * Motors.PWM_MAX))
        self.io.set_PWM_dutycycle(right_leg, abs(rightdutycycle * Motors.PWM_MAX))

    def setlinear(self, speed):
        pass

    def setspin(self, speed):
        pass


if __name__ == "__main__":
    # Instantiate the low-level object
    motors = Motors()

    # Run the code
    try:
        motors.set(0.8, 0.8)
        time.sleep(10)

    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))

    # Shutdown cleanly
    motors.shutdown()


