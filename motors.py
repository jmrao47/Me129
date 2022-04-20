#!/usr/bin/env python3

# Imports
import pigpio
import sys
import time
import math


class Motors:
    # Define the motor pins.
    MTR1_LEGA = 7
    MTR1_LEGB = 8
    MTR2_LEGA = 5
    MTR2_LEGB = 6

    MOTOR_LEGS = [MTR1_LEGA, MTR1_LEGB, MTR2_LEGA, MTR2_LEGB]

    PWM_MAX = 255
    PWM_FREQUENCY = 1000
    CAR_LENGTH = 0.135
    TURNING_FACTOR = 1

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
            print(f"Left PWM {leftdutycycle} out of range.")
            return

        if abs(rightdutycycle) > 1:
            print(f"Right PWM {rightdutycycle} out of range.")
            return

        self.clear_pins()

        # Determine left and right legs based on direction
        left_leg = Motors.MTR1_LEGA if leftdutycycle > 0 else Motors.MTR1_LEGB
        right_leg = Motors.MTR2_LEGA if rightdutycycle > 0 else Motors.MTR2_LEGB

        self.io.set_PWM_dutycycle(left_leg, abs(leftdutycycle * Motors.PWM_MAX))
        self.io.set_PWM_dutycycle(right_leg, abs(rightdutycycle * Motors.PWM_MAX))

    def getleftlineardutycycle(self, speed):
        dir = speed / abs(speed)
        leftdutycycle = (abs(speed) + .20194) / .72809 * dir

        return leftdutycycle

    def getrightlineardutycycle(self, speed):
        dir = speed / abs(speed)
        rightdutycycle = (abs(speed) + .23644) / .64746 * dir

        return rightdutycycle

    def getlinear(self, speed):
        return self.getleftlineardutycycle(speed), self.getrightlineardutycycle(speed)

    def setlinear(self, speed):
        leftdutycycle = self.getleftlineardutycycle(speed)
        rightdutycycle = self.getrightlineardutycycle(speed)

        self.set(leftdutycycle, rightdutycycle)

    # Speed in degrees per second
    # Positive is CCW (turning on right wheel)
    # Negative is CW (turning on left wheel)
    def getleftspindutycycle(self, spin):
        leftdutycycle = (abs(spin) + 206.43) / 442.86
        return leftdutycycle

    def getrightspindutycycle(self, spin):
        rightdutycycle = (abs(spin) + 184.17) / 350
        return rightdutycycle

    def getspin(self, spin):
        if spin > 0:
            return 0, self.getrightspindutycycle(spin)
        else:
            return self.getleftspindutycycle(spin), 0

    def setspin(self, speed):
        leftdutycycle, rightdutycycle = self.getspin(speed)
        self.set(leftdutycycle, rightdutycycle)

    def setvel_old(self, linear, spin):
        left_linear, right_linear = self.getlinear(linear)
        left_spin, right_spin = self.getspin(spin)

        right_speed = right_spin + right_linear
        left_speed = left_spin + left_linear

        if spin > 0:
            if right_speed > 1:
                left_speed = 1 - (right_speed - left_speed)
                right_speed = 1
        else:
            if left_speed > 1:
                right_speed = 1 - (left_speed - right_speed)
                left_speed = 1

        self.set(left_speed, right_speed)

    def setvel(self, linear, spin):
        T = 360 / abs(spin)
        outer_radius = linear * T / (2 * math.pi)
        inner_radius = outer_radius - Motors.CAR_LENGTH

        inner_speed = 2 * math.pi * inner_radius / T
        outer_speed = linear

        # Spinning CCW - left wheel inside
        if spin > 0:
            leftdutycycle = self.getleftlineardutycycle(inner_speed)
            rightdutycycle = self.getrightlineardutycycle(outer_speed)

        # Spinning CW - left wheel outside
        else:
            leftdutycycle = self.getleftlineardutycycle(outer_speed)
            rightdutycycle = self.getrightlineardutycycle(inner_speed)

        self.set(leftdutycycle, rightdutycycle)

    def setcircle(self, d, T):
        linear = d * math.pi / T
        spin = 360 / T
        motors.setvel(linear, spin)

if __name__ == "__main__":
    # Instantiate the low-level object
    motors = Motors()

    # Run the code
    try:
        d = float(input("d: "))
        T = float(input("T: "))

        motors.setcircle(d, T)
        time.sleep(100)

    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))

    # Shutdown cleanly
    motors.shutdown()


