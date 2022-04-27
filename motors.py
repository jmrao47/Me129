#!/usr/bin/env python3

# Imports
import pigpio
import sys
import time
import math


class Motors:
    DEFAULT_SPEED = 0.25
    DEFAULT_SPIRAL_ANGLE = 130

    # Define the motor pins.
    MTR1_LEGA = 7
    MTR1_LEGB = 8
    MTR2_LEGA = 5
    MTR2_LEGB = 6

    MOTOR_LEGS = [MTR1_LEGA, MTR1_LEGB, MTR2_LEGA, MTR2_LEGB]

    LEFT_IR = 14
    MIDDLE_IR = 15
    RIGHT_IR = 18

    IR_CHANNELS = [LEFT_IR, MIDDLE_IR, RIGHT_IR]

    PWM_MAX = 255
    PWM_FREQUENCY = 1000
    CAR_LENGTH = 0.135
    TURNING_FACTOR = 1

    COMPLETELY_OFF = (0, 0, 0)
    OFF_LEFT = (0, 0, 1)
    SLIGHT_OFF_LEFT = (0, 1, 1)
    OFF_RIGHT = (1, 0, 0)
    SLIGHT_OFF_RIGHT = (1, 1, 0)
    CENTERED = (0, 1, 0)
    CENTER_OFF = (1, 0, 1)
    COMPLETELY_ON = (1, 1, 1)

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

        # Set up the three IR sensors as inputs
        for ir_pin in Motors.IR_CHANNELS:
            self.io.set_mode(ir_pin, pigpio.INPUT)

    def is_searching(self):
        return self.searching

    def set_searching(self, val):
        self.searching = val

    def get_ir_states(self):
        return tuple([self.io.read(ir_pin) for ir_pin in Motors.IR_CHANNELS])

    def clear_pins(self):
        for leg in Motors.MOTOR_LEGS:
            self.io.set_PWM_dutycycle(leg, 0)

    # Clear the pins and disconnect the interface.
    def shutdown(self):
        self.clear_pins()
        self.io.stop()

    # Sets all four pins
    def set(self, leftdutycycle, rightdutycycle):
        #print(f'Left: {leftdutycycle * Motors.PWM_MAX}, Right: {rightdutycycle * Motors.PWM_MAX}')
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
        # leftdutycycle = (abs(speed) + .20194) / .72809 * dir
        leftdutycycle = (abs(speed) + .23644) / .77 * dir

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

    def drive_forward(self):
        self.setlinear(Motors.DEFAULT_SPEED)

    def spiral_outward(self, angle):
        self.setvel(Motors.DEFAULT_SPEED, angle)

    def turn_left(self):
        self.setvel(Motors.DEFAULT_SPEED, 50)

    def turn_slight_left(self):
        self.setvel(Motors.DEFAULT_SPEED, 20)

    def turn_right(self):
        self.setvel(Motors.DEFAULT_SPEED, -50)

    def turn_slight_right(self):
        self.setvel(Motors.DEFAULT_SPEED, -20)


if __name__ == "__main__":
    # Instantiate the low-level object
    motors = Motors()
    time_step = 0.05
    spiral_angle = Motors.DEFAULT_SPIRAL_ANGLE

    # Run the code
    try:
        prev_state = Motors.COMPLETELY_OFF

        while True:
            time.sleep(time_step)
            state = motors.get_ir_states()

            if state != Motors.COMPLETELY_OFF:
                motors.set_searching(False)
            
            # Spiral to search, changing angle to spiral outward
            if motors.is_searching():
                spiral_dir = spiral_angle / abs(spiral_angle)
                spiral_angle -= 0.1 * spiral_dir
                motors.spiral_outward(spiral_angle)
                continue

            # Robot is lost, so start spiraling
            if state == Motors.COMPLETELY_OFF:
                # Veered completely off left - spiral right
                if prev_state in [Motors.SLIGHT_OFF_LEFT, Motors.OFF_LEFT]:
                    print('spiral left')
                    spiral_angle = -Motors.DEFAULT_SPIRAL_ANGLE

                # Veered completely off right - spiral left
                elif prev_state in [Motors.SLIGHT_OFF_RIGHT, Motors.OFF_RIGHT]:
                    print('spiral right')
                    spiral_angle = Motors.DEFAULT_SPIRAL_ANGLE

                # Default - spiral left
                else:
                    print('default')
                    spiral_angle = Motors.DEFAULT_SPIRAL_ANGLE

                motors.set_searching(True)
                continue


            # Veered left - turn right
            elif state == Motors.OFF_LEFT:
                motors.turn_right()

            # Centered - drive straight
            elif state == Motors.CENTERED:
                motors.drive_forward()

            # Veered slight left - turn slight right
            elif state == Motors.SLIGHT_OFF_LEFT:
                motors.turn_slight_right()

            # Veered right - turn left
            elif state == Motors.OFF_RIGHT:
                motors.turn_left()
            
            # Veered slight right - turn slight left
            elif state == Motors.SLIGHT_OFF_RIGHT:
                motors.turn_slight_left()
            
            # Centered - drive straight
            elif state == (1, 1, 1):
                motors.drive_forward()

            prev_state = state
            

    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))

    # Shutdown cleanly
    motors.shutdown()


