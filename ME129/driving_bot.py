import time 
import math

from hardware_bot import HardwareBot
from infrared import Infrared

class DrivingBot(HardwareBot):
    # Driving constants
    DEFAULT_SPEED = 0.25
    DEFAULT_TURN_SPEED = 0.35
    DEFAULT_SPIRAL_ANGLE = 130
    WAITING_TIME = 1

    def __init__(self):
        super(DrivingBot, self).__init__()

    def is_searching(self):
        return self.searching

    def set_searching(self, val):
        self.searching = val

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
        left_leg = HardwareBot.MTR1_LEGA if leftdutycycle > 0 else HardwareBot.MTR1_LEGB
        right_leg = HardwareBot.MTR2_LEGA if rightdutycycle > 0 else HardwareBot.MTR2_LEGB

        self.io.set_PWM_dutycycle(left_leg, abs(leftdutycycle * DrivingBot.PWM_MAX))
        self.io.set_PWM_dutycycle(right_leg, abs(rightdutycycle * DrivingBot.PWM_MAX))

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
        inner_radius = outer_radius - HardwareBot.CAR_LENGTH

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
        self.setvel(linear, spin)

    def drive_forward(self):
        self.setlinear(DrivingBot.DEFAULT_SPEED)

    def drive_with_obstacles(self, time_step=0.01, distance_threshold=0.2):
        distances = self.ultrasound.get_distances()

        while True:
            left_obstacle, middle_obstacle, right_obstacle = [dist <= distance_threshold for dist in distances]

            # No obstacles => drive forward
            if not left_obstacle and not middle_obstacle and not right_obstacle:
                self.drive_forward()

            # Only right obstacle => turn left
            elif not left_obstacle and not middle_obstacle and right_obstacle:
                self.turn_left()

            # Only middle obstacle => turn extreme left 
            elif not left_obstacle and middle_obstacle and not right_obstacle:
                # self.turn_extreme_left()
                self.setlinear(-DrivingBot.DEFAULT_SPEED)

            # Middle and right obstacles => turn extreme left
            elif not left_obstacle and middle_obstacle and right_obstacle:
                self.turn_extreme_left()

            # Only left ostacle => turn right
            elif left_obstacle and not middle_obstacle and not right_obstacle:
                self.turn_right()

            # Left and right obstacles => go straight
            elif left_obstacle and not middle_obstacle and right_obstacle:
                self.drive_forward()

            # Left and middle obstacles => extreme right turn 
            elif left_obstacle and middle_obstacle and not right_obstacle:
                self.turn_extreme_right()

            # All obstacles => stop
            elif left_obstacle and middle_obstacle and right_obstacle:
                self.setlinear(-DrivingBot.DEFAULT_SPEED)
                

            time.sleep(time_step)
            distances = self.ultrasound.get_distances()
            print('DISTANCE:', distances)

        self.shutdown()

    def wall_following(self, k=0.5, desired_dist=0.3, wall_dir=0, time_step=0.05):
        count = 0

        while True:
            dist_to_wall = self.ultrasound.get_distances(wall_dir)
            error = dist_to_wall - desired_dist
            u = -k * error

            print(f'distance to wall: {dist_to_wall}, error: {error}, u: {u}')

            speed_left = max(0.22, min(0.28, 0.25 + u))
            speed_right = max(0.22, min(0.28, 0.25 - u)) 

            print(f'left speed: {speed_left}, right speed: {speed_right}')
            print()

            self.set(self.getleftlineardutycycle(speed_left), self.getrightlineardutycycle(speed_right))
            time.sleep(time_step)

            count += 1

            if count % 100 == 0:
                continue
                #k = float(input("k: "))

    def spiral_outward(self, angle):
        self.setvel(DrivingBot.DEFAULT_SPEED, angle)

    def turn_left(self):
        self.setvel(DrivingBot.DEFAULT_SPEED, 90)

    def turn_slight_left(self):
        self.setvel(DrivingBot.DEFAULT_SPEED, 60)

    def turn_right(self):
        self.setvel(DrivingBot.DEFAULT_SPEED, -70)

    def turn_slight_right(self):
        self.setvel(DrivingBot.DEFAULT_SPEED, -40)

    # dir = 1 is a CCW turn
    # dir = -1 is a CW turn
    def spin_in_place(self, dir=1, speed=DEFAULT_TURN_SPEED):
        leftdutycycle = -self.getleftlineardutycycle(speed)
        rightdutycycle = self.getrightlineardutycycle(speed) 

        self.set(dir * leftdutycycle, dir * rightdutycycle)

    def turn_extreme_left(self):
        self.spin_in_place(dir=1)

    def turn_extreme_right(self):
        self.spin_in_place(dir=-1)

    def turn(self, magnitude):
        print('TURNING')
        global heading

        heading = (heading + magnitude) % 4
        magnitude = (magnitude + 4) % 4

        if magnitude == 1:
            self.turn_extreme_left()
            time.sleep(0.52)

        elif magnitude == 2:
            self.turn_extreme_left()
            time.sleep(0.97)

        elif magnitude == 3:
            self.turn_extreme_right()
            time.sleep(0.55)

        elif magnitude == 0:
            return

        self.clear_pins()

    # Adjust turn to center on black line
    def adjust_to_line(self, time_step=0.05):
        state = self.get_ir_states()

        if state != Infrared.COMPLETELY_OFF:
            while state != Infrared.CENTERED:
                state = self.get_ir_states()

                # Veered left - turn right
                if state in [Infrared.SLIGHT_OFF_LEFT, Infrared.OFF_LEFT]:
                    self.turn_extreme_right()

                # Veered right - turn left
                elif state in [Infrared.SLIGHT_OFF_RIGHT, Infrared.OFF_RIGHT]:
                    self.turn_extreme_left()

                time.sleep(time_step)
                self.clear_pins()
                time.sleep(DrivingBot.WAITING_TIME)

    def follow_line(self, time_step=0.05, extra_drive_time=0.5):
        prev_state = Infrared.COMPLETELY_OFF
        state = None

        while True:
            time.sleep(time_step)
            state = self.get_ir_states()
            # print(state)

            # Robot is off the street - exit
            if state == Infrared.COMPLETELY_OFF:
                break

            # Robot is at an intersection - exit
            elif state == Infrared.COMPLETELY_ON:
                break

            # Veered left - turn right
            elif state == Infrared.OFF_LEFT:
                self.turn_right()

            # Centered - drive straight
            elif state == Infrared.CENTERED:
                self.drive_forward()

            # Veered slight left - turn slight right
            elif state == Infrared.SLIGHT_OFF_LEFT:
                self.turn_slight_right()

            # Veered right - turn left
            elif state == Infrared.OFF_RIGHT:
                self.turn_left()
            
            # Veered slight right - turn slight left
            elif state == Infrared.SLIGHT_OFF_RIGHT:
                self.turn_slight_left()

            prev_state = state

        time.sleep(extra_drive_time)
        self.clear_pins()
        time.sleep(DrivingBot.WAITING_TIME)
        return state
