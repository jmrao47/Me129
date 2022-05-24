#!/usr/bin/env python3

# Imports
import pigpio
import sys
import time
import math
import random
import threading
import traceback

from intersection import *
from ultrasound import *


class HardwareRobot:
    # Motor pins
    MTR1_LEGA = 7
    MTR1_LEGB = 8
    MTR2_LEGA = 5
    MTR2_LEGB = 6

    MOTOR_LEGS = [MTR1_LEGA, MTR1_LEGB, MTR2_LEGA, MTR2_LEGB]

    # IR sensors
    LEFT_IR = 14
    MIDDLE_IR = 15
    RIGHT_IR = 18

    IR_CHANNELS = [LEFT_IR, MIDDLE_IR, RIGHT_IR]

    # IR states
    COMPLETELY_OFF = (0, 0, 0)
    OFF_LEFT = (0, 0, 1)
    SLIGHT_OFF_LEFT = (0, 1, 1)
    OFF_RIGHT = (1, 0, 0)
    SLIGHT_OFF_RIGHT = (1, 1, 0)
    CENTERED = (0, 1, 0)
    CENTER_OFF = (1, 0, 1)
    COMPLETELY_ON = (1, 1, 1)

    # Driving constants
    DEFAULT_SPEED = 0.25
    DEFAULT_TURN_SPEED = 0.35
    DEFAULT_SPIRAL_ANGLE = 130
    PWM_MAX = 255
    PWM_FREQUENCY = 1000
    CAR_LENGTH = 0.135
    WAITING_TIME = 1

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
        for leg in Robot.MOTOR_LEGS:
            self.io.set_mode(leg, pigpio.OUTPUT)

        # Prepare the PWM.  The range gives the maximum value for 100%
        # duty cycle, using integer commands (1 up to max).
        for leg in Robot.MOTOR_LEGS:
            self.io.set_PWM_range(leg, Robot.PWM_MAX)

        # Set the PWM frequency to 1000Hz
        for leg in Robot.MOTOR_LEGS:
            self.io.set_PWM_frequency(leg, Robot.PWM_FREQUENCY)

        # Clear pins
        self.clear_pins()

        # Set up the three IR sensors as inputs
        for ir_pin in Robot.IR_CHANNELS:
            self.io.set_mode(ir_pin, pigpio.INPUT)

        self.ultrasound = Ultrasound(self.io)

    def is_searching(self):
        return self.searching

    def set_searching(self, val):
        self.searching = val

    def get_ir_states(self):
        return tuple([self.io.read(ir_pin) for ir_pin in Robot.IR_CHANNELS])

    def clear_pins(self):
        for leg in Robot.MOTOR_LEGS:
            self.io.set_PWM_dutycycle(leg, 0)

    # Clear the pins, disconnect the interface, and cancel the callbacks in the ultrasound
    def shutdown(self):
        self.clear_pins()
        self.io.stop()
        self.ultrasound.shutdown()

    # Sets all four pins
    def set(self, leftdutycycle, rightdutycycle):
        #print(f'Left: {leftdutycycle * Robot.PWM_MAX}, Right: {rightdutycycle * Robot.PWM_MAX}')
        # Return if values out of range
        if abs(leftdutycycle) > 1:
            print(f"Left PWM {leftdutycycle} out of range.")
            return

        if abs(rightdutycycle) > 1:
            print(f"Right PWM {rightdutycycle} out of range.")
            return

        self.clear_pins()

        # Determine left and right legs based on direction
        left_leg = Robot.MTR1_LEGA if leftdutycycle > 0 else Robot.MTR1_LEGB
        right_leg = Robot.MTR2_LEGA if rightdutycycle > 0 else Robot.MTR2_LEGB

        self.io.set_PWM_dutycycle(left_leg, abs(leftdutycycle * Robot.PWM_MAX))
        self.io.set_PWM_dutycycle(right_leg, abs(rightdutycycle * Robot.PWM_MAX))

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
        inner_radius = outer_radius - Robot.CAR_LENGTH

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
        self.setlinear(Robot.DEFAULT_SPEED)

    def drive_with_obstacles(self, time_step=0.01, distance_threshold=0.2):
        distances = robot.ultrasound.get_distances()

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
                self.setlinear(-Robot.DEFAULT_SPEED)

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
                self.setlinear(-Robot.DEFAULT_SPEED)
                

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
        self.setvel(Robot.DEFAULT_SPEED, angle)

    def turn_left(self):
        self.setvel(Robot.DEFAULT_SPEED, 90)

    def turn_slight_left(self):
        self.setvel(Robot.DEFAULT_SPEED, 60)

    def turn_right(self):
        self.setvel(Robot.DEFAULT_SPEED, -70)

    def turn_slight_right(self):
        self.setvel(Robot.DEFAULT_SPEED, -40)

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

        if state != Robot.COMPLETELY_OFF:
            while state != Robot.CENTERED:
                state = self.get_ir_states()

                # Veered left - turn right
                if state in [self.SLIGHT_OFF_LEFT, self.OFF_LEFT]:
                    self.turn_extreme_right()

                # Veered right - turn left
                elif state in [self.SLIGHT_OFF_RIGHT, self.OFF_RIGHT]:
                    self.turn_extreme_left()

                time.sleep(time_step)
                self.clear_pins()
                time.sleep(Robot.WAITING_TIME)

    def follow_line(self, time_step=0.05, extra_drive_time=0.5):
        prev_state = self.COMPLETELY_OFF
        state = None

        while True:
            time.sleep(time_step)
            state = self.get_ir_states()
            # print(state)

            # Robot is off the street - exit
            if state == self.COMPLETELY_OFF:
                break

            # Robot is at an intersection - exit
            elif state == self.COMPLETELY_ON:
                break

            # Veered left - turn right
            elif state == self.OFF_LEFT:
                self.turn_right()

            # Centered - drive straight
            elif state == self.CENTERED:
                self.drive_forward()

            # Veered slight left - turn slight right
            elif state == self.SLIGHT_OFF_LEFT:
                self.turn_slight_right()

            # Veered right - turn left
            elif state == self.OFF_RIGHT:
                self.turn_left()
            
            # Veered slight right - turn slight left
            elif state == self.SLIGHT_OFF_RIGHT:
                self.turn_slight_left()

            prev_state = state

        time.sleep(extra_drive_time)
        self.clear_pins()
        time.sleep(Robot.WAITING_TIME)
        return state

    # 0.15, 0.2
    # 0.6, 0.62
    # 0.93, 0.92
    # 1.3, 1.28
    def turn_to_next_street(self, direction, time_step=0.01, kick_off_time=0.25):
        global heading 

        # Kickoff turn 
        self.spin_in_place(direction, speed=Robot.DEFAULT_TURN_SPEED*1.1)
        time.sleep(kick_off_time)

        start = time.time()
        state = self.get_ir_states()

        # Turn until black line
        black_hits = 0
        while black_hits < 5:
            black_hits += (state != Robot.COMPLETELY_OFF)

            self.spin_in_place(direction)
            time.sleep(time_step)
            state = self.get_ir_states()

        time_elapsed = time.time() - start
        self.clear_pins()
        
        # 90 degree turn
        if time_elapsed < 0.35:
            turn = direction

        # 180 degree turn
        elif time_elapsed < 0.75:
            turn = 2 * direction

        elif time_elapsed < 1.1:
            turn = 3 * direction 
        
        else:
            turn = 4 * direction

        # Update heading
        time.sleep(Robot.WAITING_TIME)
        heading = (heading + turn) % 4
        return turn

    def turn_to_direction(self, cardinal_direction, intersection):
        global heading

        # Remain in current direction
        if cardinal_direction == heading:
            time.sleep(Robot.WAITING_TIME)
            return 

        # Turn opposite direction
        if cardinal_direction == (heading + 2) % 4:
            left = (heading + 1) % 4

            # 180 turn directly
            if intersection.streets[left] == NOSTREET:
                self.turn_to_next_street(1)

            # Two 90 degree turns
            else:
                self.turn_to_next_street(1)
                self.turn_to_next_street(1)
            
            return

        # Determine shorter turning direction
        left_turns = (cardinal_direction - heading + 4) % 4
        right_turns = (heading - cardinal_direction + 4) % 4
        direction = 1 if left_turns < right_turns else -1

        self.turn_to_next_street(direction)

    def map_streets(self, intersection, time_step=0.045, speed=0.82):
        global heading
        global longitude
        global latitude

        # Direction bot is coming from
        intersection.streets[(heading + 2) % 4] = CONNECTED

        # Right in front of bot
        state = self.get_ir_states()
        if state == Robot.COMPLETELY_OFF:
            intersection.streets[heading] = NOSTREET

        elif intersection.streets[heading] == UNKNOWN:
            intersection.streets[heading] = UNEXPLORED

        # Determine street status depending on 
        # if the next intersection has been seen or not
        def set_street_status(direction):
            next_intersection = find_intersection_coordinates(shift(longitude, latitude, direction))
            if next_intersection is None:
                intersection.streets[direction] = UNEXPLORED
            else:
                intersection.streets[direction] = CONNECTED

        left = (heading + 1) % 4
        right = (heading + 3) % 4

        # Explore left and end at 180 from original direction
        turn = self.turn_to_next_street(direction=1)

        if turn == 1:
            set_street_status(left)
            self.turn_to_next_street(direction=1) # Turn back to coming direction
            
        else:
            intersection.streets[left] = NOSTREET
            time.sleep(Robot.WAITING_TIME)
        
        # Explore right 
        turn = self.turn_to_next_street(direction=1)
        if turn == 1: 
            set_street_status(right)
            intersection.streets[right] = UNEXPLORED

        else: 
            intersection.streets[right] = NOSTREET

    def map_course(self):
        global heading
        global last_intersection
        global longitude
        global latitude

        self.follow_line()

        while True:
            # Get current intersection
            cur_intersection = find_intersection(longitude, latitude)
            if cur_intersection is None:
                cur_intersection = Intersection(longitude, latitude)    
                self.map_streets(cur_intersection)    
                time.sleep(Robot.WAITING_TIME)

            else: 
                cur_intersection.streets[(heading + 2) % 4] = CONNECTED    

            # Break if all streets explored
            if UNEXPLORED not in [s for i in intersections for s in i.streets]:
                break     

            # Turn
            try:
                # Turn to unexplored street, if it exists
                turn_direction = cur_intersection.get_random_street(UNEXPLORED)

            except IndexError:
                # Otherwise, turn to random valid street
                turn_direction = cur_intersection.get_random_street(CONNECTED)

            cur_intersection.streets[turn_direction] = CONNECTED
            self.turn_to_direction(turn_direction, cur_intersection)
            #time.sleep(Robot.WAITING_TIME)
    
            print(f'Current intersection: {cur_intersection}')

            # Drive until next intersection
            self.follow_line()
            longitude, latitude = shift(longitude, latitude, heading)
            last_intersection = cur_intersection

    def dijkstra(self, target_coordinates):
        start_intersection = find_intersection(longitude, latitude)
        target_intersection = find_intersection_coordinates(target_coordinates)

        seen = set()
        queue = [target_intersection]

        while len(queue) > 0:
            cur_intersection = queue.pop(0)

            for neighbor_intersection, direction in cur_intersection.get_neighbor_intersections().items():
                if neighbor_intersection not in seen:
                    queue.append(neighbor_intersection)
                    neighbor_intersection.headingToTarget = (direction - 2) % 4
                    seen.add(neighbor_intersection)

                if neighbor_intersection == start_intersection:
                    target_intersection.headingToTarget = STOP
                    return

    def drive_back(self):
        global longitude
        global latitude

        cur_intersection = find_intersection(longitude, latitude)
        print('start: ', cur_intersection)

        while cur_intersection.headingToTarget != STOP:
            self.turn_to_direction(cur_intersection.headingToTarget, cur_intersection)
            time.sleep(Robot.WAITING_TIME)
            self.follow_line()
            time.sleep(Robot.WAITING_TIME)

            longitude, latitude = shift(longitude, latitude, heading)
            print(longitude, latitude)
            cur_intersection = find_intersection(longitude, latitude)

    def ultrasound(self):
        pass


if __name__ == "__main__":
    # Instantiate the low-level object
    robot = Robot()
    thread = threading.Thread(target = robot.ultrasound.run_continual)
    thread.start()

    # Run the code
    try:
        #while True: 
        #    print(robot.ultrasound.get_distances())
        
        #time.sleep(1)
        #robot.drive_with_obstacles()

        time.sleep(1)
        print(robot.ultrasound.get_distances())
        robot.wall_following()

        # input()

        # robot.ultrasound.trigger()
        # time.sleep(0.2)
        # print('distances:', robot.ultrasound.get_distances())
        # input()

        # robot.map_course()
        # lon = int(input('Target longitude: '))
        # lat = int(input('Target latitude: '))

        # robot.dijkstra((lon, lat))
        # robot.drive_back()

    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()


    # Wait for the triggering thread to be done
    robot.ultrasound.stop_continual()
    thread.join()

    # Shutdown cleanly
    robot.shutdown()



