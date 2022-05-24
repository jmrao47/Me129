import time 

from driving_bot import DrivingBot
from infrared import Infrared
from intersection import *

class MappingBot(DrivingBot):
    def __init__(self):
        super(MappingBot, self).__init__()

    # 0.15, 0.2
    # 0.6, 0.62
    # 0.93, 0.92
    # 1.3, 1.28
    def turn_to_next_street(self, direction, time_step=0.01, kick_off_time=0.25):
        global heading 

        # Kickoff turn 
        self.spin_in_place(direction, speed=DrivingBot.DEFAULT_TURN_SPEED*1.1)
        time.sleep(kick_off_time)

        start = time.time()
        state = self.get_ir_states()

        # Turn until black line
        black_hits = 0
        while black_hits < 5:
            black_hits += (state != Infrared.COMPLETELY_OFF)

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
        time.sleep(DrivingBot.WAITING_TIME)
        heading = (heading + turn) % 4
        return turn

    def turn_to_direction(self, cardinal_direction, intersection):
        global heading

        # Remain in current direction
        if cardinal_direction == heading:
            time.sleep(DrivingBot.WAITING_TIME)
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
        if state == Infrared.COMPLETELY_OFF:
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
            time.sleep(DrivingBot.WAITING_TIME)
        
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
                time.sleep(DrivingBot.WAITING_TIME)

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
            #time.sleep(DrivingBot.WAITING_TIME)
    
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
            time.sleep(DrivingBot.WAITING_TIME)
            self.follow_line()
            time.sleep(DrivingBot.WAITING_TIME)

            longitude, latitude = shift(longitude, latitude, heading)
            print(longitude, latitude)
            cur_intersection = find_intersection(longitude, latitude)


