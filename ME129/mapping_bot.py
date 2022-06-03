import time 

import cfg  

from hardware_bot import HardwareBot
from driving_bot import DrivingBot
from infrared import Infrared
from intersection import *

class MappingBot(DrivingBot):

    def __init__(self):
        super(MappingBot, self).__init__()
        self.current_task = None
        self.current_args = None

        self.stop_task = True 
        self.pause_all_tasks = True 

        self.task_name_to_command = {
            'map_course': self.map_course, 
            'go_to': self.go_to_intersection, 
            'orient_bot': orient_bot, 
            'save_map': save_map,
            'load_map': load_map,
            'print_map': print_map,
            'clear_map': clear_map,
            'stop': self.stop_current_task,
        }

    # 0.17, 0.18
    # 0.49, 0.58
    # 0.8, 0.91
    # 1.15, 1.26
    def turn_to_next_street(self, direction=1, time_step=0.01, kick_off_time=0.25):
        # Kickoff turn 
        self.spin_in_place(direction, speed=DrivingBot.DEFAULT_TURN_SPEED*1.1)
        time.sleep(kick_off_time)

        start = time.time()
        state = self.get_ir_states()

        # Turn until black line
        black_hits = 0
        while black_hits < 50:
            black_hits += (state != Infrared.COMPLETELY_OFF)
            state = self.get_ir_states()

        time_elapsed = time.time() - start
        self.clear_pins()

        # 0.17, 0.18
        # 0.49, 0.58
        # 0.8, 0.91
        # 1.15, 1.26
        
        # 90 degree turn
        if time_elapsed < 0.25:
            turn = direction

        # 180 degree turn
        elif time_elapsed < 0.70:
            turn = 2 * direction

        elif time_elapsed < 1:
            turn = 3 * direction 
        
        else:
            turn = 4 * direction

        # Update cfg.heading
        time.sleep(DrivingBot.WAITING_TIME)
        cfg.heading = (cfg.heading + turn) % 4
        return turn

    def turn_to_direction(self, cardinal_direction, intersection):
        cardinal_direction %= 4

        # Remain in current direction
        if cardinal_direction == cfg.heading:
            #time.sleep(DrivingBot.WAITING_TIME)
            return 

        # Turn opposite direction
        if cardinal_direction == (cfg.heading + 2) % 4:
            left = (cfg.heading + 1) % 4
            right = (cfg.heading + 3) % 4

            # 180 turn directly
            if intersection.streets[left] == NOSTREET:
                self.turn_to_next_street(1)

            elif intersection.streets[right] == NOSTREET:
                self.turn_to_next_street(-1)

            # Two 90 degree turns
            else:
                self.turn_to_next_street(1)
                self.turn_to_next_street(1)
            
            return

        # Determine shorter turning direction
        direction = turn_direction(cardinal_direction)
        self.turn_to_next_street(direction)

    def map_streets(self, intersection, time_step=0.045, speed=0.82):
        # Determine street status depending on ir state, obstacles, and
        # if the opposing intersection has been seen or not
        def set_street_status():
            state = self.get_ir_states()
            opposing_intersection = get_opposing_intersection(intersection)

            # Off the tape
            if state == Infrared.COMPLETELY_OFF:
                status = NOSTREET

            # Current street blocked 
            elif self.street_blocked():
                status = BLOCKED
            
            # On the tape, haven't seen next intersection
            elif opposing_intersection is None: 
                status = UNEXPLORED
            
            # On the tape, have seen next intersection
            else: 
                status = CONNECTED

            # Set current street and opposite street statuses
            intersection.streets[cfg.heading] = status
            set_opposing_intersection(intersection, status)

        # Turn at least 270 degrees total
        total_turn = 0
        while True:
            # Set current direction street status
            set_street_status()

            # Exit if completed 270 degree turn
            if total_turn >= 3:
                break

            # Turn to next street
            turn = self.turn_to_next_street()
            total_turn += turn 

            # Set all skipped locations to NOSTREET
            if turn > 1:
                for direction in range(cfg.heading - turn + 1, cfg.heading):
                    intersection.streets[direction % 4] = NOSTREET

    def get_or_create_intersection(self, longitude, latitude):
        cur_intersection = find_intersection(longitude, latitude)

        if cur_intersection is None:
            cur_intersection = Intersection(longitude, latitude)   
        else: 
            cur_intersection.streets[(cfg.heading + 2) % 4] = CONNECTED 

        return cur_intersection

    def map_course(self, *target_coordinates):
        target_coordinates = tuple([int(coor) for coor in target_coordinates])
        time.sleep(0.5)

        while True:
            # Get current intersection
            cur_intersection = self.get_or_create_intersection(cfg.longitude, cfg.latitude)
            cur_intersection.blocked = False    
            if UNKNOWN in cur_intersection.streets or BLOCKED in cur_intersection.streets:
                self.map_streets(cur_intersection)
 
            print(f'cur intersection: {cur_intersection}')
            unexplored_intersections = [i for i in cfg.intersections for s in i.streets if s == UNEXPLORED]

            # Reached target 
            if cur_intersection.get_coordinates() == target_coordinates:
                print('Found target!')
                break 

            # End task 
            if self.stop_task:
                break

            # Break if all intersections fully explored
            if len(unexplored_intersections) == 0 and target_coordinates == ():
                print(f'final intersection: {cur_intersection}')
                break     

            # If unexplored streets exist
            if len(cur_intersection.get_streets(UNEXPLORED)) != 0:
                # Choose closest unxplored street
                if target_coordinates == ():
                    new_cardinal_direction = cur_intersection.get_closest_street(UNEXPLORED)

                # Choose unexplored streets which points bot closest to target
                else: 
                    new_cardinal_direction = \
                    cur_intersection.get_closest_neighbor_direction(target_coordinates, UNEXPLORED)

            # If no unexplored streets exist
            else: 
                # Route to unexplored intersection 
                if target_coordinates == ():
                    closest_unexplored = unexplored_intersections[0].get_coordinates()

                # Route to unexplored intersection closest to target 
                else: 
                    closest_unexplored = None 
                    min_dist = float('inf')

                    for intersection in unexplored_intersections:
                        dist = distance_between(target_coordinates, intersection.get_coordinates())

                        if dist < min_dist:
                            closest_unexplored = intersection 
                            min_dist = dist 

                if closest_unexplored is None: 
                    new_cardinal_direction = \
                    cur_intersection.get_closest_neighbor_direction(target_coordinates, CONNECTED)
                else: 
                    self.dijkstra(closest_unexplored.get_coordinates())
                    new_cardinal_direction = cur_intersection.headingToTarget

            # Turn to new direction 
            cur_intersection.streets[new_cardinal_direction] = CONNECTED
            self.turn_to_direction(new_cardinal_direction, cur_intersection)

            # Drive until next intersection
            if self.follow_line_avoiding_obstacles():
                cfg.longitude, cfg.latitude = shift(cfg.longitude, cfg.latitude, cfg.heading)

    def directed_exploration(self, target_coordinates):
        # Drive until next intersection 
        self.follow_line()
        self.get_or_create_intersection(cfg.longitude, cfg.latitude)

        if len(cfg.intersections) != 0:
            # Find closest known intersection 
            min_dist = float("inf")
            closest_intersection = None

            for intersection in cfg.intersections: 
                dist = distance_between(intersection, target_coordinates)
                if dist < min_dist:
                    min_dist = dist 
                    closest_intersection = intersection
            
            # Drive to closest intersection 
            self.dijkstra(closest_intersection)
            self.drive_back(closest_intersection)

        # Drive toward target 
        while (cfg.longitude, cfg.latitude) != target_coordinates:
            cur_intersection = self.get_or_create_intersection(cfg.longitude, cfg.latitude)


    def dijkstra(self, target_coordinates):
        start_intersection = find_intersection(cfg.longitude, cfg.latitude)
        target_intersection = find_intersection_coordinates(target_coordinates)

        # Clear cfg.heading to targets
        for intersection in cfg.intersections:
            intersection.headingToTarget = None

        seen = set()
        queue = [target_intersection]
        path_length = 0

        # BFS
        while len(queue) > 0:
            cur_intersection = queue.pop(0)
            path_length += 1

            for neighbor_intersection, direction in cur_intersection.get_neighbor_intersections().items():
                # Ignore blocked intersections 
                if neighbor_intersection.blocked:
                    continue

                if neighbor_intersection not in seen:
                    queue.append(neighbor_intersection)
                    neighbor_intersection.headingToTarget = (direction + 2) % 4
                    seen.add(neighbor_intersection)

                if neighbor_intersection == start_intersection:
                    target_intersection.headingToTarget = STOP
                    return path_length

        return float('inf')


    def drive_back(self, target_coordinates):
        cur_intersection = find_intersection(cfg.longitude, cfg.latitude)
        print('start: ', cur_intersection)

        while cur_intersection.headingToTarget != STOP:
            best_path_length = self.dijkstra(target_coordinates)
            if best_path_length == float('inf'):
                print('No path!')
                return

            print('cur intersection:', cur_intersection, 'heading:', cfg.heading)

            # Look at all blocked paths 
            for dir in cur_intersection.get_streets(BLOCKED):

                # Determine potential path if street unblocked 
                cur_intersection.streets[dir] = CONNECTED
                set_opposing_intersection(cur_intersection, CONNECTED, dir)
                unblocked_path_length = self.dijkstra(target_coordinates)

                # Backtrack 
                cur_intersection.streets[dir] = BLOCKED
                set_opposing_intersection(cur_intersection, BLOCKED, dir)

                # If potentially better path, check if street is still blocked
                if unblocked_path_length < best_path_length:
                    best_path_length = unblocked_path_length
                    self.turn_to_direction(dir, cur_intersection) 

                    if not self.street_blocked():
                        cur_intersection.streets[dir] = CONNECTED
                        set_opposing_intersection(cur_intersection, CONNECTED)

            self.dijkstra(target_coordinates)
            
            # Turn toward next intersection 
            self.turn_to_direction(cur_intersection.headingToTarget, cur_intersection)

            # If new obstacle, rerun dijkstra
            if self.street_blocked():
                cur_intersection.streets[cfg.heading] = BLOCKED
                set_opposing_intersection(cur_intersection, BLOCKED)

                self.dijkstra(target_coordinates)
                continue

            # Try to drive to next intersection 
            # If next intersection blocked, rerun dijkstra
            if not self.follow_line_avoiding_obstacles(): 
                self.dijkstra(target_coordinates)
                continue 

            # Re-orient bot 
            cfg.longitude, cfg.latitude = shift(cfg.longitude, cfg.latitude, cfg.heading)
            cur_intersection = find_intersection(cfg.longitude, cfg.latitude)

        print('Found target!')

    def go_to_intersection(self, lon, lat):
        target_coordinates = int(lon), int(lat)

        path_length = self.dijkstra(target_coordinates)
        if path_length == float('inf'):
            print('No path!')
        else:
            self.drive_back(target_coordinates)

    # Returns True if successfully drove to next intersection
    # False if it turned around 
    def follow_line_avoiding_obstacles(self):
        while True: 
            state = self.follow_line()

            if state == BLOCKED: 
                print('intersection blocked')
                # Set intersection it's cfg.heading towards to be blocked 
                lon, lat = shift(cfg.longitude, cfg.latitude, cfg.heading)
                cur_intersection = find_intersection(lon, lat)
                if cur_intersection is None: 
                    cur_intersection = Intersection(lon, lat)

                cur_intersection.blocked = True

                # Turn back and drive to previous intersection 
                self.turn_to_next_street()
                self.follow_line()
                return False 

            # if bot reaches intersection, don't do anything
            else:
                return True

    def stop_current_task(self):
        self.stop_task = True 
        self.pause_all_tasks = True 

    def start_task(self, task_name, *args):
        self.current_task = task_name
        self.current_args = args 
        self.pause_all_tasks = False 

    def is_valid_task(self, task_name):
        return task_name in self.task_name_to_command

    def driving_loop(self):
        while True:
            if self.stop_task:
                self.stop_task = False  
                
                if not self.pause_all_tasks:
                    command = self.task_name_to_command[self.current_task]
                    command(*self.current_args)







