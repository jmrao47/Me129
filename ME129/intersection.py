import random 
import pickle 

import cfg 

# cfg.headings
NORTH = 0
WEST = 1
SOUTH = 2
EAST = 3
STOP = 'Stop'

HEADING = {NORTH:'north', WEST:'west ', SOUTH:'south', EAST:'east', None:'none', STOP:'stop'} 
HEADING_TO_INDEX = {v: k for k, v in HEADING.items()}

# Street status
UNKNOWN = 'Unknown'
NOSTREET = 'NoStreet'
UNEXPLORED = 'Unexplored'
CONNECTED = 'Connected'
BLOCKED = 'Blocked'

NOPATH = 'NoPath'

# cfg.heading is a number 
def orient_bot(lon, lat, head):
    cfg.longitude = int(lon) 
    cfg.latitude = int(lat) 
    cfg.heading = HEADING_TO_INDEX[head.lower()] 
    
def print_map():
    print(cfg.intersections)

def clear_map():
    cfg.intersections = []

def save_map(filename):
    filehandler = open(filename, 'wb') 
    pickle.dump(cfg.intersections, filehandler)

def load_map(filename):
    filehandler = open(filename, 'rb') 
    cfg.intersections = pickle.load(filehandler)

def distance_between(i1, i2):
    def get_coordinates(i):
        if type(i) is tuple:
            return i 
        return i.longitude, i.latitude

    x1, y1 = get_coordinates(i1)
    x2, y2 = get_coordinates(i2)

    return abs(x1 - x2) + abs(y1 - y2)

# New cfg.longitudeitude/cfg.latitudeitude value after a step in the given cfg.heading.
def shift(longitude, latitude, heading):
    #print(f'SHIFT! lon: {cfg.longitude}, lat: {cfg.latitude}, cfg.heading: {cfg.heading}')
    if heading % 4 == NORTH:
        return (longitude, latitude+1)
    elif heading % 4 == WEST:
        return (longitude-1, latitude)
    elif heading % 4 == SOUTH:
        return (longitude, latitude-1)
    elif heading % 4 == EAST:
        return (longitude+1, latitude)
    else:
        raise Exception("This can’t be")

# Find the intersection
def find_intersection(longitude, latitude):
    list = [i for i in cfg.intersections if i.longitude == longitude and i.latitude == latitude]
    if len(list) == 0:
        return None
    if len(list) > 1:
        raise Exception("Multiple cfg.intersections at (%2d,%2d)" % (longitude, latitude))
    return list[0]

def find_intersection_coordinates(coordinates):
    lon, lat = coordinates
    return find_intersection(lon, lat)

def turn_distance_and_direction(cardinal_direction):
    left_turns = (cardinal_direction - cfg.heading + 4) % 4
    right_turns = (cfg.heading - cardinal_direction + 4) % 4

    distance = min(left_turns, right_turns)
    direction = 1 if left_turns < right_turns else -1

    return distance, direction 
    
def turn_distance(cardinal_direction):
    distance, _ = turn_distance_and_direction(cardinal_direction)
    return distance 

# Turn direction: 1 is left, -1 is right 
def turn_direction(cardinal_direction):
    _, direction = turn_distance_and_direction(cardinal_direction)
    return direction

def get_opposing_intersection(cur_intersection, direction=None):
    if direction is None:
        direction = cfg.heading

    return find_intersection_coordinates(shift(cfg.longitude, cfg.latitude, direction))

def set_opposing_intersection(cur_intersection, street_status, direction=None):
    if direction is None:
        direction = cfg.heading

    opposing_intersection = get_opposing_intersection(cur_intersection, direction)

    if opposing_intersection is not None:
        opposite_heading = (direction + 2) % 4
        opposing_intersection.streets[opposite_heading] = street_status 

class Intersection:
    # Initialize - create new intersection at (cfg.longitude, let)
    def __init__(self, longitude, latitude):
        # Whether there is an obstacle at the intersection 
        self.blocked = False 

        # Save the parameters.
        self.longitude = longitude
        self.latitude = latitude

        # Status of streets at the intersection, in NWSE directions.
        self.streets = [UNKNOWN, UNKNOWN, UNKNOWN, UNKNOWN]

        # Direction to head from this intersection in planned move.
        self.headingToTarget = None

        # You are welcome to implement an arbitrary number of
        # "neighbors" to more directly match a general graph.
        # But the above should be sufficient for a regular grid.
        # Add this to the global list of cfg.intersections to make it searchable.

        if find_intersection(longitude, latitude) is not None:
            raise Exception("Duplicate intersection at (%2d,%2d)" % (longitude, latitude))
        cfg.intersections.append(self)

    # Print format
    def __repr__(self):
        return f'({self.longitude}, {self.latitude}), N:{self.streets[0]},\
        W:{self.streets[1]}, S:{self.streets[2]}, E:{self.streets[3]},\
        cfg.heading: {HEADING[self.headingToTarget]}, BLOCKED: {self.blocked}\n'

    def get_streets(self, street_status):
        return [i for i in range(4) if self.streets[i] == street_status]

    def get_closest_neighbor_direction(self, target_coordinates, street_status):
        min_dist = float('inf')
        best_direction = None

        for direction in self.get_streets(street_status):
            neighbor_coordinates = shift(self.longitude, self.latitude, direction)
            dist = distance_between(target_coordinates, neighbor_coordinates)

            if dist < min_dist: 
                min_dist = dist 
                best_direction = direction

        return best_direction


    def get_neighbor_intersections(self):
        neighbors = {}

        for direction in self.get_streets(CONNECTED):
            neighbor_lon, neighbor_lat = shift(self.longitude, self.latitude, direction)
            neighbor_intersection = find_intersection(neighbor_lon, neighbor_lat)

            if neighbor_intersection is not None:
                neighbors[neighbor_intersection] = direction

        return neighbors

    def get_random_street(self, street_status):
        indices = self.get_streets(street_status)
        return random.choice(indices)

    def get_closest_street(self, street_status):
        directions = self.get_streets(street_status)

        closest_dir_distance = float('inf')
        closest_dir = None

        for dir in directions:
            dist = turn_distance(dir)

            if dist < closest_dir_distance:
                closest_dir_distance = dist
                closest_dir = dir 

        return closest_dir

    def get_coordinates(self):
        return self.longitude, self.latitude
    

