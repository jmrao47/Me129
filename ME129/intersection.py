import random 

# Headings
NORTH = 0
WEST = 1
SOUTH = 2
EAST = 3

HEADING = {NORTH:'North', WEST:'West ', SOUTH:'South', EAST:'East', None:'None'} 

# Street status
UNKNOWN = 'Unknown'
NOSTREET = 'NoStreet'
UNEXPLORED = 'Unexplored'
CONNECTED = 'Connected'
STOP = 'Stop'

# Global Variables:
intersections = [] # List of intersections
last_intersection = None # Last intersection visited
longitude = 0 # Current east/west coordinate
latitude = 0 # Current north/south coordinate
heading = NORTH # Current heading

# New longitudeitude/latitudeitude value after a step in the given heading.
def shift(longitude, latitude, heading):
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
    list = [i for i in intersections if i.longitude == longitude and i.latitude == latitude]
    if len(list) == 0:
        return None
    if len(list) > 1:
        raise Exception("Multiple intersections at (%2d,%2d)" % (longitude, latitude))
    return list[0]

def find_intersection_coordinates(coordinates):
    lon, lat = coordinates
    return find_intersection(lon, lat)

class Intersection:
    # Initialize - create new intersection at (longitude, let)
    def __init__(self, longitude, latitude):
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
        # Add this to the global list of intersections to make it searchable.
        global intersections
        if find_intersection(longitude, latitude) is not None:
            raise Exception("Duplicate intersection at (%2d,%2d)" % (longitude,latitude))
        intersections.append(self)

    # Print format
    def __repr__(self):
        return ("(%2d, %2d) N:%s W:%s S:%s E:%s - head %s\n" %
        (self.longitude, self.latitude, self.streets[0],
        self.streets[1], self.streets[2], self.streets[3],
        HEADING[self.headingToTarget]))

    def get_streets(self, street_status):
        return [i for i in range(4) if self.streets[i] == street_status]

    def get_neighbor_intersections(self):
        neighbors = {}

        for direction in range(4):
            if self.streets[direction] == CONNECTED:
                neighbor_lon, neighbor_lat = shift(self.longitude, self.latitude, direction)
                neighbor_intersection = find_intersection(neighbor_lon, neighbor_lat)
                neighbors[neighbor_intersection] = direction

        return neighbors

    def get_random_street(self, street_status):
        indices = [i for i in range(len(self.streets)) if self.streets[i] == street_status]
        return random.choice(indices)
    

