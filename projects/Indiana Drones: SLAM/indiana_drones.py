######################################################################
# This file copyright the Georgia Institute of Technology
#
# Permission is given to students to use or modify this file (only)
# to work on their assignments.
#
# You may NOT publish this file or make it available to others not in
# the course.
#
######################################################################

"""
 === Introduction ===

   The assignment is broken up into two parts.

   Part A:
        Create a SLAM implementation to process a series of landmark measurements (location of tree centers) and movement updates.
        The movements are defined for you so there are no decisions for you to make, you simply process the movements
        given to you.
        Hint: A planner with an unknown number of motions works well with an online version of SLAM.

    Part B:
        Here you will create the action planner for the drone.  The returned actions will be executed with the goal being to navigate to
        and extract the treasure from the environment marked by * while avoiding obstacles (trees).
        Actions:
            'move distance steering'
            'extract treasure_type x_coordinate y_coordinate'
        Example Actions:
            'move 1 1.570963'
            'extract * 1.5 -0.2'

    Note: All of your estimates should be given relative to your drone's starting location.

    Details:
    - Start position
      - The drone will land at an unknown location on the map, however, you can represent this starting location
        as (0,0), so all future drone location estimates will be relative to this starting location.
    - Measurements
      - Measurements will come from trees located throughout the terrain.
        * The format is {'landmark id':{'distance':0.0, 'bearing':0.0, 'type':'D', 'radius':0.5}, ...}
      - Only trees that are within the horizon distance will return measurements. Therefore new trees may appear as you move through the environment.
    - Movements
      - Action: 'move 1.0 1.570963'
        * The drone will turn counterclockwise 90 degrees [1.57 radians] first and then move 1.0 meter forward.
      - Movements are stochastic due to, well, it being a robot.
      - If max distance or steering is exceeded, the drone will not move.
      - Action: 'extract * 1.5 -0.2'
        * The drone will attempt to extract the specified treasure (*) from the current location of the drone (1.5, -0.2).
      - The drone must be within 0.25 distance to successfully extract a treasure.

    The drone will always execute a measurement first, followed by an action.
    The drone will have a time limit of 10 seconds to find and extract all of the needed treasures.
"""

from math import *
from typing import Dict, List
from rait import matrix

# If you see different scores locally and on Gradescope this may be an indication
# that you are uploading a different file than the one you are executing locally.
# If this local ID doesn't match the ID on Gradescope then you uploaded a different file.
OUTPUT_UNIQUE_FILE_ID = False
if OUTPUT_UNIQUE_FILE_ID:
    import hashlib, pathlib
    file_hash = hashlib.md5(pathlib.Path(__file__).read_bytes()).hexdigest()
    print(f'Unique file ID: {file_hash}')

class SLAM:
    """Create a basic SLAM module.
    """

    def __init__(self):
        """Initialize SLAM components here.
        """

        # The following code is adapted from:
        # https://gatech.instructure.com/courses/402244/pages/2-online-slam-answer?module_item_id=4089292
        # Problem 6: Online SLAM (Answer)

        # initialize the omega and xi matricies
        self.omega = matrix()
        self.omega.zero(2, 2)
        self.omega.value[0][0] = 1
        self.omega.value[1][1] = 1

        self.xi = matrix()
        self.xi.zero(2, 1)
        self.xi.value[0][0] = 0
        self.xi.value[1][0] = 0

        # End of code citation

        # initialize the drone's orientation
        self.orientation = 0

        # keep track of the landmarks
        self.landmarks = {}

        # set the noise
        self.measurement_noise = 1
        self.motion_noise = 1

    # Provided Functions
    def get_coordinates(self):
        """
        Retrieves the estimated (x, y) locations in meters of the drone and all landmarks (trees) when called.

        Args: None

        Returns:
            The (x,y) coordinates in meters of the drone and all landmarks (trees) in the format:
                    {
                        'self': (x, y),
                        '<landmark_id_1>': (x1, y1),
                        '<landmark_id_2>': (x2, y2),
                        ....
                    }
        """

        # The following code is adapted from:
        # https://gatech.instructure.com/courses/402244/pages/2-online-slam-answer?module_item_id=4089292
        # Problem 6: Online SLAM (Answer)

        # solve the matrices
        mu = self.omega.inverse() * self.xi

        # End of code citation

        # initialize a dictionary with the drone's coordinates
        coordinates = {'self': (mu.value[0][0], mu.value[1][0])}

        # add landmarks and their coordinates to the dictionary
        for landmark_id, index in self.landmarks.items():
            coordinates[landmark_id] = (mu.value[2*(index+1)][0], mu.value[2*(index+1)+1][0])

        return coordinates

    def process_measurements(self, measurements: Dict):
        """
        Process a new series of measurements and update (x,y) location of drone and landmarks

        Args:
            measurements: Collection of measurements of tree positions and radius in the format
            {'landmark id':{'distance': float <meters>, 'bearing':float <radians>, 'type': char,
            'radius':float <meters>}, ...}
        """

        for landmark_id, measurement in measurements.items():

            if landmark_id not in self.landmarks:

                # add new landmarks to the dictionary
                self.landmarks[landmark_id] = len(self.landmarks)

                # The following code is adapted from:
                # https://gatech.instructure.com/courses/402244/pages/2-online-slam-answer?module_item_id=4089292
                # Problem 6: Online SLAM (Answer)

                # expand the matrices
                dim = len(self.omega.value)
                idxs = list(range(dim))
                self.omega = self.omega.expand(dim+2, dim+2, idxs, idxs)
                self.xi = self.xi.expand(dim+2, 1, idxs, [0])

                # End of code citation

            # calculate the distances
            distance = measurement['distance']
            bearing = measurement['bearing']
            dx = distance * cos(self.orientation + bearing)
            dy = distance * sin(self.orientation + bearing)

            # The following code is adapted from:
            # https://gatech.instructure.com/courses/402244/pages/2-online-slam-answer?module_item_id=4089292
            # Problem 6: Online SLAM (Answer)

            # m is the index of the landmark coordinate in the matrix/vector
            m = 2 * (1 + self.landmarks[landmark_id])

            # update the information matrix/vector based on the measurement
            for b in range(2):
                self.omega.value[b][b] += 1 / self.measurement_noise
                self.omega.value[m+b][m+b] += 1 / self.measurement_noise
                self.omega.value[b][m+b] += -1 / self.measurement_noise
                self.omega.value[m+b][b] += -1 / self.measurement_noise

                if b == 0:
                    self.xi.value[b][0] += -dx / self.measurement_noise
                    self.xi.value[m+b][0] += dx / self.measurement_noise
                else:
                    self.xi.value[b][0] += -dy / self.measurement_noise
                    self.xi.value[m+b][0] += dy / self.measurement_noise

            # End of code citation

    def process_movement(self, distance: float, steering: float):
        """
        Process a new movement and update (x,y) location of drone

        Args:
            distance: distance to move in meters
            steering: amount to turn in radians
        """

        # The following code is adapted from:
        # https://gatech.instructure.com/courses/402244/pages/2-online-slam-answer?module_item_id=4089292
        # Problem 6: Online SLAM (Answer)

        # set the dimension of the filter
        dim = 2 * (1 + len(self.landmarks))

        # expand the information matrix/vector by one new position
        idxs = [0, 1] + list(range(4, dim+2))
        self.omega = self.omega.expand(dim+2, dim+2, idxs, idxs)
        self.xi = self.xi.expand(dim+2, 1, idxs, [0])

        # End of code citation

        # update the drone's orientation
        self.orientation += steering

        # calculate the distances
        dx = distance * cos(self.orientation)
        dy = distance * sin(self.orientation)

        # The following code is adapted from:
        # https://gatech.instructure.com/courses/402244/pages/2-online-slam-answer?module_item_id=4089292
        # Problem 6: Online SLAM (Answer)

        # update the information matrix/vector based on the robot motion
        for b in range(4):
            self.omega.value[b][b] += 1 / self.motion_noise
        for b in range(2):
            self.omega.value[b][b+2] += -1 / self.motion_noise
            self.omega.value[b+2][b] += -1 / self.motion_noise
            if b == 0:
                self.xi.value[b][0] += -dx / self.motion_noise
                self.xi.value[b+2][0] += dx / self.motion_noise
            else:
                self.xi.value[b][0] += -dy / self.motion_noise
                self.xi.value[b+2][0] += dy / self.motion_noise

        # factor out the previous pose
        new_idxs = list(range(2, len(self.omega.value)))
        a = self.omega.take([0, 1], new_idxs)
        b = self.omega.take([0, 1])
        c = self.xi.take([0, 1], [0])
        self.omega = self.omega.take(new_idxs) - a.transpose() * b.inverse() * a
        self.xi = self.xi.take(new_idxs, [0]) - a.transpose() * b.inverse() * c

        # End of code citation


class IndianaDronesPlanner:
    """
    Create a planner to navigate the drone to reach and extract the treasure marked by * from an unknown start position while avoiding obstacles (trees).
    """

    def __init__(self, max_distance: float, max_steering: float):
        """
        Initialize your planner here.

        Args:
            max_distance: the max distance the drone can travel in a single move in meters.
            max_steering: the max steering angle the drone can turn in a single move in radians.
        """

        self.max_distance = max_distance
        self.max_steering = max_steering
        self.slam = SLAM()

    def next_move(self, measurements: Dict, treasure_location: Dict):
        """Next move based on the current set of measurements.

        Args:
            measurements: Collection of measurements of tree positions and radius in the format
                          {'landmark id':{'distance': float <meters>, 'bearing':float <radians>, 'type': char, 'radius':float <meters>}, ...}
            treasure_location: Location of Treasure in the format {'x': float <meters>, 'y':float <meters>, 'type': char '*'}

        Return: action: str, points_to_plot: dict [optional]
            action (str): next command to execute on the drone.
                allowed:
                    'move distance steering'
                    'move 1.0 1.570963'  - Turn left 90 degrees and move 1.0 distance.

                    'extract treasure_type x_coordinate y_coordinate'
                    'extract * 1.5 -0.2' - Attempt to extract the treasure * from your current location (x = 1.5, y = -0.2).
                                           This will succeed if the specified treasure is within the minimum sample distance.

            points_to_plot (dict): point estimates (x,y) to visualize if using the visualization tool [optional]
                            'self' represents the drone estimated position
                            <landmark_id> represents the estimated position for a certain landmark
                format:
                    {
                        'self': (x, y),
                        '<landmark_id_1>': (x1, y1),
                        '<landmark_id_2>': (x2, y2),
                        ....
                    }
        """

        # process the measurements and get drone, landmark locations
        self.slam.process_measurements(measurements)
        points_to_plot = self.slam.get_coordinates()

        # get the drone location and treasure location
        drone_x, drone_y = points_to_plot['self']
        treasure_type, treasure_x, treasure_y = treasure_location.values()

        # calculate the distance and steering necessary to reach the treasure
        dx = treasure_x - drone_x
        dy = treasure_y - drone_y
        distance = sqrt(dx ** 2 + dy ** 2)
        treasure_angle = atan2(dy, dx)
        steering = treasure_angle - self.slam.orientation

        # extract the treasure if close enough
        if distance <= 0.25:
            action = f"extract {treasure_type} {drone_x} {drone_y}"
            return action, points_to_plot

        # limit the distance and steering angle
        distance = min(distance, self.max_distance)
        steering = min(max(-self.max_steering, steering), self.max_steering)

        # process the movement
        self.slam.process_movement(distance, steering)

        action = f"move {distance} {steering}"

        return action, points_to_plot


def who_am_i():
    # Please specify your GT login ID in the whoami variable (ex: jsmith224).
    whoami = 'wnobles6'
    return whoami
