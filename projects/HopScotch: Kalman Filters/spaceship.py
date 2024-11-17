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

# If you see different scores locally and on Gradescope this may be an
# indication that you are uploading a different file than the one you are
# executing locally. If this local ID doesn't match the ID on Gradescope then
# you uploaded a different file.
OUTPUT_UNIQUE_FILE_ID = False
if OUTPUT_UNIQUE_FILE_ID:
    import hashlib, pathlib
    file_hash = hashlib.md5(pathlib.Path(__file__).read_bytes()).hexdigest()
    print(f'Unique file ID: {file_hash}')

from rait import *
import math

class Spaceship():
    """The Spaceship to guide across the galaxy."""

    def __init__(self, bounds, xy_start):
        """Initialize the Spaceship."""
        self.x_bounds = bounds['x']
        self.y_bounds = bounds['y']
        self.agent_pos_start = xy_start

        # store estimated asteroid location, velocity, acceleration, and covariance information
        self.asteroid_estimations = {}

        # initialize the time step and measurement noise
        self.dt = 1
        self.sigma_r = 1

        # The matrices below are adapted from the kf-tune PDF:
        # https://gatech.instructure.com/courses/402244/files/folder/Kalman%20Filter%20Tutorial?

        # define the state transition matrix
        self.F = matrix([
            [1, 0, self.dt, 0, 0.5*self.dt**2, 0],
            [0, 1, 0, self.dt, 0, 0.5*self.dt**2],
            [0, 0, 1, 0, self.dt, 0],
            [0, 0, 0, 1, 0, self.dt],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]])

        # initialize the measurement matrix
        self.H = matrix([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0]])

        # define the observational uncertainty matrix
        self.R = matrix([
            [(1.5*self.sigma_r)**2, 0],
            [0, (1.5*self.sigma_r)**2]])

        # initialize the covariance matrix
        self.P = matrix([
            [25, 0, 0, 0, 0, 0],
            [0, 25, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0.01, 0],
            [0, 0, 0, 0, 0, 0.01]])

        # initialize the state noise matrix
        self.Q = matrix([
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0]])

        # End of code adapted from kf-tune PDF:
        # https://gatech.instructure.com/courses/402244/files/folder/Kalman%20Filter%20Tutorial?

        # Part B: keep track of the ID of the ridden asteroid
        self.ridden_asteroid = None


    def predict_from_observations(self, asteroid_observations):
        """Observe asteroid locations and predict their positions at time t+1.
        Parameters
        ----------
        self = a reference to the current object, the Spaceship.
        asteroid_observations = A dictionary of tuples containing i: (x, y), where i, x, and y are:
        i = the asteroid's ID
        x = noisy x-coordinate observations taken at time t
        y = noisy y-coordinate observations taken at time t.

        asteroid_observations format:
        ```
        `{1: (x-measurement, y-measurement),
          2: (x-measurement, y-measurement)...
          100: (x-measurement, y-measurement),
          }`
        ```

        Returns
        -------
        The output of the `predict_from_observations` function should be a dictionary of tuples
        of estimated asteroid locations one timestep into the future
        (i.e. the inputs are for measurements taken at time t, and you return where the asteroids will be at time t+1).

        A dictionary of tuples containing i: (x, y), where i, x, and y are:
        i = the asteroid's ID
        x = the estimated x-coordinate of asteroid i's position for time t+1
        y = the estimated y-coordinate of asteroid i's position for time t+1
        Return format:
        `{1: (x-coordinate, y-coordinate),
          2: (x-coordinate, y-coordinate)...
          100: (x-coordinate, y-coordinate)
          }`
        """
        # To view the visualization with the default pdf output (incorrect) uncomment the line below
        # return asteroid_observations

        # FOR STUDENT TODO: Update the Spaceship's estimate of where the asteroids will be located in the next time step
        for asteroid_id, (x, y) in asteroid_observations.items():

            # initialize asteroid states with no previous information
            if asteroid_id not in self.asteroid_estimations:
                self.asteroid_estimations[asteroid_id] = {
                    'state': matrix([[x, y, 0, 0, 0, 0]]).transpose(),
                    'covariance': self.P}

            # The matrices below are adapted from the kf-tune PDF:
            # https://gatech.instructure.com/courses/402244/files/folder/Kalman%20Filter%20Tutorial?

            # get the previous state and covariance
            xe = self.asteroid_estimations[asteroid_id]['state']
            P = self.asteroid_estimations[asteroid_id]['covariance']

            # get the observed x and y locations at time t
            z = matrix([[x, y]]).transpose()

            # update the state and covariance with the measurement
            S = self.H * P * self.H.transpose() + self.R
            K = P * self.H.transpose() * S.inverse()
            y = z - self.H * xe
            xe += K * y
            P -= K * self.H * P

            # predict for time t+1
            xe = self.F * xe
            P = self.F * P * self.F.transpose() + self.Q

            # End of code adapted from kf-tune PDF:
            # https://gatech.instructure.com/courses/402244/files/folder/Kalman%20Filter%20Tutorial?

            # update the dictionary with state and covariance information for the asteroid
            self.asteroid_estimations[asteroid_id]['state'] = xe
            self.asteroid_estimations[asteroid_id]['covariance'] = P

            # update the asteroid observations dictionary
            asteroid_observations[asteroid_id] = (xe[0][0], xe[1][0])

        return asteroid_observations


    def jump(self, asteroid_observations, agent_data):
        """ Return the id of the asteroid the spaceship should jump/hop onto in the next timestep
        ----------
        self = a reference to the current object, the Spaceship
        asteroid_observations: Same as predict_from_observations method
        agent_data: a dictionary containing agent related data:
        'jump_distance' - a float representing agent jumping distance,
        'ridden_asteroid' - an int representing the ID of the ridden asteroid if available, None otherwise.
        Note: 'agent_pos_start' - A tuple representing the (x, y) position of the agent at t=0 is available in the constructor.

        agent_data format:
        {'ridden_asteroid': None,
         'jump_distance': agent.jump_distance,
         }
        Returns
        -------
        You are to return two items.
        1: idx, this represents the ID of the asteroid on which to jump if a jump should be performed in the next timestep.
        Return None if you do not intend to jump on an asteroid in the next timestep
        2. Return the estimated positions of the asteroids (i.e. the output of 'predict_from_observations method)
        IFF you intend to have them plotted in the visualization. Otherwise return None
        -----
        an example return
        idx to hop onto in the next timestep: 3,
        estimated_results = {1: (x-coordinate, y-coordinate),
          2: (x-coordinate, y-coordinate)}

        return 3, estimated_return

        """
        # FOR STUDENT TODO: Update the idx of the asteroid on which to jump

        # get the current position
        if agent_data['ridden_asteroid']:
            curr_x, curr_y = asteroid_observations[agent_data['ridden_asteroid']]
            self.agent_pos_start = (curr_x, curr_y)
        else:
            curr_x, curr_y = self.agent_pos_start

        # make predictions on asteroid locations
        predicted_locations = self.predict_from_observations(asteroid_observations)

        # introduce a buffer to keep the spaceship within bounds and in range of asteroids
        jump_buffer = 0.11
        bounds_buffer = 0.1
        x_lower = self.x_bounds[0] + bounds_buffer
        x_upper = self.x_bounds[1] - bounds_buffer
        y_lower = self.y_bounds[0]
        y_upper = self.y_bounds[1] - bounds_buffer
        safe_jump_radius = agent_data['jump_distance'] - jump_buffer

        # select asteroids whose position is within bounds
        bounded_locations = {
            asteroid_id: (x, y) for asteroid_id, (x, y) in predicted_locations.items()
            if x_lower <= x <= x_upper and y_lower <= y <= y_upper}

        # select asteroids within the spaceship's jump radius
        jump_radius_locations = {}
        for asteroid_id, (x, y) in bounded_locations.items():
            euclidean_distance = math.sqrt((x - curr_x)**2 + (y - curr_y)**2)
            if euclidean_distance <= safe_jump_radius:
                jump_radius_locations[asteroid_id] = (x, y, euclidean_distance)

        # sort the Euclidean distances to find the closest asteroids
        jump_radius_locations = dict(sorted(jump_radius_locations.items(), key=lambda x:x[1][2]))

        # remove the distances from the dictionary
        jump_radius_locations = {
            asteroid_id: (x, y) for asteroid_id, (x, y, d) in jump_radius_locations.items()}

        # select asteroids with positive y velocity and dy/dx greater than 1
        good_velocities = {}
        for asteroid_id, (x, y) in jump_radius_locations.items():
            x_veloc = self.asteroid_estimations[asteroid_id]['state'][2][0]
            y_veloc = self.asteroid_estimations[asteroid_id]['state'][3][0]

            if y_veloc > 0 and x_veloc == 0:
                good_velocities[asteroid_id] = (x, y, y_veloc)
            elif y_veloc > 0 and x_veloc != 0:
                dy_dx = abs(y_veloc / x_veloc)
                if dy_dx > 1:
                    good_velocities[asteroid_id] = (x, y, dy_dx)

        # sort by greatest dy/dx ratio
        good_velocities = dict(sorted(good_velocities.items(), key=lambda x:x[1][2], reverse=True))

        # remove the ratios from the dictionary
        good_velocities = {
            asteroid_id: (x, y) for asteroid_id, (x, y, r) in good_velocities.items()}

        # jump to a remaining candidate whose ID is not shared with agent data
        idx = None
        if good_velocities:
            for asteroid_id in good_velocities:
                if asteroid_id != agent_data['ridden_asteroid']:
                    idx = asteroid_id
                    self.agent_pos_start = good_velocities[idx]
                    break

        return idx, predicted_locations


def who_am_i():
    # Please specify your GT login ID in the whoami variable (ex: jsmith224).
    whoami = 'wnobles6'
    return whoami
