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

# These import statements give you access to library functions which you may
# (or may not?) want to use.
import random
import time
from math import *
from body import *
from solar_system import *
from satellite import *


def estimate_next_pos(gravimeter_measurement, get_theoretical_gravitational_force_at_point, distance, steering, other=None):
    """
    Estimate the next (x,y) position of the satelite.
    This is the function you will have to write for part A.
    :param gravimeter_measurement: float
        A floating point number representing
        the measured magnitude of the gravitation pull of all the planets
        felt at the target satellite at that point in time.
    :param get_theoretical_gravitational_force_at_point: Func
        A function that takes in (x,y) and outputs a float representing the magnitude of the gravitation pull from
        of all the planets at that (x,y) location at that point in time.
    :param distance: float
        The target satellite's motion distance
    :param steering: float
        The target satellite's motion steering
    :param other: any
        This is initially None, but if you return an OTHER from
        this function call, it will be passed back to you the next time it is
        called, so that you can use it to keep track of important information
        over time. (We suggest you use a dictionary so that you can store as many
        different named values as you want.)
    :return:
        estimate: Tuple[float, float]. The (x,y) estimate of the target satellite at the next timestep
        other: any. Any additional information you'd like to pass between invocations of this function
        optional_points_to_plot: List[Tuple[float, float, float]].
            A list of tuples like (x,y,h) to plot for the visualization
    """
    # time.sleep(1)  # uncomment to pause for the specified seconds each timestep

    num_particles = 1000
    satellite_length = 10.2
    sigma = 2.0e-7
    fuzzing_noise = 0.1 * AU
    percentage_to_fuzz = 0.2
    num_particles_to_fuzz = int(percentage_to_fuzz * num_particles)

    # initialize the particles
    if other is None:
        particles = []
        for _ in range(num_particles):
            x = random.uniform(-4 * AU, 4 * AU)
            y = random.uniform(-4 * AU, 4 * AU)
            heading = atan2(y, x) + 0.5 * pi
            mass = random.uniform(1000, 100000)
            particles.append(Satellite(x, y, heading, satellite_length, mass, 0, 0))
        other = {'particles': particles}
    else:
        particles = other['particles']

    # calculate weights
    weights = []
    for particle in particles:
        theoretical_force = get_theoretical_gravitational_force_at_point(particle.x, particle.y)
        weight = exp(-0.5 * ((theoretical_force - gravimeter_measurement) / sigma) ** 2) / (sigma * sqrt(2 * pi))
        weights.append(weight)

    # resample the particles
    # The code below is adapted from:
    # https://gatech.instructure.com/courses/402244/pages/21-resampling-wheel-answer?module_item_id=4088900
    # Particle Filters: Resampling Wheel (Answer)
    resampled_particles = []
    index = int(random.random() * num_particles)
    beta = 0.0
    max_weight = max(weights)
    for _ in range(len(particles)):
        beta += random.random() * 2.0 * max_weight
        while beta > weights[index]:
            beta -= weights[index]
            index = (index + 1) % num_particles
        resampled_particles.append(particles[index])
    # end of code citation

    # add roughening
    roughened_particles = []
    for i, particle in enumerate(resampled_particles):

        if i < num_particles_to_fuzz:

            # add noise to the particle's position
            x_new = particle.x + random.gauss(0, fuzzing_noise)
            y_new = particle.y + random.gauss(0, fuzzing_noise)

            # initialize a new Satellite object
            new_particle = Satellite(x_new, y_new, particle.h, satellite_length, random.uniform(1000, 100000), particle.g_measurement_noise, particle.percent_illuminated_measurement_noise)
        else:
            new_particle = particle

        # add the particle to a list of updated particles
        roughened_particles.append(new_particle)

    # move the particles and thus predict the target's next position
    new_particles = []
    for particle in roughened_particles:

        # The code below is adapted from:
        # https://gatech.instructure.com/courses/402244/pages/4-circular-motion-answer?module_item_id=4089022
        # Particle Filters: Circular Motion (Answer)

        # get the turning angle
        turning_angle = distance / satellite_length * tan(steering)

        # straight line motion
        if abs(turning_angle) < 0.001:
            x_new = particle.x + distance * cos(particle.h)
            y_new = particle.y + distance * sin(particle.h)
            heading_new = (particle.h + turning_angle) % (2 * pi)

        # cyclic motion
        else:
            radius = distance / turning_angle
            center_x = particle.x - sin(particle.h) * radius
            center_y = particle.y + cos(particle.h) * radius
            x_new = center_x + sin(particle.h + turning_angle) * radius
            y_new = center_y - cos(particle.h + turning_angle) * radius
            heading_new = atan2(y_new - center_y, x_new - center_x) + 0.5 * pi

        # end of code citation

        # initialize a new Satellite object and add it to a list of updated particles
        new_particle = Satellite(x_new, y_new, heading_new, satellite_length, random.uniform(1000, 100000), particle.g_measurement_noise, particle.percent_illuminated_measurement_noise)
        new_particles.append(new_particle)

    # estimate the next state of the target satellite
    total_weight = sum(weights)
    if total_weight > 0:
        x = sum(p.x * w for p, w in zip(new_particles, weights)) / total_weight
        y = sum(p.y * w for p, w in zip(new_particles, weights)) / total_weight
    else:
        x = sum(p.x for p in new_particles) / num_particles
        y = sum(p.y for p in new_particles) / num_particles
    xy_estimate = (x, y)

    # update the dictionary, particles to plot
    other['particles'] = new_particles
    optional_points_to_plot = [(p.x, p.y, p.h) for p in new_particles]

    return xy_estimate, other, optional_points_to_plot


def next_angle(solar_system, percent_illuminated_measurements, percent_illuminated_sense_func,
               distance, steering, other=None):
    """
    Gets the next angle at which to send out an sos message to the home planet,
    the last planet in the solar system.
    This is the function you will have to write for part B.
    :param solar_system: SolarSystem
        A model of the solar system containing the sun and planets as Bodys (contains positions, velocities, and masses)
        Planets are listed in order from closest to furthest from the sun
    :param percent_illuminated_measurements: List[float]
        A list of floating point number from 0 to 100 representing
        the measured percent illumination of each planet in order from closest to furthest to sun
        as seen by the target satellite.
    :param percent_illuminated_sense_func: Func
        A function that takes in (x,y) and outputs the list of percent illuminated measurements of each planet
        as would be seen by satellite at that (x,y) location.
    :param distance: float
        The target satellite's motion distance
    :param steering: float
        The target satellite's motion steering
    :param other: any
        This is initially None, but if you return an OTHER from
        this function call, it will be passed back to you the next time it is
        called, so that you can use it to keep track of important information
        over time. (We suggest you use a dictionary so that you can store as many
        different named values as you want.)
    :return:
        bearing: float. The absolute angle from the satellite to send an sos message between -pi and pi
        xy_estimate: Tuple[float, float]. The (x,y) estimate of the target satellite at the next timestep
        other: any. Any additional information you'd like to pass between invocations of this function
        optional_points_to_plot: List[Tuple[float, float, float]].
            A list of tuples like (x,y,h) to plot for the visualization
    """

    num_particles = 1000
    satellite_length = 10.2
    sigma = 0.5
    fuzzing_noise = 0.1 * AU
    percentage_to_fuzz = 0.2
    num_particles_to_fuzz = int(percentage_to_fuzz * num_particles)

    # initialize the particles
    if other is None:
        particles = []
        for _ in range(num_particles):
            x = random.uniform(-4 * AU, 4 * AU)
            y = random.uniform(-4 * AU, 4 * AU)
            heading = atan2(y, x) + 0.5 * pi
            mass = random.uniform(1000, 100000)
            particles.append(Satellite(x, y, heading, satellite_length, mass, 0, 0))
        other = {'particles': particles}
    else:
        particles = other['particles']

    # calculate weights
    # The code below is adapted from:
    # https://gatech.instructure.com/courses/402244/pages/9-robot-class-details?module_item_id=4088846
    # Particle Filters: Robot Class Details
    weights = []
    for particle in particles:
        weight = 1.0
        theoretical_illuminations = percent_illuminated_sense_func(particle.x, particle.y)
        for target_meas, satellite_meas in zip(percent_illuminated_measurements, theoretical_illuminations):
            weight *= exp(-0.5 * ((satellite_meas - target_meas) / sigma) ** 2) / (sigma * sqrt(2 * pi))
        weights.append(weight)
    # end of code citation

    # resample the particles
    # The code below is adapted from:
    # https://gatech.instructure.com/courses/402244/pages/21-resampling-wheel-answer?module_item_id=4088900
    # Particle Filters: Resampling Wheel (Answer)
    resampled_particles = []
    index = int(random.random() * num_particles)
    beta = 0.0
    max_weight = max(weights)
    for _ in range(len(particles)):
        beta += random.random() * 2.0 * max_weight
        while beta > weights[index]:
            beta -= weights[index]
            index = (index + 1) % num_particles
        resampled_particles.append(particles[index])
    # end of code citation

    # add roughening
    roughened_particles = []
    for i, particle in enumerate(resampled_particles):

        if i < num_particles_to_fuzz:

            # add noise to the particle's position
            x_new = particle.x + random.gauss(0, fuzzing_noise)
            y_new = particle.y + random.gauss(0, fuzzing_noise)

            # initialize a new Satellite object
            new_particle = Satellite(x_new, y_new, particle.h, satellite_length, random.uniform(1000, 100000), particle.g_measurement_noise, particle.percent_illuminated_measurement_noise)
        else:
            new_particle = particle

        # add the particle to a list of updated particles
        roughened_particles.append(new_particle)

    # move the particles and thus predict the target's next position
    new_particles = []
    for particle in roughened_particles:

        # The code below is adapted from:
        # https://gatech.instructure.com/courses/402244/pages/4-circular-motion-answer?module_item_id=4089022
        # Particle Filters: Circular Motion (Answer)

        # get the turning angle
        turning_angle = distance / satellite_length * tan(steering)

        # straight line motion
        if abs(turning_angle) < 0.001:
            x_new = particle.x + distance * cos(particle.h)
            y_new = particle.y + distance * sin(particle.h)
            heading_new = (particle.h + turning_angle) % (2 * pi)

        # cyclic motion
        else:
            radius = distance / turning_angle
            center_x = particle.x - sin(particle.h) * radius
            center_y = particle.y + cos(particle.h) * radius
            x_new = center_x + sin(particle.h + turning_angle) * radius
            y_new = center_y - cos(particle.h + turning_angle) * radius
            heading_new = atan2(y_new - center_y, x_new - center_x) + 0.5 * pi

        # end of code citation

        # initialize a new Satellite object and add it to a list of updated particles
        new_particle = Satellite(x_new, y_new, heading_new, satellite_length, random.uniform(1000, 100000), particle.g_measurement_noise, particle.percent_illuminated_measurement_noise)
        new_particles.append(new_particle)

    # estimate the next state of the target satellite
    total_weight = sum(weights)
    if total_weight > 0:
        x = sum(p.x * w for p, w in zip(new_particles, weights)) / total_weight
        y = sum(p.y * w for p, w in zip(new_particles, weights)) / total_weight
    else:
        x = sum(p.x for p in new_particles) / num_particles
        y = sum(p.y for p in new_particles) / num_particles
    xy_estimate = (x, y)

    # get the last, outermost planet
    home_planet = solar_system.planets[-1]

    # get the angle to send an SOS message this timestep
    bearing = atan2(home_planet.r[1] - y, home_planet.r[0] - x)

    # update the dictionary, particles to plot
    other['particles'] = new_particles
    optional_points_to_plot = [(p.x, p.y, p.h) for p in new_particles]

    return bearing, xy_estimate, other, optional_points_to_plot


def who_am_i():
    # Please specify your GT login ID in the whoami variable (ex: jsmith224).
    whoami = 'wnobles6'
    return whoami
