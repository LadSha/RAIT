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

import numpy as np
import math
from body import *
from solar_system import *
from satellite import *

def move(distance,steering, x, y, orientation):
    # You can replace the INSIDE of this function with the move function you modified in the module quiz
    # Note that you will need to handle motion noise inside your function accordingly


    length = 10.2
    bearing_noise = 0
    steering_noise = 0
    distance_noise = 0
    steering2 = random.gauss(steering, steering_noise)
    distance2 = random.gauss(distance, distance_noise)

    turn = tan(steering2) * distance2 / length

    radius = distance2 / turn
    cx = x - (sin(orientation) * radius)
    cy = y + (cos(orientation) * radius)
    orientation = (orientation + turn) % (2. * pi)
    x = cx + (sin(orientation) * radius)
    y = cy - (cos(orientation) * radius)

    return ([x, y, orientation])


def estimate_next_pos(gravimeter_measurement, gravimeter_sense_func, distance, steering, other=None):
    """
    Estimate the next (x,y) position of the satelite.
    This is the function you will have to write for part A.
    :param gravimeter_measurement: float
        A floating point number representing
        the measured magnitude of the gravitation pull of all the planets
        felt at the target satellite at that point in time.
    :param gravimeter_sense_func: Func
        A function that takes in (x,y) and outputs the magnitude of the gravitation pull of all the planets
        felt at that (x,y) location at that point in time.
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

    # example of how to get the gravity magnitude at a point in the solar system:
    # particle_gravimeter_measurement = gravimeter_sense_func(-1*AU, 1*AU)

    N = 1000
    AU = 1.49597870700e11
    p = []
    for i in range(N):
        dist_from_sun = random.uniform(-4*AU , 4*AU)
        angle = random.uniform(0, 2 * math.pi)
        sat_init_x = dist_from_sun * math.cos(angle)
        sat_init_y = dist_from_sun * math.sin(angle)
        p.append([sat_init_x, sat_init_y, angle])
    p2 = []
    for i in range(N):
        p2.append(move(distance,steering, p[i][0], p[i][1], p[i][2]))
    p = p2

    w = []
    bearing_noise = .08
    for i in range(N):
        error_bearing = abs(gravimeter_measurement - gravimeter_sense_func(p[i][0],p[i][1]))
        error_bearing = (error_bearing + pi) % (2.0 * pi) - pi

        error = ((exp((- (error_bearing ** 2) / (bearing_noise ** 2)) / 2.0) /
                   sqrt(2.0 * pi * (bearing_noise ** 2))))
        w.append(error)


    p3 = []
    index = int(random.random() * N) % N
    beta = 0.0
    mw = max(w)
    for i in range(N):
        beta += random.random() * 2.0 * mw
        while beta > w[index]:
            beta -= w[index]
            index = (index + 1) % N
        p3.append(p[index])
    p = p3



    # pass one test case once you start to write your solution....
    xy_estimate = (129744950838.85445, 36955392255.3185)

    # You may optionally also return a list of (x,y,h) points that you would like
    # the PLOT_PARTICLES=True visualizer to plot for visualization purposes.
    # If you include an optional third value, it will be plotted as the heading
    # of your particle.
    optional_points_to_plot = [(1*AU, 1*AU), (2*AU, 2*AU), (3*AU, 3*AU)]  # Sample (x,y) to plot
    optional_points_to_plot = [(1*AU, 1*AU, 0.5), (2*AU, 2*AU, 1.8), (3*AU, 3*AU, 3.2)]  # (x,y,heading)

    return xy_estimate, other, optional_points_to_plot


def next_angle(solar_system, percent_illuminated_measurements, percent_illuminated_sense_func,
               distance, steering, other=None):
    """
    Gets the next angle at which to send out an sos message to the home planet,
    the last planet in the solar system.
    This is the function you will have to write for part B.
    The input parameters are exactly the same as for part A.
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

    # At what angle to send an SOS message this timestep
    bearing = 0.0
    xy_estimate = (110172640485.32968, -66967324464.19617)

    # You may optionally also return a list of (x,y) or (x,y,h) points that
    # you would like the PLOT_PARTICLES=True visualizer to plot.
    optional_points_to_plot = [ (1*AU,1*AU), (2*AU,2*AU), (3*AU,3*AU) ]  # Sample plot points

    return bearing, xy_estimate, other, optional_points_to_plot


def who_am_i():
    # Please specify your GT login ID in the whoami variable (ex: jsmith223).
    whoami = 'lshamaei3'
    return whoami
