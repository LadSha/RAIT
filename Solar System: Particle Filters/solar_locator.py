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


def get_position(p):
    x = 0.0
    y = 0.0
    orientation = 0.0
    for i in range(len(p)):
        x += p[i][0]
        y += p[i][1]
        orientation += ((p[i][2] - p[0][2] + pi) % (2.0 * pi)) + p[0][2] - pi
    return [(x / len(p)), (y / len(p)), (orientation / len(p))]


def mimic(p, steering, distance, L):
    p3 = []
    for i in range(len(p)):
        theta = p[i][2]
        r = L / tan(steering)
        x_dist = sin(theta) * r
        y_dist = cos(theta) * r
        center_x = p[i][0] - x_dist
        center_y = p[i][1] + y_dist
        beta = distance / r
        x_dist_new = sin(theta + beta) * r
        y_dist_new = cos(theta + beta) * r
        x_new = center_x + x_dist_new
        y_new = center_y - y_dist_new
        theta_new = (theta + beta) % (2 * pi)
        p3.append((x_new, y_new, theta_new))
    return p3


def fuzz(p, steering, distance, L):
    p3 = []
    bound = distance / 100
    for i in range(len(p)):
        dist_fuzz = random.uniform(-1 * bound, bound)
        theta = p[i][2]
        r = L / tan(steering)
        x_dist = sin(theta) * r
        y_dist = cos(theta) * r
        center_x = p[i][0] - x_dist
        center_y = p[i][1] + y_dist
        beta = dist_fuzz / r
        x_dist_new = sin(theta + beta) * r
        y_dist_new = cos(theta + beta) * r
        x_new = center_x + x_dist_new
        y_new = center_y - y_dist_new
        theta_new = (theta + beta) % (2 * pi)
        p3.append((x_new, y_new, theta_new))
    return p3


def estimate_next_pos(
    gravimeter_measurement, gravimeter_sense_func, distance, steering, other=None
):
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
    # create particles
    p = []
    if other == None:
        for i in range(N):
            dist_from_sun = 4 * AU * random.uniform(0, 1) + 1
            angle = random.uniform(0, 2 * math.pi)
            sat_init_x = dist_from_sun * math.cos(angle)
            sat_init_y = dist_from_sun * math.sin(angle)

            p.append((sat_init_x, sat_init_y))
    else:
        p = other
    # weight update
    # sigma = 9.099203572901015e-06
    sigma = gravimeter_measurement / 2
    w = []

    for i in range(N):
        mu = gravimeter_sense_func(p[i][0], p[i][1])

        curr_weight = (1 / (sigma * sqrt(2 * math.pi))) * exp(
            -0.5 * (((mu - gravimeter_measurement) / sigma) ** 2)
        )

        w.append(curr_weight)

    # # resample
    p2 = []
    index = int(random.random() * N) % N
    beta = 0.0
    mw = max(w)
    w_resample = []
    w_total = 0
    for i in range(N):
        beta += random.random() * 2.0 * mw
        while beta > w[index]:
            beta -= w[index]
            index = (index + 1) % N
        p2.append(p[index])
        w_resample.append(w[index])
        w_total += w[index]
    p = p2

    # # # Mimic
    p3 = []
    for i in range(N):
        r = sqrt(p[i][0] ** 2 + p[i][1] ** 2)
        # arc length = beta * radius
        ang = atan2(p[i][1], p[i][0])
        angle_change = distance / r
        # fuzz
        random_fuzz_angle = angle_change * random.uniform(-0.2, 0.2)
        new_angle = angle_change + random_fuzz_angle + ang
        new_x = r * math.cos(new_angle)
        new_y = r * math.sin(new_angle)
        # p3.append((new_x, new_y, new_angle + math.pi / 2))
        p3.append((new_x, new_y))
    p = p3

    xy_estimate = (
        sum([v[0] for v in p]) / float(len(p)),
        sum([v[1] for v in p]) / float(len(p)),
    )

    other = p.copy()
    return xy_estimate, other, p


# def estimate_next_pos(
#     gravimeter_measurement, gravimeter_sense_func, distance, steering, other=None
# ):
#     """
#     Estimate the next (x,y) position of the satelite.
#     This is the function you will have to write for part A.
#     :param gravimeter_measurement: float
#         A floating point number representing
#         the measured magnitude of the gravitation pull of all the planets
#         felt at the target satellite at that point in time.
#     :param gravimeter_sense_func: Func
#         A function that takes in (x,y) and outputs the magnitude of the gravitation pull of all the planets
#         felt at that (x,y) location at that point in time.
#     :param distance: float
#         The target satellite's motion distance
#     :param steering: float
#         The target satellite's motion steering
#     :param other: any
#         This is initially None, but if you return an OTHER from
#         this function call, it will be passed back to you the next time it is
#         called, so that you can use it to keep track of important information
#         over time. (We suggest you use a dictionary so that you can store as many
#         different named values as you want.)
#     :return:
#         estimate: Tuple[float, float]. The (x,y) estimate of the target satellite at the next timestep
#         other: any. Any additional information you'd like to pass between invocations of this function
#         optional_points_to_plot: List[Tuple[float, float, float]].
#             A list of tuples like (x,y,h) to plot for the visualization
#     """
#     # time.sleep(1)  # uncomment to pause for the specified seconds each timestep

#     # example of how to get the gravity magnitude at a point in the solar system:
#     # particle_gravimeter_measurement = gravimeter_sense_func(-1*AU, 1*AU)

#     N = 1000
#     AU = 1.49597870700e11
#     # create particles
#     p = []
#     if other == None:
#         for i in range(N):
#             dist_from_sun = 4 * AU * random.uniform(0, 1) + 1
#             angle = random.uniform(0, 2 * math.pi)
#             sat_init_x = dist_from_sun * math.cos(angle)
#             sat_init_y = dist_from_sun * math.sin(angle)

#             p.append(
#                 (sat_init_x, sat_init_y, atan2(sat_init_y, sat_init_x) + pi / 2),
#             )

#     else:
#         p = other
#     # weight update
#     # sigma = 2.181367944633899e-6

#     sigma = gravimeter_measurement / 10
#     w = []

#     for i in range(N):
#         mu = gravimeter_sense_func(p[i][0], p[i][1])

#         curr_weight = (1 / (sigma * sqrt(2 * math.pi))) * exp(
#             -0.5 * (((mu - gravimeter_measurement) / sigma) ** 2)
#         )

#         w.append(curr_weight)

#     # # resample
#     p2 = []
#     index = int(random.random() * N) % N
#     beta = 0.0
#     mw = max(w)

#     for i in range(N):
#         beta += random.random() * 2.0 * mw
#         while beta > w[index]:
#             beta -= w[index]
#             index = (index + 1) % N
#         p2.append(p[index])

#     p = p2

#     # fuzz
#     p_fuzz = fuzz(p, steering, distance, L=10.2)
#     p = p_fuzz.copy()

#     # # Mimic
#     p_mimic = mimic(p, steering, distance, L=10.2)
#     p = p_mimic.copy()

#     # # # # Mimic
#     # p3 = []
#     # for i in range(N):
#     #     r = sqrt(p[i][0] ** 2 + p[i][1] ** 2)
#     #     # arc length = beta * radius
#     #     theta = atan2(p[i][1], p[i][0])
#     #     angle_change = distance / r
#     #     # fuzz
#     #     random_fuzz_angle = angle_change * random.uniform(-0.5, 0.5)
#     #     new_angle = angle_change + theta + random_fuzz_angle
#     #     new_x = r * math.cos(new_angle)
#     #     new_y = r * math.sin(new_angle)

#     #     p3.append((new_x, new_y))
#     # p = p3

#     xy_estimate = (
#         sum([v[0] for v in p]) / float(len(p)),
#         sum([v[1] for v in p]) / float(len(p)),
#     )

#     other = p.copy()
#     return xy_estimate, other, p


def next_angle(
    solar_system,
    percent_illuminated_measurements,
    percent_illuminated_sense_func,
    distance,
    steering,
    other=None,
):
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

    N = 2000
    AU = 1.49597870700e11
    # create particles
    p = []

    if other == None:
        for i in range(N):
            dist_from_sun = 4 * AU * random.uniform(0, 1) + 1
            angle = random.uniform(0, 2 * math.pi)
            sat_init_x = dist_from_sun * math.cos(angle)
            sat_init_y = dist_from_sun * math.sin(angle)

            p.append((sat_init_x, sat_init_y, atan2(sat_init_y, sat_init_x) + pi / 2))
    else:
        p = other
    # weight update

    w = []

    bearing_noise = 0.9
    for i in range(N):
        mu = percent_illuminated_sense_func(p[i][0], p[i][1])

        error = 1.0
        for j in range(len(percent_illuminated_measurements)):
            error_bearing = abs(percent_illuminated_measurements[j] - mu[j])
            error_bearing = (error_bearing + pi) % (2.0 * pi) - pi

            error *= exp((-(error_bearing**2) / (bearing_noise**2)) / 2.0) / sqrt(
                2.0 * pi * (bearing_noise**2)
            )
        w.append(error)
    # # resample
    p2 = []
    index = int(random.random() * N) % N
    beta = 0.0
    mw = max(w)

    for i in range(N):
        beta += random.random() * 2.0 * mw
        while beta > w[index]:
            beta -= w[index]
            index = (index + 1) % N
        p2.append(p[index])

    p = p2
    # fuzz
    p_fuzz = fuzz(p, steering, distance, L=10.2)
    p = p_fuzz.copy()

    # # Mimic
    p_mimic = mimic(p, steering, distance, L=10.2)
    p = p_mimic.copy()

    x_estimate, y_esimate, theta_estimate = get_position(p)

    other = p.copy()

    solar_system.move_planets()
    home_planet = solar_system.planets[-1].r
    home_planet_x = home_planet[0]
    home_planet_y = home_planet[1]
    delta_x = x_estimate - home_planet_x
    delta_y = y_esimate - home_planet_y

    abs_angle = atan2(delta_y, delta_x)
    # if delta_x * delta_y < 0:
    #     bearing = abs_angle * -1 + pi / 2

    # else:
    #     bearing = abs_angle

    return (
        abs_angle,
        (x_estimate, y_esimate),
        other,
        p,
    )


def who_am_i():
    # Please specify your GT login ID in the whoami variable (ex: jsmith223).
    whoami = "lshamaei3"
    return whoami


if __name__ == "__main__":
    print(mimic([(0.118, -0.54, 0.1)], 0.166, 1.07, 0.2))
