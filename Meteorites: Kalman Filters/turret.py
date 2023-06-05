import math

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

# Optional: You may use deepcopy to help prevent aliasing
# from copy import deepcopy

# You may use either the numpy library or Sebastian Thrun's matrix library for
# your matrix math in this project; uncomment the import statement below for
# the library you wish to use, and ensure that the library you are not using is
# commented out.
from matrix import matrix
from math import atan2
# If you see different scores locally and on Gradescope this may be an
# indication that you are uploading a different file than the one you are
# executing locally. If this local ID doesn't match the ID on Gradescope then
# you uploaded a different file.
OUTPUT_UNIQUE_FILE_ID = False
if OUTPUT_UNIQUE_FILE_ID:
    import hashlib, pathlib
    file_hash = hashlib.md5(pathlib.Path(__file__).read_bytes()).hexdigest()
    print(f'Unique file ID: {file_hash}')


class Turret(object):
    """The laser used to defend against invading Meteorites."""

    def __init__(self, init_pos, max_angle_change,
                 dt):
        """Initialize the Turret."""
        self.x_pos = init_pos['x']
        self.y_pos = init_pos['y']
        self.max_angle_change = max_angle_change
        self.dt = dt
        self.history = []

        self.pred = ()
        self.target_ID = None
    def predict_from_observations(self, meteorite_observations):
        """Observe meteorite locations and predict their positions at time t+1.

        Parameters
        ----------
        self = a reference to the current object, the Turret
        meteorite_observations = a list of noisy observations of
            meteorite locations, taken at time t

        Returns
        -------
        A tuple or list of tuples containing (i, x, y), where i, x, and y are:
        i = the meteorite's ID
        x = the estimated x-coordinate of meteorite i's position for time t+1
        y = the estimated y-coordinate of meteorite i's position for time t+1

        Return format hint:
        For a tuple of tuples, this would look something like
        ((1, 0.4, 0.381), (2, 0.77, 0.457), ...)
        For a list of tuples, this would look something like
        [(1, 0.4, 0.381), (2, 0.77, 0.457), ...]

        Notes
        -----
        Each observation in meteorite_observations is a tuple
        (i, x, y), where i is the unique ID for a meteorite, and x, y are the
        x, y locations (with noise) of the current observation of that
        meteorite at this timestep. Only meteorites that are currently
        'in-bounds' will appear in this list, so be sure to use the meteorite
        ID, and not the position/index within the list to identify specific
        meteorites.
        The list/tuple of tuples you return may change in size as meteorites
        move in and out of bounds.
        """
        # TODO: Update the Turret's estimate of where the meteorites are
        # located at the current timestep and return the updated estimates

        # initial state estimate

        self.pred=()

        for (i,x_0,y_0) in meteorite_observations:

            x =  matrix([[x_0],[y_0],[1.],[1],[1]])

            s_acc = 1/3
            #The code is taken from course lectures https://gatech.instructure.com/courses/325394/pages/27-kalman-matrices-answer?module_item_id=3202342

            P =matrix([[6,0,0,0,0], [0., 6,0,0,0], [0,0.,6,0.,0], [0,0., 0,6,0],[0,0.,0,0,6]])
            H = matrix([[1, 0, 0, 0, 0], [0, 1, 0, 0,0]])
            R = matrix([[0.1,0], [0, 0.1]])
            I = matrix([[1., 0., 0., 0., 0], [0., 1., 0., 0., 0], [0., 0., 1., 0., 0], [0., 0., 0., 1., 0],
                        [0., 0., 0, 0, 1.]])
            Q = matrix([[0, 0, 0, 0, 0],
                      [0., 0., 0, 0, 0],
                      [0., 0., 0., 0, 0], [0., 0., 0, 0., 0], [0., 0., 0, 0, 0.]])


            previously_seen_idx = None
            F = matrix([[1, 0, self.dt, 0, s_acc * 0.5 * self.dt ** 2], [0, 1, 0, self.dt, 0.5 * self.dt ** 2], [0, 0, 1, 0, self.dt * s_acc],
                        [0, 0, 0, 1, self.dt], [0, 0, 0, 0, 1]])

            for idx, met in enumerate(self.history):
                if i==met[0]:
                    x=met[1][0]

                    P=met[2][0]
                    previously_seen_idx=idx
                    break
            #observe
            z = matrix([[x_0],[y_0]])

            y = z - (H*x)

            S = H * P * H.transpose() + R

            K = P * H.transpose()*S.inverse() # kalman gain
            x = x + (K*y)
            P = (I - (K*H)) * P

            #predict
            x = (F*x)

            P = F * P * F.transpose() + Q   # predicted covariance
            if previously_seen_idx!=None:
                self.history[idx] = [i,[x],[P]]
            else:
                self.history.append([i,[x], [P]])

            curr_x = x[0][0]
            curr_y = x[1][0]
            curr = (i,curr_x, curr_y),
            self.pred = self.pred+ curr

        return self.pred

    def get_laser_action(self, current_aim_rad):
        """Return the laser's action; it can change its aim angle and/or fire.

        Parameters
        ----------
        self = a reference to the current object, the Turret
        current_aim_rad = the laser turret's current aim angle, in radians,
            provided by the simulation.


        Returns
        -------
        Float (desired change in laser aim angle, in radians)
        Bool (True if the laser should fire next timestep, False otherwise)
        Note that the float should be returned first, and the bool second

        Notes
        -----
        The laser can aim in the range [0.0, pi].

        The maximum amount the laser's aim angle can change in a given timestep
        is self.max_angle_change radians. Larger change angles will be
        clamped to self.max_angle_change, but will keep the same sign as the
        returned desired angle change (e.g. an angle change of -3.0 rad would
        be clamped to -self.max_angle_change).

        If the laser is aimed at 0.0 rad, it will point horizontally to the
        right; if it is aimed at pi rad, it will point to the left.

        If bool returned from this function is True, the laser will fire.
        """

        sorted_by_y = list(self.pred)
        target_idx = -1000

        for i in range(0,len(sorted_by_y)):
            if self.target_ID == sorted_by_y[i][0]:
                target_idx = i

        if target_idx==-1000:
            sorted_by_y = sorted(sorted_by_y, key=lambda x: x[2])
            for i in range(0, len(sorted_by_y)):
                if sorted_by_y[i][0] !=-1:
                    target_idx = i
                    self.target_ID = sorted_by_y[i][0]
                    break

        if sorted_by_y[target_idx][2] <0.45:
            angle = math.degrees(math.atan((1+sorted_by_y[target_idx][2])/sorted_by_y[target_idx][1]))
            current_aim_degree = math.degrees(current_aim_rad)
            #The angle it returns is 180 degrees off, so adjusted it here
            if angle<0:
                angle=angle+180
            if current_aim_degree-.85<angle<.85+current_aim_degree:
                return (0, True)
            else:
                return (math.radians(angle - current_aim_degree), False)

        return (0, False)

def who_am_i():
    # Please specify your GT login ID in the whoami variable (ex: jsmith223).
    whoami = 'lshamaei3'
    return whoami


