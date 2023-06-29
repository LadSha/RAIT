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


def get_score(
    hover_error,
    max_allowed_velocity,
    drone_max_velocity,
    max_allowed_oscillations,
    total_oscillations,
):
    vel_score = 1

    if max_allowed_velocity != 0 and drone_max_velocity > max_allowed_velocity:
        vel_score = max_allowed_velocity / drone_max_velocity
        vel_score = min(vel_score, 1)

    osc_score = 1

    if max_allowed_oscillations != -1 and total_oscillations > max_allowed_oscillations:
        osc_score = max_allowed_oscillations / total_oscillations
        osc_score = min(osc_score, 1)

    min_hover_error = hover_error / vel_score / osc_score

    return min_hover_error


def pid_thrust(
    target_elevation, drone_elevation, tau_p=0, tau_d=0, tau_i=0, data: dict() = {}
):
    """
    Student code for Thrust PID control. Drone's starting x, y position is (0, 0).

    Args:
    target_elevation: The target elevation that the drone has to achieve
    drone_elevation: The drone's elevation at the current time step
    tau_p: Proportional gain
    tau_i: Integral gain
    tau_d: Differential gain
    data: Dictionary that you can use to pass values across calls.
        Reserved keys:
            max_rpm_reached: (True|False) - Whether Drone has reached max RPM in both its rotors.

    Returns:
        Tuple of thrust, data
        thrust - The calculated change in thrust using PID controller
        data - A dictionary containing any values you want to pass to the next
            iteration of this function call.
            Reserved keys:
                max_rpm_reached: (True|False) - Whether Drone has reached max RPM in both its rotors.
    """

    if "diff" not in data:
        data["diff"] = [0]

    error = target_elevation - drone_elevation
    error_derivative = error - data["diff"][-1]
    thrust = tau_d * error_derivative + tau_p * error

    data["diff"].append(error)
    return thrust, data


def pid_roll(target_x, drone_x, tau_p=0, tau_d=0, tau_i=0, data: dict() = {}):
    """
    Student code for PD control for roll. Drone's starting x,y position is 0, 0.

    Args:
    target_x: The target horizontal displacement that the drone has to achieve
    drone_x: The drone's x position at this time step
    tau_p: Proportional gain, supplied by the test suite
    tau_i: Integral gain, supplied by the test suite
    tau_d: Differential gain, supplied by the test suite
    data: Dictionary that you can use to pass values across calls.

    Returns:
        Tuple of roll, data
        roll - The calculated change in roll using PID controller
        data - A dictionary containing any values you want to pass to the next
            iteration of this function call.

    """

    if "diff" not in data:
        data["diff"] = [0]

    error = target_x - drone_x
    error_derivative = error - data["diff"][-1]
    roll = tau_d * error_derivative + tau_p * error

    data["diff"].append(error)
    return roll, data


def find_parameters_thrust(run_callback, tune="thrust", DEBUG=False, VISUALIZE=False):
    """
    Student implementation of twiddle algorithm will go here. Here you can focus on
    tuning gain values for Thrust test cases only.

    Args:
    run_callback: A handle to DroneSimulator.run() method. You should call it with your
                PID gain values that you want to test with. It returns an error value that indicates
                how well your PID gain values followed the specified path.

    tune: This will be passed by the test harness.
            A value of 'thrust' means you only need to tune gain values for thrust.
            A value of 'both' means you need to tune gain values for both thrust and roll.

    DEBUG: Whether or not to output debugging statements during twiddle runs
    VISUALIZE: Whether or not to output visualizations during twiddle runs

    Returns:
        tuple of the thrust_params, roll_params:
            thrust_params: A dict of gain values for the thrust PID controller
              thrust_params = {'tau_p': 0.0, 'tau_d': 0.0, 'tau_i': 0.0}

            roll_params: A dict of gain values for the roll PID controller
              roll_params   = {'tau_p': 0.0, 'tau_d': 0.0, 'tau_i': 0.0}

    """

    # Initialize a list to contain your gain values that you want to tune
    params = [0.1, 0.1, 0.1]

    # Create dicts to pass the parameters to run_callback
    thrust_params = {"tau_p": params[0], "tau_d": params[1], "tau_i": params[2]}

    # If tuning roll, then also initialize gain values for roll PID controller
    roll_params = {"tau_p": 0, "tau_d": 0, "tau_i": 0}

    # Call run_callback, passing in the dicts of thrust and roll gain values
    # (
    #     hover_error,
    #     max_allowed_velocity,
    #     drone_max_velocity,
    #     max_allowed_oscillations,
    #     total_oscillations,
    # ) = run_callback(thrust_params, roll_params, VISUALIZE=VISUALIZE)

    # Calculate best_error from above returned values
    best_error = None

    delta_p = 0.1
    delta_d = 0.1
    delta_i = 0.1

    

    # min_hover_error = get_score(
    #     hover_error,
    #     max_allowed_velocity,
    #     drone_max_velocity,
    #     max_allowed_oscillations,
    #     total_oscillations,
    # )

    tol = 0.1
    min_hover_error = 1000
    prev_tau_p=1000
    while prev_tau_p-thrust_params["tau_p"] > 0.04:
        (
        hover_error,
        max_allowed_velocity,
        drone_max_velocity,
        max_allowed_oscillations,
        total_oscillations,
        ) = run_callback(thrust_params, roll_params, VISUALIZE=VISUALIZE)
        delta_p=delta_p*.9
        prev_tau_p = thrust_params["tau_p"]
        if drone_max_velocity>=max_allowed_velocity or hover_error<0:
            thrust_params["tau_p"]-=delta_p
        else:
            thrust_params["tau_p"]+=delta_p
        min_hover_error = hover_error


        # if drone_max_velocity>max_allowed_velocity:
        #     new_tau_p = thrust_params["tau_p"] - 0.9*delta_p*max_allowed_velocity/drone_max_velocity

        # if max_allowed_velocity<=drone_max_velocity:
        #     new_tau_p = thrust_params["tau_p"]-delta_p/2
        # new_tau_p = delta_p + thrust_params["tau_p"]

        # (
        #     hover_error,
        #     max_allowed_velocity,
        #     drone_max_velocity,
        #     max_allowed_oscillations,
        #     total_oscillations,
        # ) = run_callback(thrust_params, roll_params, VISUALIZE=VISUALIZE)

        # if drone_max_velocity>max_allowed_velocity:
        #     thrust_params["tau_p"] = thrust_params["tau_p"] - 0.9*delta_p*max_allowed_velocity/drone_max_velocity

        # curr_hover_error = get_score(
        #     hover_error,
        #     max_allowed_velocity,
        #     drone_max_velocity,
        #     max_allowed_oscillations,
        #     total_oscillations,
        # )

        # if curr_hover_error < min_hover_error:
        #     min_hover_error = curr_hover_error

        #     delta_p += 1.01
        # else:
        #     delta_p = delta_p * -1
        #     thrust_params["tau_p"] = 2 * delta_p + thrust_params["tau_p"]
        #     (
        #         hover_error,
        #         max_allowed_velocity,
        #         drone_max_velocity,
        #         max_allowed_oscillations,
        #         total_oscillations,
        #     ) = run_callback(thrust_params, roll_params, VISUALIZE=VISUALIZE)
        #     new_hover_error = get_score(
        #         hover_error,
        #         max_allowed_velocity,
        #         drone_max_velocity,
        #         max_allowed_oscillations,
        #         total_oscillations,
        #     )
        #     if new_hover_error < min_hover_error:
        #         min_hover_error = new_hover_error
        #         delta_p *= 1.01
        #     else:
        #         thrust_params["tau_p"] = -2 * delta_p + thrust_params["tau_p"]
        #         delta_p *= 0.75

        # print(thrust_params, hover_error)
        # Update derivative
        # thrust_params["tau_d"] = delta_d + thrust_params["tau_d"]

        # (
        #     hover_error,
        #     max_allowed_velocity,
        #     drone_max_velocity,
        #     max_allowed_oscillations,
        #     total_oscillations,
        # ) = run_callback(thrust_params, roll_params, VISUALIZE=VISUALIZE)

        # curr_hover_error = get_score(
        #     hover_error,
        #     max_allowed_velocity,
        #     drone_max_velocity,
        #     max_allowed_oscillations,
        #     total_oscillations,
        # )

        # if curr_hover_error < min_hover_error:
        #     min_hover_error = curr_hover_error
        #     delta_d *= 1.25
        # else:
        #     delta_d = delta_d * -1
        #     thrust_params["tau_d"] = 2 * delta_d + thrust_params["tau_d"]
        #     (
        #         hover_error,
        #         max_allowed_velocity,
        #         drone_max_velocity,
        #         max_allowed_oscillations,
        #         total_oscillations,
        #     ) = run_callback(thrust_params, roll_params, VISUALIZE=VISUALIZE)
        #     new_hover_error = get_score(
        #         hover_error,
        #         max_allowed_velocity,
        #         drone_max_velocity,
        #         max_allowed_oscillations,
        #         total_oscillations,
        #     )
        #     if new_hover_error < min_hover_error:
        #         min_hover_error = new_hover_error
        #         delta_d *= 1.25
        #     else:
        #         thrust_params["tau_d"] = -2 * delta_d + thrust_params["tau_d"]
        #         delta_d *= 0.75
        # # Update integral
        # thrust_params["tau_i"] = delta_i + thrust_params["tau_i"]

        # (
        #     hover_error,
        #     max_allowed_velocity,
        #     drone_max_velocity,
        #     max_allowed_oscillations,
        #     total_oscillations,
        # ) = run_callback(thrust_params, roll_params, VISUALIZE=VISUALIZE)

        # curr_hover_error = get_score(
        #     hover_error,
        #     max_allowed_velocity,
        #     drone_max_velocity,
        #     max_allowed_oscillations,
        #     total_oscillations,
        # )

        # if curr_hover_error < min_hover_error:
        #     min_hover_error = curr_hover_error
        #     delta_i *= 1.25
        # else:
        #     delta_i = delta_i * -1
        #     thrust_params["tau_i"] = 2 * delta_i + thrust_params["tau_i"]
        #     (
        #         hover_error,
        #         max_allowed_velocity,
        #         drone_max_velocity,
        #         max_allowed_oscillations,
        #         total_oscillations,
        #     ) = run_callback(thrust_params, roll_params, VISUALIZE=VISUALIZE)
        #     new_hover_error = get_score(
        #         hover_error,
        #         max_allowed_velocity,
        #         drone_max_velocity,
        #         max_allowed_oscillations,
        #         total_oscillations,
        #     )
        #     if new_hover_error < min_hover_error:
        #         min_hover_error = new_hover_error
        #         delta_i *= 1.25
        #     else:
        #         thrust_params["tau_i"] = -2 * delta_i + thrust_params["tau_i"]
        #         delta_i *= 0.75

    # Implement your code to use twiddle to tune the params and find the best_error

    # Return the dict of gain values that give the best error.

    return thrust_params, roll_params


def find_parameters_with_int(run_callback, tune="thrust", DEBUG=False, VISUALIZE=False):
    """
    Student implementation of twiddle algorithm will go here. Here you can focus on
    tuning gain values for Thrust test case with Integral error

    Args:
    run_callback: A handle to DroneSimulator.run() method. You should call it with your
                PID gain values that you want to test with. It returns an error value that indicates
                how well your PID gain values followed the specified path.

    tune: This will be passed by the test harness.
            A value of 'thrust' means you only need to tune gain values for thrust.
            A value of 'both' means you need to tune gain values for both thrust and roll.

    DEBUG: Whether or not to output debugging statements during twiddle runs
    VISUALIZE: Whether or not to output visualizations during twiddle runs

    Returns:
        tuple of the thrust_params, roll_params:
            thrust_params: A dict of gain values for the thrust PID controller
              thrust_params = {'tau_p': 0.0, 'tau_d': 0.0, 'tau_i': 0.0}

            roll_params: A dict of gain values for the roll PID controller
              roll_params   = {'tau_p': 0.0, 'tau_d': 0.0, 'tau_i': 0.0}

    """

    # Initialize a list to contain your gain values that you want to tune, e.g.,
    params = [0, 0, 0]

    # Create dicts to pass the parameters to run_callback
    thrust_params = {"tau_p": params[0], "tau_d": params[1], "tau_i": params[2]}

    # If tuning roll, then also initialize gain values for roll PID controller
    roll_params = {"tau_p": 0, "tau_d": 0, "tau_i": 0}

    # Call run_callback, passing in the dicts of thrust and roll gain values
    (
        hover_error,
        max_allowed_velocity,
        drone_max_velocity,
        max_allowed_oscillations,
        total_oscillations,
    ) = run_callback(thrust_params, roll_params, VISUALIZE=VISUALIZE)

    vel_score = 1

    if max_allowed_velocity != 0 and drone_max_velocity > max_allowed_velocity:
        vel_score = max_allowed_velocity / drone_max_velocity
        vel_score = min(vel_score, 1)

    osc_score = 1

    if max_allowed_oscillations != -1 and total_oscillations > max_allowed_oscillations:
        osc_score = max_allowed_oscillations / total_oscillations
        osc_score = min(osc_score, 1)

    min_error = hover_error / vel_score / osc_score

    # Calculate best_error from above returned values
    best_error = None

    # Implement your code to use twiddle to tune the params and find the best_error

    # Return the dict of gain values that give the best error.

    return thrust_params, roll_params


def find_parameters_with_roll(run_callback, tune="both", DEBUG=False, VISUALIZE=False):
    """
    Student implementation of twiddle algorithm will go here. Here you will
    find gain values for Thrust as well as Roll PID controllers.

    Args:
    run_callback: A handle to DroneSimulator.run() method. You should call it with your
                PID gain values that you want to test with. It returns an error value that indicates
                how well your PID gain values followed the specified path.

    tune: This will be passed by the test harness.
            A value of 'thrust' means you only need to tune gain values for thrust.
            A value of 'both' means you need to tune gain values for both thrust and roll.

    DEBUG: Whether or not to output debugging statements during twiddle runs
    VISUALIZE: Whether or not to output visualizations during twiddle runs

    Returns:
        tuple of the thrust_params, roll_params:
            thrust_params: A dict of gain values for the thrust PID controller
              thrust_params = {'tau_p': 0.0, 'tau_d': 0.0, 'tau_i': 0.0}

            roll_params: A dict of gain values for the roll PID controller
              roll_params   = {'tau_p': 0.0, 'tau_d': 0.0, 'tau_i': 0.0}

    """
    # Initialize a list to contain your gain values that you want to tune, e.g.,
    params = [0, 0, 0]

    # Create dicts to pass the parameters to run_callback
    thrust_params = {"tau_p": params[0], "tau_d": params[1], "tau_i": params[2]}

    # If tuning roll, then also initialize gain values for roll PID controller
    roll_params = {"tau_p": 0, "tau_d": 0, "tau_i": 0}

    # Call run_callback, passing in the dicts of thrust and roll gain values
    (
        hover_error,
        max_allowed_velocity,
        drone_max_velocity,
        max_allowed_oscillations,
        total_oscillations,
    ) = run_callback(thrust_params, roll_params, VISUALIZE=VISUALIZE)

    # Calculate best_error from above returned values
    best_error = None

    # Implement your code to use twiddle to tune the params and find the best_error

    # Return the dict of gain values that give the best error.

    return thrust_params, roll_params


def who_am_i():
    # Please specify your GT login ID in the whoami variable (ex: jsmith223).
    whoami = "lshamaei3"
    return whoami
