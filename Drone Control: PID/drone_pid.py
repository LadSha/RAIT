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
    params = [0.1, 0., 0.]

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

    min_hover_error = get_score(hover_error,
    max_allowed_velocity,
    drone_max_velocity,
    max_allowed_oscillations,
    total_oscillations,)

        
    delta_p = 0.001
    # delta_p_decrease = 0.01
    delta_d = 0.1
    change_count = 0
    while change_count<20:
        
        thrust_params_old = thrust_params.copy()
        new_thrust_params = thrust_params.copy()
        new_thrust_params["tau_p"] += delta_p

        (
        hover_error,
        max_allowed_velocity,
        drone_max_velocity,
        max_allowed_oscillations,
        total_oscillations,
        ) = run_callback(new_thrust_params, roll_params, VISUALIZE=VISUALIZE)

        new_hover_error = get_score(hover_error,
        max_allowed_velocity,
        drone_max_velocity,
        max_allowed_oscillations,
        total_oscillations,)

        if min_hover_error>new_hover_error:
            min_hover_error = new_hover_error
            thrust_params = new_thrust_params
            delta_p*=1.2
            change_count = 0
        else:
            new_thrust_params = thrust_params.copy()
            new_thrust_params["tau_p"] -= delta_p
            (
            hover_error,
            max_allowed_velocity,
            drone_max_velocity,
            max_allowed_oscillations,
            total_oscillations,
            ) = run_callback(new_thrust_params, roll_params, VISUALIZE=VISUALIZE)

            new_hover_error = get_score(hover_error,
            max_allowed_velocity,
            drone_max_velocity,
            max_allowed_oscillations,
            total_oscillations,)
            if min_hover_error>new_hover_error:
                min_hover_error = new_hover_error
                thrust_params = new_thrust_params
                delta_p*=1.2
                change_count = 0
            else:
                delta_p*=0.75        
        
        #update D
        new_thrust_params = thrust_params.copy()
        new_thrust_params["tau_d"] += delta_d

        (
        hover_error,
        max_allowed_velocity,
        drone_max_velocity,
        max_allowed_oscillations,
        total_oscillations,
        ) = run_callback(new_thrust_params, roll_params, VISUALIZE=VISUALIZE)

        new_hover_error = get_score(hover_error,
        max_allowed_velocity,
        drone_max_velocity,
        max_allowed_oscillations,
        total_oscillations,)

        if min_hover_error>new_hover_error:
            min_hover_error = new_hover_error
            thrust_params = new_thrust_params
            delta_d*=1.2
            change_count = 0
        else:
            new_thrust_params = thrust_params.copy()
            new_thrust_params["tau_d"] -= delta_d
            (
            hover_error,
            max_allowed_velocity,
            drone_max_velocity,
            max_allowed_oscillations,
            total_oscillations,
            ) = run_callback(new_thrust_params, roll_params, VISUALIZE=VISUALIZE)

            new_hover_error = get_score(hover_error,
            max_allowed_velocity,
            drone_max_velocity,
            max_allowed_oscillations,
            total_oscillations,)
            if min_hover_error>new_hover_error:
                min_hover_error = new_hover_error
                thrust_params = new_thrust_params
                delta_d*=1.2
                change_count = 0
            else:
                delta_d*=0.75

        change = abs(thrust_params_old["tau_p"] - thrust_params["tau_p"]) + abs(thrust_params_old["tau_d"] - thrust_params['tau_d'])+abs(thrust_params_old["tau_i"] - thrust_params["tau_i"])
        if change ==0:
            change_count+=1


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

        # Initialize a list to contain your gain values that you want to tune
    params = [5, 100., 100.]

    # Create dicts to pass the parameters to run_callback
    thrust_params = {"tau_p": params[0], "tau_d": params[1], "tau_i": params[2]}

    # If tuning roll, then also initialize gain values for roll PID controller
    roll_params = {"tau_p": 0., "tau_d": 0., "tau_i": 0}

    # Call run_callback, passing in the dicts of thrust and roll gain values
    
    (
        hover_error,
        max_allowed_velocity,
        drone_max_velocity,
        max_allowed_oscillations,
        total_oscillations,
    ) = run_callback(thrust_params, roll_params, VISUALIZE=VISUALIZE)

    min_hover_error = get_score(hover_error,
    max_allowed_velocity,
    drone_max_velocity,
    max_allowed_oscillations,
    total_oscillations,)

        
    delta_p = 0.01
    # delta_p_decrease = 0.01
    delta_d = 0.1
    delta_i = 0.1
    change_count = 0
    while change_count<20:
        
        thrust_params_old = thrust_params.copy()
        new_thrust_params = thrust_params.copy()
        new_thrust_params["tau_p"] += delta_p

        (
        hover_error,
        max_allowed_velocity,
        drone_max_velocity,
        max_allowed_oscillations,
        total_oscillations,
        ) = run_callback(new_thrust_params, roll_params, VISUALIZE=VISUALIZE)

        new_hover_error = get_score(hover_error,
        max_allowed_velocity,
        drone_max_velocity,
        max_allowed_oscillations,
        total_oscillations,)

        if min_hover_error>new_hover_error:
            min_hover_error = new_hover_error
            thrust_params = new_thrust_params
            delta_p*=1.2
            change_count = 0
        else:
            new_thrust_params = thrust_params.copy()
            new_thrust_params["tau_p"] -= delta_p
            (
            hover_error,
            max_allowed_velocity,
            drone_max_velocity,
            max_allowed_oscillations,
            total_oscillations,
            ) = run_callback(new_thrust_params, roll_params, VISUALIZE=VISUALIZE)

            new_hover_error = get_score(hover_error,
            max_allowed_velocity,
            drone_max_velocity,
            max_allowed_oscillations,
            total_oscillations,)
            if min_hover_error>new_hover_error:
                min_hover_error = new_hover_error
                thrust_params = new_thrust_params
                delta_p*=1.2
                change_count = 0
            else:
                delta_p*=0.75        
        
        #update D
        new_thrust_params = thrust_params.copy()
        new_thrust_params["tau_d"] += delta_d

        (
        hover_error,
        max_allowed_velocity,
        drone_max_velocity,
        max_allowed_oscillations,
        total_oscillations,
        ) = run_callback(new_thrust_params, roll_params, VISUALIZE=VISUALIZE)

        new_hover_error = get_score(hover_error,
        max_allowed_velocity,
        drone_max_velocity,
        max_allowed_oscillations,
        total_oscillations,)

        if min_hover_error>new_hover_error:
            min_hover_error = new_hover_error
            thrust_params = new_thrust_params
            delta_d*=1.2
            change_count = 0
        else:
            new_thrust_params = thrust_params.copy()
            new_thrust_params["tau_d"] -= delta_d
            (
            hover_error,
            max_allowed_velocity,
            drone_max_velocity,
            max_allowed_oscillations,
            total_oscillations,
            ) = run_callback(new_thrust_params, roll_params, VISUALIZE=VISUALIZE)

            new_hover_error = get_score(hover_error,
            max_allowed_velocity,
            drone_max_velocity,
            max_allowed_oscillations,
            total_oscillations,)
            if min_hover_error>new_hover_error:
                min_hover_error = new_hover_error
                thrust_params = new_thrust_params
                delta_d*=1.2
                change_count = 0
            else:
                delta_d*=0.75

#update I
        new_thrust_params = thrust_params.copy()
        new_thrust_params["tau_i"] += delta_i

        (
        hover_error,
        max_allowed_velocity,
        drone_max_velocity,
        max_allowed_oscillations,
        total_oscillations,
        ) = run_callback(new_thrust_params, roll_params, VISUALIZE=VISUALIZE)

        new_hover_error = get_score(hover_error,
        max_allowed_velocity,
        drone_max_velocity,
        max_allowed_oscillations,
        total_oscillations,)

        if min_hover_error>new_hover_error:
            min_hover_error = new_hover_error
            thrust_params = new_thrust_params
            delta_i*=1.2
            change_count = 0
        else:
            new_thrust_params = thrust_params.copy()
            new_thrust_params["tau_i"] -= delta_i
            (
            hover_error,
            max_allowed_velocity,
            drone_max_velocity,
            max_allowed_oscillations,
            total_oscillations,
            ) = run_callback(new_thrust_params, roll_params, VISUALIZE=VISUALIZE)

            new_hover_error = get_score(hover_error,
            max_allowed_velocity,
            drone_max_velocity,
            max_allowed_oscillations,
            total_oscillations,)
            if min_hover_error>new_hover_error:
                min_hover_error = new_hover_error
                thrust_params = new_thrust_params
                delta_i*=1.2
                change_count = 0
            else:
                delta_i*=0.75
        





        change = abs(thrust_params_old["tau_p"] - thrust_params["tau_p"]) + abs(thrust_params_old["tau_d"] - thrust_params['tau_d'])+abs(thrust_params_old["tau_i"] - thrust_params["tau_i"])
        if change ==0:
            change_count+=1


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


        # Initialize a list to contain your gain values that you want to tune
    params = [0.2, 0., 0.]

    # Create dicts to pass the parameters to run_callback
    thrust_params = {"tau_p": params[0], "tau_d": params[1], "tau_i": params[2]}

    # If tuning roll, then also initialize gain values for roll PID controller
    roll_params = {"tau_p": -0.1, "tau_d": -0.2, "tau_i": 0}

    # Call run_callback, passing in the dicts of thrust and roll gain values
    (
        hover_error,
        max_allowed_velocity,
        drone_max_velocity,
        max_allowed_oscillations,
        total_oscillations,
    ) = run_callback(thrust_params, roll_params, VISUALIZE=VISUALIZE)

    min_hover_error = get_score(hover_error,
    max_allowed_velocity,
    drone_max_velocity,
    max_allowed_oscillations,
    total_oscillations,)

        

#Find Thrust
    delta_p = 0.001
    delta_d = 0.1
    change_count = 0
    delta_p_roll = 0.001
    delta_d_roll = 0.1
    while change_count<20:
        
        thrust_params_old = thrust_params.copy()
        new_thrust_params = thrust_params.copy()
        new_thrust_params["tau_p"] += delta_p

        (
        hover_error,
        max_allowed_velocity,
        drone_max_velocity,
        max_allowed_oscillations,
        total_oscillations,
        ) = run_callback(new_thrust_params, roll_params, VISUALIZE=VISUALIZE)

        new_hover_error = get_score(hover_error,
        max_allowed_velocity,
        drone_max_velocity,
        max_allowed_oscillations,
        total_oscillations,)

        if min_hover_error>new_hover_error:
            min_hover_error = new_hover_error
            thrust_params = new_thrust_params
            delta_p*=1.2
            change_count = 0
        else:
            new_thrust_params = thrust_params.copy()
            new_thrust_params["tau_p"] -= delta_p
            (
            hover_error,
            max_allowed_velocity,
            drone_max_velocity,
            max_allowed_oscillations,
            total_oscillations,
            ) = run_callback(new_thrust_params, roll_params, VISUALIZE=VISUALIZE)

            new_hover_error = get_score(hover_error,
            max_allowed_velocity,
            drone_max_velocity,
            max_allowed_oscillations,
            total_oscillations,)
            if min_hover_error>new_hover_error:
                min_hover_error = new_hover_error
                thrust_params = new_thrust_params
                delta_p*=1.2
                change_count = 0
            else:
                delta_p*=0.75
                # delta_p_increase*=0.75
        
        
        #update D
        new_thrust_params = thrust_params.copy()
        new_thrust_params["tau_d"] += delta_d

        (
        hover_error,
        max_allowed_velocity,
        drone_max_velocity,
        max_allowed_oscillations,
        total_oscillations,
        ) = run_callback(new_thrust_params, roll_params, VISUALIZE=VISUALIZE)

        new_hover_error = get_score(hover_error,
        max_allowed_velocity,
        drone_max_velocity,
        max_allowed_oscillations,
        total_oscillations,)

        if min_hover_error>new_hover_error:
            min_hover_error = new_hover_error
            thrust_params = new_thrust_params
            delta_d*=1.2
            change_count = 0
        else:
            new_thrust_params = thrust_params.copy()
            new_thrust_params["tau_d"] -= delta_d
            (
            hover_error,
            max_allowed_velocity,
            drone_max_velocity,
            max_allowed_oscillations,
            total_oscillations,
            ) = run_callback(new_thrust_params, roll_params, VISUALIZE=VISUALIZE)

            new_hover_error = get_score(hover_error,
            max_allowed_velocity,
            drone_max_velocity,
            max_allowed_oscillations,
            total_oscillations,)
            if min_hover_error>new_hover_error:
                min_hover_error = new_hover_error
                thrust_params = new_thrust_params
                delta_d*=1.2
                change_count = 0
            else:
                delta_d*=0.75


    
    #Find P - Roll            
        roll_params_old = roll_params.copy()
        new_roll_params = roll_params.copy()
        new_roll_params["tau_p"] += delta_p_roll

        (
        hover_error,
        max_allowed_velocity,
        drone_max_velocity,
        max_allowed_oscillations,
        total_oscillations,
        ) = run_callback(thrust_params, new_roll_params, VISUALIZE=VISUALIZE)

        new_hover_error = get_score(hover_error,
        max_allowed_velocity,
        drone_max_velocity,
        max_allowed_oscillations,
        total_oscillations,)

        if min_hover_error>new_hover_error:
            min_hover_error = new_hover_error
            roll_params = new_roll_params
            delta_p_roll*=1.2
            change_count = 0
        else:
            new_roll_params = thrust_params.copy()
            new_roll_params["tau_p"] -= delta_p_roll
            (
            hover_error,
            max_allowed_velocity,
            drone_max_velocity,
            max_allowed_oscillations,
            total_oscillations,
            ) = run_callback(thrust_params, new_roll_params, VISUALIZE=VISUALIZE)

            new_hover_error = get_score(hover_error,
            max_allowed_velocity,
            drone_max_velocity,
            max_allowed_oscillations,
            total_oscillations,)
            if min_hover_error>new_hover_error:
                min_hover_error = new_hover_error
                roll_params = new_roll_params
                delta_p_roll*=1.2
                change_count = 0
            else:
                delta_p_roll*=0.75

        
        #update D-roll
        new_roll_params = roll_params.copy()
        new_roll_params["tau_d"] += delta_d_roll

        (
        hover_error,
        max_allowed_velocity,
        drone_max_velocity,
        max_allowed_oscillations,
        total_oscillations,
        ) = run_callback(thrust_params, new_roll_params, VISUALIZE=VISUALIZE)

        new_hover_error = get_score(hover_error,
        max_allowed_velocity,
        drone_max_velocity,
        max_allowed_oscillations,
        total_oscillations,)

        if min_hover_error>new_hover_error:
            min_hover_error = new_hover_error
            roll_params = new_roll_params
            delta_d_roll*=1.2
            change_count = 0
        else:
            new_roll_params = roll_params.copy()
            new_roll_params["tau_d"] -= delta_d_roll
            (
            hover_error,
            max_allowed_velocity,
            drone_max_velocity,
            max_allowed_oscillations,
            total_oscillations,
            ) = run_callback(thrust_params, new_roll_params, VISUALIZE=VISUALIZE)

            new_hover_error = get_score(hover_error,
            max_allowed_velocity,
            drone_max_velocity,
            max_allowed_oscillations,
            total_oscillations,)
            if min_hover_error>new_hover_error:
                min_hover_error = new_hover_error
                roll_params = new_roll_params
                delta_d_roll*=1.2
                change_count = 0
            else:
                delta_d_roll*=0.75



        change = abs(roll_params_old["tau_p"] - roll_params["tau_p"]) + abs(roll_params_old["tau_d"] - roll_params['tau_d'])+abs(roll_params_old["tau_i"] - roll_params["tau_i"]) + abs(thrust_params_old["tau_p"] - thrust_params["tau_p"]) + abs(thrust_params_old["tau_d"] - thrust_params['tau_d'])+abs(thrust_params_old["tau_i"] - thrust_params["tau_i"]) 
        if change ==0:
            change_count+=1


    return thrust_params, roll_params


def who_am_i():
    # Please specify your GT login ID in the whoami variable (ex: jsmith223).
    whoami = "lshamaei3"
    return whoami
