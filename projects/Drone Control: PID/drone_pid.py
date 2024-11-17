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


def pid_thrust(target_elevation, drone_elevation, tau_p=0, tau_d=0, tau_i=0, data: dict() = {}):
    '''
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
    '''

    # initialize values to store for future calls
    if not data:
        data = {'error': 0, 'error_int': 0, 'max_rpm_reached': False}

    # calculate the cross-track error terms
    error = target_elevation - drone_elevation
    error_integral = error + data['error_int']
    error_derivative = error - data['error']

    # calculate the thrust
    thrust = tau_p * error + tau_i * error_integral + tau_d * error_derivative

    # update the dictionary
    data['error'] = error
    data['error_int'] = error_integral

    return thrust, data


def pid_roll(target_x, drone_x, tau_p=0, tau_d=0, tau_i=0, data:dict() = {}):
    '''
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

    '''

    # initialize values to store for future calls
    if not data:
        data = {'error': 0, 'error_integral': 0}

    # calculate the cross-track error terms
    error = target_x - drone_x
    error_integral = error + data['error_integral']
    error_derivative = error - data['error']

    # calculate the thrust
    roll = -tau_p * error - tau_i * error_integral - tau_d * error_derivative

    # update the dictionary
    data['error'] = error
    data['error_integral'] = error_integral

    return roll, data


def find_parameters_thrust(run_callback, tune='thrust', DEBUG=False, VISUALIZE=False):
    '''
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

    '''

    # initialize lists to contain the gain, delta values to tune
    params = [1, 20]
    dparams = [1, 1]

    # set the tolerance for the twiddle algorithm
    tolerance = 0.001

    # Create dicts to pass the parameters to run_callback
    thrust_params = {'tau_p': params[0], 'tau_d': params[1], 'tau_i': 0}
    roll_params   = {'tau_p': 0, 'tau_d': 0, 'tau_i': 0}

    # Call run_callback, passing in the dicts of thrust and roll gain values
    hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(thrust_params, roll_params, VISUALIZE=VISUALIZE)
    best_error = calculate_combined_error(hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations)

    # use twiddle to tune the params and find the best_error
    # The code below is adapted from:
    # https://gatech.instructure.com/courses/402244/pages/21-parameter-optimization-solution?module_item_id=4089250
    # PID Control: Parameter Optimization (solution)
    while sum(dparams) > tolerance:
        for i, (param, dparam) in enumerate(zip(params, dparams)):
            params[i] += dparam

            # update the thrust params
            thrust_params = {'tau_p': params[0], 'tau_d': params[1], 'tau_i': 0}

            hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(thrust_params, roll_params, VISUALIZE)
            error = calculate_combined_error(hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations)
            if error < best_error:
                best_error = error
                dparams[i] *= 1.1
            else:
                params[i] -= 2 * dparam

                # update the thrust params
                thrust_params = {'tau_p': params[0], 'tau_d': params[1], 'tau_i': 0}

                hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(thrust_params, roll_params, VISUALIZE)
                error = calculate_combined_error(hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations)
                if error < best_error:
                    best_error = error
                    dparams[i] *= 1.1
                else:
                    params[i] += dparam
                    dparams[i] *= 0.9
    # End of code citation

    # update the dictionary with the optimal gain values
    thrust_params = {'tau_p': params[0], 'tau_d': params[1], 'tau_i': 0}

    return thrust_params, roll_params

def find_parameters_with_int(run_callback, tune='thrust', DEBUG=False, VISUALIZE=False):
    '''
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

    '''

    # initialize lists to contain the gain, delta values to tune
    params = [3000, 100000, 0.5]
    dparams = [1, 1, 1]

    # set the tolerance for the twiddle algorithm
    tolerance = 0.001

    # Create dicts to pass the parameters to run_callback
    thrust_params = {'tau_p': params[0], 'tau_d': params[1], 'tau_i': params[2]}
    roll_params   = {'tau_p': 0, 'tau_d': 0, 'tau_i': 0}

    # Call run_callback, passing in the dicts of thrust and roll gain values
    hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(thrust_params, roll_params, VISUALIZE=VISUALIZE)
    best_error = calculate_combined_error(hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations)

    # use twiddle to tune the params and find the best_error
    # The code below is adapted from:
    # https://gatech.instructure.com/courses/402244/pages/21-parameter-optimization-solution?module_item_id=4089250
    # PID Control: Parameter Optimization (solution)
    while sum(dparams) > tolerance:
        for i, (param, dparam) in enumerate(zip(params, dparams)):
            params[i] += dparam

            # update the thrust params
            thrust_params = {'tau_p': params[0], 'tau_d': params[1], 'tau_i': params[2]}

            hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(thrust_params, roll_params, VISUALIZE)
            error = calculate_combined_error(hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations)
            if error < best_error:
                best_error = error
                dparams[i] *= 1.1
            else:
                params[i] -= 2 * dparam

                # update the thrust params
                thrust_params = {'tau_p': params[0], 'tau_d': params[1], 'tau_i': params[2]}

                hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(thrust_params, roll_params, VISUALIZE)
                error = calculate_combined_error(hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations)
                if error < best_error:
                    best_error = error
                    dparams[i] *= 1.1
                else:
                    params[i] += dparam
                    dparams[i] *= 0.9
    # End of code citation

    # update the dictionary with the optimal gain values
    thrust_params = {'tau_p': params[0], 'tau_d': params[1], 'tau_i': params[2]}

    return thrust_params, roll_params

def find_parameters_with_roll(run_callback, tune='both', DEBUG=False, VISUALIZE=False):
    '''
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

    '''
    # initialize lists to contain the gain, delta values to tune
    params = [1, 30, 0.25, 5]
    dparams = [1, 1, 1, 1]

    # set the tolerance
    tolerance = 0.001

    # Create dicts to pass the parameters to run_callback
    thrust_params = {'tau_p': params[0], 'tau_d': params[1], 'tau_i': 0}
    roll_params   = {'tau_p': params[2], 'tau_d': params[3], 'tau_i': 0}

    # Call run_callback, passing in the dicts of thrust and roll gain values
    hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(thrust_params, roll_params, VISUALIZE=VISUALIZE)
    best_error = calculate_combined_error(hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations)

    # use twiddle to tune the params and find the best_error
    # The code below is adapted from:
    # https://gatech.instructure.com/courses/402244/pages/21-parameter-optimization-solution?module_item_id=4089250
    # PID Control: Parameter Optimization (solution)
    while sum(dparams) > tolerance:
        for i, (param, dparam) in enumerate(zip(params, dparams)):
            params[i] += dparam

            # update the thrust params and roll params
            thrust_params = {'tau_p': params[0], 'tau_d': params[1], 'tau_i': 0}
            roll_params = {'tau_p': params[2], 'tau_d': params[3], 'tau_i': 0}

            hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(thrust_params, roll_params, VISUALIZE)
            error = calculate_combined_error(hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations)
            if error < best_error:
                best_error = error
                dparams[i] *= 1.1
            else:
                params[i] -= 2 * dparam

                # update the thrust params and roll params
                thrust_params = {'tau_p': params[0], 'tau_d': params[1], 'tau_i': 0}
                roll_params   = {'tau_p': params[2], 'tau_d': params[3], 'tau_i': 0}

                hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(thrust_params, roll_params, VISUALIZE)
                error = calculate_combined_error(hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations)
                if error < best_error:
                    best_error = error
                    dparams[i] *= 1.1
                else:
                    params[i] += dparam
                    dparams[i] *= 0.9
    # End of code citation

    # update the dictionaries with the optimal gain values
    thrust_params = {'tau_p': params[0], 'tau_d': params[1], 'tau_i': 0}
    roll_params = {'tau_p': params[2], 'tau_d': params[3], 'tau_i': 0}

    return thrust_params, roll_params


def calculate_combined_error(hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations):
    """Calculates an error based on hover error, velocity error, and oscillatory error.

    Args:
        hover_error (float): The difference between the target elevation and the drone's current elevation.
        max_allowed_velocity (float): The maximum velocity allowed for the drone in a particular simulation.
        drone_max_velocity (float): The drone's velocity at a point in its flight.
        max_allowed_oscillations (float): The maximum number of oscillations allowed for the drone in a particular simulation.
        total_oscillations (float): The drone's oscillations at a point in its flight.

    Returns:
        float: A value between 0 and 100 representing an error.
    """

    # the overall error is just the hover error if the drone doesn't exceed any limits imposed
    combined_error = hover_error

    # set penalty weights for exceeding velocity, oscillation limits
    velocity_penalty = 2
    oscillation_penalty = 2

    # get the velocity error, oscillation error
    velocity_error = drone_max_velocity - max_allowed_velocity
    oscillation_error = total_oscillations - max_allowed_oscillations

    # add a weighted error to the combined error if the maximum velocity is exceeded
    if velocity_error > 0:
        combined_error += velocity_error * velocity_penalty

    # add a weighted error to the combined error if the maximum oscillations is exceeded
    if oscillation_error > 0:
        combined_error += oscillation_error * oscillation_penalty

    return combined_error


def who_am_i():
    # Please specify your GT login ID in the whoami variable (ex: jsmith224).
    whoami = 'wnobles6'
    return whoami
