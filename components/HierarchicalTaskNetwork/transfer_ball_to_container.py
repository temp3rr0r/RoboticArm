import pyhop


# Helper methods

def taxi_rate(dist):
    return 1.5 + 0.5 * dist


def center_servos(state):
    return move_arm(state, "center")


def get_diameter(actee):
    if actee == "ball":
        return 3  # cm
    pyhop.failure_reason = "can't get diameter of {}".format(actee)
    return False


def is_within_bounds(location1, min_bounds, max_bounds):
    print(location1, max_bounds, min_bounds)
    for i in range(len(location1)):
        if location1[i] < min_bounds[i] or location1[i] > max_bounds[i]:
            return False
    return True


def get_xyz(actee):
    if actee == "ball":
        arm_xyz = [-16, 12.5, 1.0]
        servo_values = [1500, 1500, 1500, 1500, 1500, 1500]  # TODO: hardcode real servo values
        return arm_xyz, servo_values
    elif actee == "container":
        arm_xyz = [10, 10, 15.0]
        servo_values = [1500, 1500, 1500, 1500, 1500, 1500]  # TODO: hardcode real servo values
        return arm_xyz, servo_values
    elif actee == "center":
        arm_xyz = [0, 0, 1]  # TODO: mock get arm position
        servo_values = [1500, 1500, 1500, 1500, 1500, 1500]
        return arm_xyz, servo_values

    pyhop.failure_reason = "can't get xyz of {}".format(actee)
    return False


def move_arm(state, to_):
    arm_min_bounds = state.min_bounds["xyz"]
    arm_max_bounds = state.max_bounds["xyz"]
    if to_ == "ball":
        arm_xyz, servo_values = get_xyz("ball")
        if not is_within_bounds(arm_xyz, arm_min_bounds, arm_max_bounds):
            pyhop.failure_reason = "can't move arm to {}: {} outside of bounds: {} {}".format(to_, arm_xyz, arm_min_bounds, arm_max_bounds)
            return False
        return arm_xyz, servo_values
    elif to_ == "container":
        arm_xyz, servo_values = get_xyz("container")
        if not is_within_bounds(arm_xyz, arm_min_bounds, arm_max_bounds):
            pyhop.failure_reason = "can't move arm to {}: {} outside of bounds: {} {}".format(to_, arm_xyz, arm_min_bounds, arm_max_bounds)
            return False
        return arm_xyz, servo_values
    elif to_ == "center":
        arm_xyz, servo_values = get_xyz("center")
        if not is_within_bounds(arm_xyz, arm_min_bounds, arm_max_bounds):
            pyhop.failure_reason = "can't move arm to {}: {} outside of bounds: {} {}".format(to_, arm_xyz, arm_min_bounds, arm_max_bounds)
            return False
        return arm_xyz, servo_values
    else:
        pyhop.failure_reason = "can't move arm to {}".format(to_)
        return False


def close_hand(distance):
    # get servo value for cm from distance
    current_hand_servo_value = 1300
    suggested_hand_distance_servo_value = 1000
    distance_ratio = 0.9

    if 500 < suggested_hand_distance_servo_value <= 1500:
        return suggested_hand_distance_servo_value * distance_ratio

    pyhop.failure_reason = "can't close hand to distance {}".format(distance)
    return False


def open_hand(distance):
    # get servo value for cm from distance
    current_hand_servo_value = 1000
    suggested_hand_distance_servo_value = 1300
    distance_ratio = 1.1

    if 500 < suggested_hand_distance_servo_value <= 1500:
        return suggested_hand_distance_servo_value * distance_ratio

    pyhop.failure_reason = "can't open hand to distance {}".format(distance)
    return False


# Operators (primitive tasks)

def initialize(state, actor):
    if actor == "arm":
        arm_xyz, servo_values = center_servos(state)
        if arm_xyz == [0, 0, 1] and servo_values == [1500, 1500, 1500, 1500, 1500, 1500]:
            state.loc['arm_xyz'] = arm_xyz
            state.loc['servo_values'] = servo_values
            return state
    pyhop.failure_reason = "{} can't initialize".format(actor)
    return False


def grab(state, actor, actee, from_):
    if actee == "ball":
        arm_xyz, servo_values = get_xyz(actee)
        if arm_xyz != False:
            state.loc['arm_xyz'] = arm_xyz
            state.loc['servo_values'] = servo_values
            actee_diameter = get_diameter(actee)
            if open_hand(actee_diameter):
                arm_xyz, servo_values = move_arm(state, actee)
                if arm_xyz != False:
                    if close_hand(actee_diameter):
                        state.loc['arm_xyz'] = arm_xyz
                        state.loc['servo_values'] = servo_values
                        return state
    pyhop.failure_reason = "{} can't grab {} from {}".format(actor, actee, from_)
    return False


def put(state, actor, actee, to_):
    if True:
        # Get xyz of container
        # move arm over the container
        # open hand
        return state
    pyhop.failure_reason = "TODO"
    return False


# pyhop.declare_operators(walk, initialize_arm, grab_ball, move_ball_to_container)
pyhop.declare_operators(initialize, grab, put, move_arm, close_hand, open_hand)
print('')
pyhop.print_operators()

# Methods (compound tasks)

# def transfer_to_container(state, a, x, y):
#     if state.cash[a] >= taxi_rate(state.dist[x][y]):
#         return [('initialize_arm', a, x), ('grab_ball', a, x, y), ('move_ball_to_container', a)]
#     return False


def transfer(state, actor, actee, from_, to_):
    if actor == "arm":
        return [('initialize', actor), ('grab', actor, actee, from_), ('put', actor, actee, to_)]

    pyhop.failure_reason = "Unknown actor: " + actor
    return False


pyhop.declare_methods('transfer_ball_to_container', transfer)

print('')
pyhop.print_methods()

current_world_model = pyhop.State('current_world_model')
current_world_model.loc = {'ball': 'table'}  # TODO: check ball not on table -> False
current_world_model.cash = {'ball': 20}
current_world_model.max_bounds = {'xyz': [25, 25, 25]}
current_world_model.min_bounds = {'xyz': [-25, -25, -25]}
current_world_model.weather = {'raining': False}
current_world_model.superhero = {'superman': True}
current_world_model.owe = {'ball': 0}
current_world_model.dist = {'table': {'container': 2}, 'container': {'table': 2}}

htn_plan = pyhop.pyhop(current_world_model, [('transfer_ball_to_container', 'arm', 'ball', 'table', 'container')], verbose=1, all_plans=True)


if not htn_plan:
    print("-- Failure_reason: {}".format(pyhop.failure_reason))
