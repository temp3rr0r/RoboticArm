import pyhop


# Helper methods

def taxi_rate(dist):
    return 1.5 + 0.5 * dist


def center_servos():
    arm_xyz = 0, 0, 1  # TODO: mock get arm position
    servo_values = [1500, 1500, 1500, 1500, 1500, 1500]
    return arm_xyz, servo_values

# Operators (primitive tasks)

def walk(state, a, x, y):
    if state.loc[a] == x:
        state.loc[a] = y
        return state
    else:
        return False


def initialize_arm(state, a, x):
    state.loc['taxi'] = x
    return state


def grab_ball(state, a, x, y):
    if state.loc['taxi'] == x and state.loc[a] == x:
        state.loc['taxi'] = y
        state.loc[a] = y
        state.owe[a] = taxi_rate(state.dist[x][y])
        return state
    else:
        return False


def move_ball_to_container(state, a):
    if state.cash[a] >= state.owe[a]:
        state.cash[a] = state.cash[a] - state.owe[a]
        state.owe[a] = 0
        return state
    else:
        return False

def initialize(state, actor):
    arm_xyz, servo_values = center_servos()
    if arm_xyz == (0, 0, 1) and servo_values == [1500, 1500, 1500, 1500, 1500, 1500]:
        state.loc['arm_xyz'] = arm_xyz
        state.loc['servo_values'] = servo_values
        return state
    else:
        return False


def grab(state, actor, actee, from_):
    if True:
        # do stuff
        return state
    return False


def put(state, actor, actee, to_):
    if True:
        # do stuff
        return state
    return False


# pyhop.declare_operators(walk, initialize_arm, grab_ball, move_ball_to_container)
pyhop.declare_operators(walk, initialize_arm, grab_ball, move_ball_to_container, initialize, grab, put)
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
    return False


pyhop.declare_methods('transfer_ball_to_container', transfer)

print('')
pyhop.print_methods()

current_world_model = pyhop.State('current_world_model')
current_world_model.loc = {'ball': 'table'}  # TODO: check ball not on table
current_world_model.cash = {'ball': 20}
current_world_model.weather = {'raining': False}
current_world_model.superhero = {'superman': True}
current_world_model.owe = {'ball': 0}
current_world_model.dist = {'table': {'container': 2}, 'container': {'table': 2}}

pyhop.pyhop(current_world_model, [('transfer_ball_to_container', 'arm', 'ball', 'table', 'container')], verbose=1, all_plans=True)