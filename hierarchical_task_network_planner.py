import pyhop


class HierarchicalTaskNetworkPlanner:

    def __init__(self):

        # Helper methods
        def center_servos(state):
            return move_arm(state, "center")

        def get_diameter(actee):
            if actee == "ball":
                return 3  # cm
            pyhop.failure_reason = "can't get diameter of {}".format(actee)
            return False

        def is_within_bounds(location1, min_bounds, max_bounds):
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
            return False, False

        # Operators (primitive tasks)

        def move_arm(state, to_):
            arm_min_bounds = state.min_bounds["xyz"]
            arm_max_bounds = state.max_bounds["xyz"]
            if to_ == "ball":
                arm_xyz, servo_values = get_xyz("ball")
                if not is_within_bounds(arm_xyz, arm_min_bounds, arm_max_bounds):
                    pyhop.failure_reason = "can't move arm to {}: {} outside of bounds: {} {}"\
                        .format(to_, arm_xyz, arm_min_bounds, arm_max_bounds)
                    return False, False
                return arm_xyz, servo_values
            elif to_ == "container":
                arm_xyz, servo_values = get_xyz("container")
                if not is_within_bounds(arm_xyz, arm_min_bounds, arm_max_bounds):
                    pyhop.failure_reason = "can't move arm to {}: {} outside of bounds: {} {}" \
                        .format(to_, arm_xyz, arm_min_bounds, arm_max_bounds)
                    return False, False
                return arm_xyz, servo_values
            elif to_ == "center":
                arm_xyz, servo_values = get_xyz("center")
                if not is_within_bounds(arm_xyz, arm_min_bounds, arm_max_bounds):
                    pyhop.failure_reason = "can't move arm to {}: {} outside of bounds: {} {}" \
                        .format(to_, arm_xyz, arm_min_bounds, arm_max_bounds)
                    return False, False
                return arm_xyz, servo_values
            else:
                pyhop.failure_reason = "can't move arm to {}".format(to_)
                return False, False

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

        def initialize(state, actor):
            if actor == "arm":
                arm_xyz, servo_values = center_servos(state)
                if arm_xyz == [0, 0, 1] and servo_values == [1500, 1500, 1500, 1500, 1500, 1500]:
                    state.location['arm_xyz'] = arm_xyz
                    state.location['servo_values'] = servo_values
                    state.initialized["arm"] = True
                    return state
            pyhop.failure_reason = "{} can't initialize".format(actor)
            return False

        def grab(state, actor, actee, from_):
            if actee == "ball":
                if state.location['ball'] != 'table':
                    pyhop.failure_reason = "{} can't locate {} from {}".format(actor, actee, from_)
                    return False

                arm_xyz, servo_values = get_xyz(actee)
                if arm_xyz != False:
                    state.location['arm_xyz'] = arm_xyz
                    state.location['servo_values'] = servo_values
                    actee_diameter = get_diameter(actee)
                    if open_hand(actee_diameter):
                        arm_xyz, servo_values = move_arm(state, actee)
                        if arm_xyz != False:
                            if close_hand(actee_diameter):
                                state.location['arm_xyz'] = arm_xyz
                                state.location['servo_values'] = servo_values
                                state.grabbed["ball"] = True
                                return state
            pyhop.failure_reason = "{} can't grab {} from {}".format(actor, actee, from_)
            return False

        def put(state, actor, actee, to_):
            if to_ == "container":
                arm_xyz, servo_values = get_xyz(to_)
                if arm_xyz != False:
                    state.location['arm_xyz'] = arm_xyz
                    state.location['servo_values'] = servo_values
                    arm_xyz, servo_values = move_arm(state, to_)
                    if arm_xyz != False:
                        actee_diameter = get_diameter(actee)
                        if open_hand(actee_diameter):
                            state.location['arm_xyz'] = arm_xyz
                            state.location['servo_values'] = servo_values
                            return state
            pyhop.failure_reason = "{} can't put {} to {}".format(actor, actee, to_)
            return False

        pyhop.declare_operators(initialize, grab, put, move_arm, close_hand, open_hand)
        # print('')
        pyhop.print_operators()

        # Methods (compound tasks)

        def put_grabbed(state, actor, actee, from_, to_):
            if actor == "arm":
                if state.grabbed["ball"]:
                    return [('put', actor, actee, to_)]

            pyhop.failure_reason = "{} can't put_grabbed {} from {} to {}".format(actor, actee, from_, to_)
            return False

        def initialize_transfer(state, actor, actee, from_, to_):
            if actor == "arm":
                return [('initialize', actor), ('grab', actor, actee, from_), ('put', actor, actee, to_)]

            pyhop.failure_reason = "{} can't initialize and transfer {} from {} to {}".format(actor, actee, from_, to_)
            return False

        def transfer(state, actor, actee, from_, to_):
            if actor == "arm":
                if state.initialized["arm"]:
                    return [('grab', actor, actee, from_), ('put', actor, actee, to_)]

            pyhop.failure_reason = "{} can't transfer {} from {} to {}".format(actor, actee, from_, to_)
            return False

        pyhop.declare_methods('transfer_ball_to_container', initialize_transfer, put_grabbed, transfer)

    def get_plans(self, world_model, goal):
        return pyhop.pyhop(world_model, goal, verbose=0, all_plans=True, sort_asc=True)
