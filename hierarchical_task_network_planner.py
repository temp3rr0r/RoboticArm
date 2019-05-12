import pyhop


class HierarchicalTaskNetworkPlanner:

    def __init__(self):
        self.failure_reason = ""

        # Helper methods

        def center_servos(state):
            return move_arm(state, "center")

        def get_diameter(actee):
            if actee == "target_object":
                return 3  # cm
            self.failure_reason = "can't get diameter of {}".format(actee)
            return False

        def is_within_bounds(location1, min_bounds, max_bounds):
            for i in range(len(location1)):
                if location1[i] < min_bounds[i] or location1[i] > max_bounds[i]:
                    return False
            return True

        def get_xyz(state, actee):
            if actee == "target_object":
                xyz = state.xyz["target_object"]
                servo_values = [1500, 1500, 1500, 1500, 1500, 1500]  # TODO: hardcode real servo values
                return xyz, servo_values
            elif actee == "container":
                xyz = [10, 10, 15.0]
                servo_values = [1500, 1500, 1500, 1500, 1500, 1500]  # TODO: hardcode real servo values
                return xyz, servo_values
            elif actee == "center":
                xyz = [0, 0, 1]  # TODO: mock get arm position
                servo_values = [1500, 1500, 1500, 1500, 1500, 1500]
                return xyz, servo_values

            self.failure_reason = "can't get xyz of {}".format(actee)
            return False, False

        # Operators (primitive tasks)

        def move_arm(state, to_):
            arm_min_bounds = state.min_bounds["xyz"]
            arm_max_bounds = state.max_bounds["xyz"]
            if to_ == "target_object":
                arm_xyz, servo_values = get_xyz(state, "target_object")
                if not is_within_bounds(arm_xyz, arm_min_bounds, arm_max_bounds):
                    self.failure_reason = "can't move arm to {}: {} outside of bounds: {} {}"\
                        .format(to_, arm_xyz, arm_min_bounds, arm_max_bounds)
                    return False, False
                return arm_xyz, servo_values
            elif to_ == "container":
                arm_xyz, servo_values = get_xyz(state, "container")
                if not is_within_bounds(arm_xyz, arm_min_bounds, arm_max_bounds):
                    self.failure_reason = "can't move arm to {}: {} outside of bounds: {} {}" \
                        .format(to_, arm_xyz, arm_min_bounds, arm_max_bounds)
                    return False, False
                return arm_xyz, servo_values
            elif to_ == "center":
                arm_xyz, servo_values = get_xyz(state, "center")
                if not is_within_bounds(arm_xyz, arm_min_bounds, arm_max_bounds):
                    self.failure_reason = "can't move arm to {}: {} outside of bounds: {} {}" \
                        .format(to_, arm_xyz, arm_min_bounds, arm_max_bounds)
                    return False, False
                return arm_xyz, servo_values
            else:
                self.failure_reason = "can't move arm to {}".format(to_)
                return False, False

        def close_hand(distance):
            # get servo value for cm from distance
            current_hand_servo_value = 1300
            suggested_hand_distance_servo_value = 1000
            distance_ratio = 0.9

            if 500 < suggested_hand_distance_servo_value <= 1500:
                return suggested_hand_distance_servo_value * distance_ratio

            self.failure_reason = "can't close hand to distance {}".format(distance)
            return False

        def open_hand(distance):
            # get servo value for cm from distance
            current_hand_servo_value = 1000
            suggested_hand_distance_servo_value = 1300
            distance_ratio = 1.1

            if 500 < suggested_hand_distance_servo_value <= 1500:
                return suggested_hand_distance_servo_value * distance_ratio

            self.failure_reason = "can't open hand to distance {}".format(distance)
            return False

        def initialize(state, actor):
            if actor == "arm":
                arm_xyz, servo_values = center_servos(state)
                if arm_xyz == [0, 0, 1] and servo_values == [1500, 1500, 1500, 1500, 1500, 1500]:
                    state.location['arm_xyz'] = arm_xyz
                    state.location['servo_values'] = servo_values
                    state.initialized["arm"] = True
                    return state
            self.failure_reason = "{} can't initialize".format(actor)
            return False

        def grab(state, actor, actee, from_):
            if actee == "target_object":
                if state.location['target_object'] != 'table':
                    self.failure_reason = "{} can't locate {} from {}".format(actor, actee, from_)
                    return False

                arm_xyz, servo_values = get_xyz(state, actee)
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
                                state.grabbed["target_object"] = True
                                return state
            self.failure_reason = "{} can't grab {} from {}".format(actor, actee, from_)
            return False

        def put(state, actor, actee, to_):
            if to_ == "container":
                arm_xyz, servo_values = get_xyz(state, to_)
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
            self.failure_reason = "{} can't put {} to {}".format(actor, actee, to_)
            return False

        pyhop.declare_operators(initialize, grab, put, move_arm, close_hand, open_hand)
        # print('')
        pyhop.print_operators()

        # Methods (compound tasks)

        def put_grabbed(state, actor, actee, from_, to_):
            if actor == "arm":
                if state.grabbed["target_object"]:
                    return [('put', actor, actee, to_)]

            self.failure_reason = "{} can't put_grabbed {} from {} to {}".format(actor, actee, from_, to_)
            return False

        def initialize_transfer(state, actor, actee, from_, to_):
            if actor == "arm":
                # TODO: connect tasks with actual actions
                # TODO: fully flatten actions
                # TODO: add all primitive tasks (from coordination): move_arm_above_xyz, close_hand, open_hand, move_arm_above_xyz
                return [('initialize', actor), ('grab', actor, actee, from_), ('put', actor, actee, to_)]

                # def open_hand(distance):
                # def close_hand(distance):
                # distance = 3.3
                # return [('initialize', actor),
                #         ('open_hand', distance),
                #         ('grab', actor, actee, from_),
                #         ('put', actor, actee, to_)]

            self.failure_reason = "{} can't initialize and transfer {} from {} to {}".format(actor, actee, from_, to_)
            return False

        def transfer(state, actor, actee, from_, to_):
            if actor == "arm":
                if state.initialized["arm"]:
                    return [('grab', actor, actee, from_), ('put', actor, actee, to_)]

            self.failure_reason = "{} can't transfer {} from {} to {}".format(actor, actee, from_, to_)
            return False

        pyhop.declare_methods('transfer_target_object_to_container', initialize_transfer, put_grabbed, transfer)

    def get_plans(self, world_model, goal):
        """

        :param world_model: The current perceived world state (json object).
        :param goal: The end goal we try to achieve (tuple).
        :return: List of suggested plans.
        """

        if goal is not "":
            return pyhop.pyhop(world_model, goal, verbose=0, all_plans=True, sort_asc=True)
        else:
            return ""


if __name__ == '__main__':

    htn_planner = HierarchicalTaskNetworkPlanner()
    goal = [('transfer_target_object_to_container', 'arm', 'target_object', 'table', 'container')]
    intentions = goal  # I := I0; Initial Intentions
    from world_model import WorldModel
    beliefs = WorldModel()  # B := B0; Initial Beliefs
    beliefs.current_world_model.xyz["target_object"] = [-10, -10, 0]

    htn_plans = htn_planner.get_plans(beliefs.current_world_model, intentions)  # π := plan(B, I); MEANS_END REASONING
    print()
    if not htn_plans:
        print("-- No valid plan. Failure_reason: {}".format(htn_planner.failure_reason))
    else:
        beliefs.current_world_model.plans = htn_plans
        print("Best current_world_model.plan: ", beliefs.current_world_model.plans[0])
