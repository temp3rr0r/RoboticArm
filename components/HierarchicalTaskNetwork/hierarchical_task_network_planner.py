import pyhop


class HierarchicalTaskNetworkPlanner:

    def __init__(self):
        self.failure_reason = ""
        self.verbose = 0

        # Helper methods

        def is_within_bounds(location1, min_bounds, max_bounds):
            for i in range(len(location1)):
                if location1[i] < min_bounds[i] or location1[i] > max_bounds[i]:
                    return False
            return True

        # Operators (primitive tasks)

        def move_arm(state, to_):
            arm_min_bounds = state.min_bounds["xyz"]
            arm_max_bounds = state.max_bounds["xyz"]

            if to_ == "target_object" or to_ == "container":
                xyz = state.xyz[to_]
                if not is_within_bounds(xyz, arm_min_bounds, arm_max_bounds):
                    self.failure_reason = "can't move arm to {}: {} outside of bounds: {} {}"\
                        .format(to_, xyz, arm_min_bounds, arm_max_bounds)
                    return False
                return state

            return state

        def move_arm_above(state, to_):
            return move_arm(state, to_)

        def close_hand(state):
            gripper_min_bounds = state.min_bounds["object_side_length"]
            gripper_max_bounds = state.max_bounds["object_side_length"]
            distance = state.size['object_side_length']

            if distance < gripper_min_bounds or distance > gripper_max_bounds:
                self.failure_reason = "can't close hand to distance {}".format(distance)
                print(self.failure_reason)
                return False

            return state

        def open_hand(state):
            gripper_min_bounds = state.min_bounds["object_side_length"]
            gripper_max_bounds = state.max_bounds["object_side_length"]
            distance = state.size['object_side_length']

            if distance < gripper_min_bounds or distance > gripper_max_bounds:
                self.failure_reason = "can't open hand to distance {}".format(distance)
                return False

            return state

        def initialize(state, actor):
            if actor == "arm":
                state.initialized["arm"] = True
                return state
            self.failure_reason = "{} can't initialize".format(actor)
            return False

        pyhop.declare_operators(initialize, move_arm, move_arm_above, close_hand, open_hand)
        if self.verbose > 0:
            pyhop.print_operators()

        # Methods (compound tasks)

        def put_grabbed(state, actor, actee, from_, to_):
            if actor == "arm":
                if state.grabbed["target_object"]:
                    return [
                            ('move_arm_above', to_),
                            ('open_hand',)
                            ]
            return False

        def full_transfer(state, actor, actee, from_, to_):
            if actor == "arm":
                return [('initialize', actor),
                        ('open_hand',),

                        ('move_arm_above', 'target_object'),
                        ('move_arm', 'target_object'),
                        ('close_hand',),
                        ('move_arm_above', 'target_object'),

                        ('move_arm_above', to_),
                        ('open_hand',)
                        ]
            return False

        def transfer(state, actor, actee, from_, to_):
            if actor == "arm":
                if state.initialized["arm"]:
                    return [('move_arm_above', 'target_object'),
                            ('move_arm', 'target_object'),
                            ('close_hand',),
                            ('move_arm_above', 'target_object'),

                            ('move_arm_above', to_),
                            ('open_hand',)
                            ]
            return False

        pyhop.declare_methods('transfer_target_object_to_container', full_transfer, put_grabbed, transfer)

    def get_plans(self, world_model, goal):
        """

        :param world_model: The current perceived world state (json object).
        :param goal: The end goal we try to achieve (tuple).
        :return: List of suggested plans.
        """

        if goal is not "":
            return pyhop.pyhop(world_model, goal, verbose=self.verbose, all_plans=True, sort_asc=True)
        else:
            return ""


if __name__ == '__main__':

    htn_planner = HierarchicalTaskNetworkPlanner()
    end_goal = [('transfer_target_object_to_container', 'arm', 'target_object', 'table', 'container')]
    intentions = end_goal  # I := I0; Initial Intentions
    from world_model import WorldModel
    beliefs = WorldModel()  # B := B0; Initial Beliefs

    print()
    beliefs.current_world_model.xyz["target_object"] = [-10, -10, 0]
    htn_plans = htn_planner.get_plans(beliefs.current_world_model, intentions)  # π := plan(B, I); MEANS_END REASONING
    if not htn_plans:
        print("-- No valid plan. Failure_reason: {}".format(htn_planner.failure_reason))
    else:
        beliefs.current_world_model.plans = htn_plans
        print("== Best current_world_model.plan: ", beliefs.current_world_model.plans[0])

    print()
    beliefs.current_world_model.xyz["target_object"] = [-510, -10, 0]
    htn_plans = htn_planner.get_plans(beliefs.current_world_model, intentions)  # π := plan(B, I); MEANS_END REASONING
    if not htn_plans:
        print("-- No valid plan. Failure_reason: {}".format(htn_planner.failure_reason))
    else:
        beliefs.current_world_model.plans = htn_plans
        print("Best current_world_model.plan: ", beliefs.current_world_model.plans[0])

    print()
    beliefs.current_world_model.xyz["container"] = [-510, -10, 0]
    htn_plans = htn_planner.get_plans(beliefs.current_world_model, intentions)  # π := plan(B, I); MEANS_END REASONING
    if not htn_plans:
        print("-- No valid plan. Failure_reason: {}".format(htn_planner.failure_reason))
    else:
        beliefs.current_world_model.plans = htn_plans
        print("Best current_world_model.plan: ", beliefs.current_world_model.plans[0])

    print()
    beliefs.current_world_model.size['object_side_length'] = 53.0
    htn_plans = htn_planner.get_plans(beliefs.current_world_model, intentions)  # π := plan(B, I); MEANS_END REASONING
    if not htn_plans:
        print("-- No valid plan. Failure_reason: {}".format(htn_planner.failure_reason))
    else:
        beliefs.current_world_model.plans = htn_plans
        print("Best current_world_model.plan: ", beliefs.current_world_model.plans[0])
