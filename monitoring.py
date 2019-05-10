from control import Control


class Monitoring:

    def __init__(self):
        self.control = Control()
        self.verbose = False

    def execute_action(self, action, world_model):
        action_successful = False

        if action == ('initialize', 'arm'):  # TODO: for testing only
            action, actor = action
            # TODO:
            object_side_length = 4.0
            action_successful = self.control.initialize_arm()
            action_successful = self.control.open_hand(object_side_length)
        #     percept = {"initialized": {'arm': True}}
        #     beliefs = beliefs.belief_revision(percept)
        elif action == ('grab', 'arm', 'target_object', 'table'):
            # pass
            xyz = world_model.xyz["target_object"]
            object_side_length = 4.0
            if self.verbose:
                print("target_object xyz: {}".format(xyz))
            xyz[2] = object_side_length * 2.0
            action_successful = self.control.move_arm_to_object(xyz)  # Above object
            xyz[2] = object_side_length * 0.5
            action_successful = self.control.move_arm_to_object(xyz)  # To object
            action_successful = self.control.close_hand(object_side_length)
        #     percept = {"grabbed": {'target_object': True}, "initialized": {'arm': False}}
        #     beliefs = beliefs.belief_revision(percept)
        elif action == ('put', 'arm', 'target_object', 'container'):
            container_xyz = [-15.0, 24.0, 19]
            container_xyz = [-0.1, 24.0, 19]
            object_side_length = 4.4
            action_successful = self.control.move_arm_to_container(container_xyz)
            action_successful = self.control.open_hand(object_side_length)
        #     percept = {"location": {"target_object": "container"}}
        #     beliefs = beliefs.belief_revision(percept)
        #     percept = {"grabbed": {'target_object': False}}
        #     beliefs = beliefs.belief_revision(percept)

        return action_successful


if __name__ == '__main__':

    # Sequence for testing
    monitoring = Monitoring()
    # monitoring.control.send_requests = True
    monitoring.control.center_init = False
    # monitoring.control.detect_last_position = True
    monitoring.execute_action(('initialize', 'arm'))
    monitoring.execute_action(('grab', 'arm', 'target_object', 'table'))
    monitoring.execute_action(('put', 'arm', 'target_object', 'container'))
    import numpy as np
    # target_position = np.array([12.5, -12.5, 2.0]) * monitoring.control.scale
    target_position = np.array([20, -20.0, 20]) * monitoring.control.scale
    # target_position = np.array([12.5, -12.5, 25]) * monitoring.control.scale
    # target_position = np.array([-16, 0.0, 10]) * monitoring.control.scale
    # target_position = np.array([-20, -20, 25]) * monitoring.control.scale
    # target_position = np.array([0, 0, 0]) * monitoring.control.scale
    # target_position = np.array([-13.12, 0.27, 1.5]) * monitoring.control.scale
    action_successful = monitoring.control.move_arm(np.array(target_position))
