from control import Control


class Monitoring:

    def __init__(self):
        self.control = Control()

    def execute_action(self, action):
        action_successful = False

        if action == ('initialize', 'arm'):  # TODO: for testing only
            action, actor = action
            # TODO:
            action_successful = self.control.initialize_arm()
        #     percept = {"initialized": {'arm': True}}
        #     beliefs = beliefs.belief_revision(percept)
        elif action == ('grab', 'arm', 'target_object', 'table'):
            action_successful = self.control.close_hand()
        #     percept = {"grabbed": {'target_object': True}, "initialized": {'arm': False}}
        #     beliefs = beliefs.belief_revision(percept)
        elif action == ('put', 'arm', 'target_object', 'container'):
            action_successful = self.control.move_arm_to_container()
        #     percept = {"location": {"target_object": "container"}}
        #     beliefs = beliefs.belief_revision(percept)
        #     percept = {"grabbed": {'target_object': False}}
        #     beliefs = beliefs.belief_revision(percept)

        return action_successful


if __name__ == '__main__':

    monitoring = Monitoring()
    monitoring.control.send_requests = True  # TODO: test
    monitoring.control.center_init = False  # TODO: test
    monitoring.control.detect_last_position = True
    monitoring.execute_action(('initialize', 'arm'))
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
