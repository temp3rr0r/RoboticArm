from control import Control


class Monitoring:

    def __init__(self):
        self.control = Control()
        self.verbose = False

    def execute_action(self, action, world_model):
        action_successful = False

        if action == ('initialize', 'arm'):
            object_side_length = world_model.size["object_side_length"]
            action_successful = self.control.initialize_arm()
            action_successful = self.control.open_hand(object_side_length)
        elif action == ('grab', 'arm', 'target_object', 'table'):
            # pass
            xyz = world_model.xyz["target_object"]
            object_side_length = world_model.size["object_side_length"]
            if self.verbose:
                print("target_object xyz: {}".format(xyz))
            action_successful = self.control.move_arm_above_xyz(xyz, object_side_length * 2.0)
            action_successful = self.control.move_arm_above_xyz(xyz, object_side_length * 0.5)
            action_successful = self.control.close_hand(object_side_length)
            action_successful = self.control.move_arm_above_xyz(xyz, object_side_length * 2.0)
        elif action == ('put', 'arm', 'target_object', 'container'):
            container_xyz = world_model.xyz["container"]
            object_side_length = world_model.size["object_side_length"]
            action_successful = self.control.move_arm_above_xyz(container_xyz, 14)
            action_successful = self.control.open_hand(object_side_length)

        return action_successful


if __name__ == '__main__':

    # Sequence for testing
    from world_model import WorldModel
    current_world_model = WorldModel()
    monitoring = Monitoring()
    monitoring.control.send_requests = False
    monitoring.control.center_init = False
    monitoring.control.detect_last_position = False
    monitoring.execute_action(('initialize', 'arm'), current_world_model.current_world_model)
    monitoring.execute_action(('grab', 'arm', 'target_object', 'table'), current_world_model.current_world_model)
    monitoring.execute_action(('put', 'arm', 'target_object', 'container'), current_world_model.current_world_model)
    import numpy as np
    # target_position = np.array([12.5, -12.5, 2.0]) * monitoring.control.scale
    target_position = np.array([20, -20.0, 20]) * monitoring.control.scale
    # target_position = np.array([12.5, -12.5, 25]) * monitoring.control.scale
    # target_position = np.array([-16, 0.0, 10]) * monitoring.control.scale
    # target_position = np.array([-20, -20, 25]) * monitoring.control.scale
    # target_position = np.array([0, 0, 0]) * monitoring.control.scale
    # target_position = np.array([-13.12, 0.27, 1.5]) * monitoring.control.scale
    action_successful_test = monitoring.control.move_arm(np.array(target_position))
