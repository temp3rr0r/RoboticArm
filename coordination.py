from control import Control


class Coordination:
    """
    Executes actions (discrete), by invoking control commands (continuous).
    """

    def __init__(self):
        self.control = Control()
        self.verbose = False

    def execute_action(self, action, world_model):
        """
        Executes actions by using information from the world model and invoking control commands.
        :param action: Tuple of "actor", "actee" (and "from", "to" in some cases).
        :param world_model: The current world state, to extract information from.
        :return: True if action command was executed successful.
        """
        action_successful = False

        if action == ('initialize', 'arm'):
            object_side_length = world_model.size["object_side_length"]
            # action_successful = self.control.initialize_arm()
        elif action == ('open_hand',):
            action_successful = self.control.open_hand(world_model.size["object_side_length"])
        elif action == ('move_arm_above', 'target_object'):
            action_successful = self.control.move_arm_above_xyz(
                world_model.xyz["target_object"], world_model.location["servo_values"],
                world_model.size["object_side_length"] * 2.0)
        elif action == ('move_arm_up', 'target_object'):
            action_successful = self.control.move_arm_up(
                world_model.location["servo_values"],
                world_model.size["object_side_length"] * 2.0)
        elif action == ('move_arm', 'target_object'):
            action_successful = self.control.move_arm_above_xyz(
                world_model.xyz["target_object"], world_model.location["servo_values"],
                world_model.size["object_side_length"] * 0.5)
        elif action == ('close_hand',):
            action_successful = self.control.close_hand(world_model.size["object_side_length"])
        elif action == ('move_arm_above', 'container'):
            action_successful = self.control.move_arm_above_xyz(
                world_model.xyz["container"], world_model.location["servo_values"], 14)

        return action_successful


if __name__ == '__main__':

    # Sequence for testing
    from world_model import WorldModel
    current_world_model = WorldModel()
    coordination = Coordination()
    coordination.control.send_requests = False
    coordination.control.center_init = False
    coordination.control.detect_last_position = False
    # coordination.execute_action(('initialize', 'arm'), current_world_model.current_world_model)
    # coordination.execute_action(('grab', 'arm', 'target_object', 'table'), current_world_model.current_world_model)
    # coordination.execute_action(('put', 'arm', 'target_object', 'container'), current_world_model.current_world_model)

    # [('initialize', 'arm'), ('open_hand',), ('move_arm_above', 'target_object'), ('move_arm', 'target_object'),
    #  ('close_hand',), ('move_arm_above', 'target_object'), ('move_arm_above', 'container'), ('open_hand',)]

    coordination.execute_action(('initialize', 'arm'), current_world_model.current_world_model)
    coordination.execute_action(('open_hand', ), current_world_model.current_world_model)
    coordination.execute_action(('move_arm_above', 'target_object'), current_world_model.current_world_model)
    coordination.execute_action(('move_arm', 'target_object'), current_world_model.current_world_model)
    coordination.execute_action(('close_hand',), current_world_model.current_world_model)
    coordination.execute_action(('move_arm_up', 'target_object'), current_world_model.current_world_model)

    coordination.execute_action(('move_arm_above', 'container'), current_world_model.current_world_model)
    coordination.execute_action(('open_hand', ), current_world_model.current_world_model)
    # coordination.execute_action(('grab', 'arm', 'target_object', 'table'), current_world_model.current_world_model)
    # coordination.execute_action(('put', 'arm', 'target_object', 'container'), current_world_model.current_world_model)
    import numpy as np
    # target_position = np.array([12.5, -12.5, 2.0]) * coordination.control.scale
    target_position = np.array([20, -20.0, 20]) * coordination.control.scale
    # target_position = np.array([12.5, -12.5, 25]) * coordination.control.scale
    # target_position = np.array([-16, 0.0, 10]) * coordination.control.scale
    # target_position = np.array([-20, -20, 25]) * coordination.control.scale
    # target_position = np.array([0, 0, 0]) * coordination.control.scale
    # target_position = np.array([-13.12, 0.27, 1.5]) * coordination.control.scale
    last_servo_values = current_world_model.current_world_model.location["servo_values"]
    action_successful_test = coordination.control.move_arm(np.array(target_position), last_servo_values)
