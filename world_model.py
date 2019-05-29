import pyhop
import time


class WorldModel:
    """
    Stores and updates current & past world models, instances of a Pyhop State class.
    """

    def __init__(self):
        self.current_world_model = pyhop.State('current_world_model')
        self.current_world_model.tick = 0
        self.current_world_model.max_ticks = 100
        self.current_world_model.timestamp = time.time()
        self.current_world_model.location = {
            'target_object': 'table',
            "servo_values": [1500, 1500, 1500, 1500, 1500, 1500]}  # TODO: Move 2 gremlin world model with init
        self.current_world_model.xyz = {'target_object': [-30, -30, 0],
                                        'container': [-0.1, 24.0, 12],
                                        'end_effector': [-0.1, 24.0, 12]
                                        }
        self.current_world_model.size = {'object_side_length': 4.0}
        self.current_world_model.min_bounds = {'xyz': [-25, -25, -25], 'object_side_length': 0.5}
        self.current_world_model.max_bounds = {'xyz': [25, 25, 25], 'object_side_length': 6.0}
        self.current_world_model.threshold = {'grabbing_distance': 4.5}  # cm
        self.current_world_model.distance = {'distance_to_gripper': 11.2}  # cm
        self.current_world_model.grabbed = {'target_object': False}
        self.current_world_model.initialized = {'arm': False}
        self.current_world_model.url = {'arm': "ESP_02662E"}

        self.current_world_model.plans = []
        self.world_model_history = []

    def update_tick(self):
        """
        Updates the tick count and the current timestamp.
        :return: The current tick.
        """
        self.current_world_model.tick += 1
        self.current_world_model.timestamp = time.time()
        return self.current_world_model.tick
