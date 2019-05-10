import pyhop
import time
import copy


class WorldModel:
    """
    Stores and updates current & past world models, instances of a Pyhop State class.
    """

    def __init__(self):
        self.current_world_model = pyhop.State('current_world_model')
        self.current_world_model.tick = 0
        self.current_world_model.timestamp = time.time()
        self.current_world_model.location = {'target_object': 'table'}
        self.current_world_model.xyz = {'target_object': [-30, -30, 0], 'container': [-0.1, 24.0, 12]}
        self.current_world_model.size = {'object_side_length': 4.0}
        self.current_world_model.grabbed = {'target_object': False}
        self.current_world_model.initialized = {'arm': False}
        self.current_world_model.min_bounds = {'xyz': [-25, -25, -25]}
        self.current_world_model.max_bounds = {'xyz': [25, 25, 25]}
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

    def belief_revision(self, percept):
        """
        Updates the internal world model: B = beliefRevisionFunction(B, ρ)
        :param percept: Dictionary.
        :return: The updated world model, instance of pyhop State class.
        """

        if percept is not "":  # TODO: check the not
            self.world_model_history.append(copy.deepcopy(self.current_world_model))  # Store the current world model

            for key in percept.keys():
                if key == "grabbed":
                    self.current_world_model.grabbed = percept["grabbed"]
                elif key == "location":
                    self.current_world_model.location = percept["location"]
                elif key == "initialized":
                    self.current_world_model.initialized = percept["initialized"]
                elif key == "xyz":
                    self.current_world_model.xyz["target_object"] = percept["xyz"]["target_object"]

        return self

