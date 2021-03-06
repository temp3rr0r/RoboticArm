import copy
import time


class Monitoring:
    """
    Raises events, if monitored sensor data exceed specific thresholds. From actual world, to world model.
    """

    @staticmethod
    def fire_events(world_model, percept):
        """
        Updates the current world model with events: B = fireEvents(B, ρ)
        :param world_model: World model, instance of the WorldModel class.
        :param percept: Dictionary. Input sensory data.
        :return: The updated world model, instance of the WorldModel class with updated list of raised events
        """
        if percept is not "":
            world_model.world_model_history.append(copy.deepcopy(world_model.current_world_model))  # Store as history

            for key in percept.keys():  # TODO: thresholds
                if key == "distance":    # TODO: if distance sensor <= 4.5 cm (min distance from plain) -> grabbed
                    distance_to_gripper = percept["distance"]["distance_to_gripper"]
                    if distance_to_gripper <= world_model.current_world_model.threshold["grabbing_distance"]:
                        world_model.current_world_model.grabbed["target_object"] = True
                        world_model.current_world_model.location["target_object"] = "arm"
                elif key == "location":  # TODO: if xyz of object within limits -> on table else -> not on table
                    for key2 in percept["location"]:
                        if key2 == "target_object":
                            world_model.current_world_model.location["target_object"] = percept["location"][
                                "target_object"]
                elif key == "initialized":  # TODO: if servos at 1500-ish -> initialized
                    world_model.current_world_model.initialized = percept["initialized"]

        return world_model


if __name__ == '__main__':

    # Sequence for testing
    from world_model import WorldModel
    beliefs = WorldModel()
    monitoring = Monitoring()

    time.sleep(0.1)
    current_percept = {"distance": {'distance_to_gripper': 8.2}}
    beliefs.update_tick()
    beliefs = monitoring.fire_events(beliefs, current_percept)

    time.sleep(0.1)
    current_percept = {"distance": {'distance_to_gripper': 5.2}}
    beliefs.update_tick()
    beliefs = monitoring.fire_events(beliefs, current_percept)

    time.sleep(0.1)
    current_percept = {"distance": {'distance_to_gripper': 2.2}}
    beliefs.update_tick()
    beliefs = monitoring.fire_events(beliefs, current_percept)

    print()
    print("Final World model:")
    print("-- Ticks: {}".format(beliefs.current_world_model.tick))
    print("-- initialized: {}".format(beliefs.current_world_model.initialized))
    print("-- location: {}".format(beliefs.current_world_model.location))
    print("-- grabbed: {}".format(beliefs.current_world_model.grabbed))
    print("-- timestamp: {}".format(beliefs.current_world_model.timestamp))
    print()
    print("World model History:")
    for tick in range(len(beliefs.world_model_history)):
        print("Tick {}:".format(tick))
        print("-- initialized: {}".format(beliefs.world_model_history[tick].initialized))
        print("-- location: {}".format(beliefs.world_model_history[tick].location))
        print("-- grabbed: {}".format(beliefs.world_model_history[tick].grabbed))
        print("-- timestamp: {}".format(beliefs.world_model_history[tick].timestamp))
