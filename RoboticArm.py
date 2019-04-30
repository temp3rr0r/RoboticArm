from components import Control, Perception, Plan, WorldModel, Monitoring


class RoboticArm:

    def __init__(self):

        self.tick = 1  # seconds  # TODO: tick vs heartbeat?
        self.steps = 100  # seconds
        self.control = Control
        self.plan = Plan
        self.monitoring = Monitoring
        self.world_model = WorldModel
        self.perception = Perception

    def run(self):
        for step in range(self.steps):
            # do stuff
            pass
