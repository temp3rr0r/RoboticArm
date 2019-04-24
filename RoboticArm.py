from components import Control, Perception, Plan, WorldModel, Monitoring


class RoboticArm:

    def __init__(self):

        self.heartbeat = 1  # seconds
        self.steps = 100 # seconds
        self.control = Control
        self.plan = Plan
        self.monitoring = Monitoring
        self.world_model = WorldModel
        self.perception = Perception

    def run(self):
        for step in range(self.steps):
            # do stuff
            pass
