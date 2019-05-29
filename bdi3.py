import datetime
from world_model import WorldModel
from hierarchical_task_network_planner import HierarchicalTaskNetworkPlanner
from collections import deque
from perception import Perception
from coordination import Coordination
from monitoring import Monitoring
import datetime
from spade.agent import Agent
from spade.behaviour import CyclicBehaviour, PeriodicBehaviour
from spade.message import Message


class BDIAgent(Agent):
    class BDIBehaviour(PeriodicBehaviour):
        async def on_start(self):

            self.terminate = False
            self.SUCCESS = False
            self.verbose = True
            # Initialization
            self.htn_planner = HierarchicalTaskNetworkPlanner()
            self.goal = [('transfer_target_object_to_container', 'arm', 'target_object', 'table', 'container')]
            self.intentions = self.goal  # I := I0; Initial Intentions
            self.beliefs = WorldModel()  # B := B0; Initial Beliefs
            self.monitoring = Monitoring()
            self.perception = Perception()
            self.coordination = Coordination()

            # Disable all 3 coordination switches for testing
            self.coordination.control.send_requests = True
            self.coordination.control.center_init = False
            self.coordination.control.detect_last_position = True

            self.what, self.why, self.how_well, self.what_else, self.why_failed = "", "", "", "", ""
            self.plans = []
            self.start_time = datetime.datetime.now()

        async def run(self):

            # print(f"-- Sender Agent: PeriodicSenderBehaviour running at {datetime.datetime.now().time()}: {self.counter}")

            if not self.SUCCESS and not self.terminate and self.beliefs.update_tick() < self.beliefs.current_world_model.max_ticks:

                # msg = Message(to="madks2@temp3rr0r-pc")  # Instantiate the message
                # msg.body = "Hello World: " + str(self.counter)  # Set the message content
                # await self.send(msg)

                percept = self.perception.get_percept(text_engraving=(self.why_failed, self.how_well))  # get next percept ρ; OBSERVE the world
                self.beliefs = self.perception.belief_revision(self.beliefs, percept)  # B:= brf(B, ρ);
                self.beliefs = self.monitoring.fire_events(self.beliefs, percept)
                self.intentions = self.deliberate(self.beliefs,
                                                  self.intentions)  # DELIBERATE about what INTENTION to achieve next
                SUCCESS = True if self.intentions == "" else False
                self.plans = self.htn_planner.get_plans(self.beliefs.current_world_model,
                                              self.intentions)  # π := plan(B, I); MEANS_END REASONING

                if self.plans != False:
                    if len(self.plans) > 0:
                        self.plans.sort(key=len)  # TODO: check why sorting doesn't work on "deeper" levels
                        if self.verbose:
                            print("{}: Plan: {}".format(self.beliefs.current_world_model.tick, self.plans[0]))
                        selected_plan = deque(
                            self.plans[0])  # TODO: Use a "cost function" to evaluate the best plan, not shortest
                        why_failed = ""

                        # while not (empty(π) or succeeded(Ι, Β) or impossible(I, B)) do
                        while len(selected_plan) > 0 and self.beliefs.update_tick() < self.beliefs.current_world_model.max_ticks:
                            action, selected_plan = selected_plan.popleft(), selected_plan  # α := hd(π); π := tail(π);

                            if self.verbose:
                                print("{}: Action: {}".format(self.beliefs.current_world_model.tick, action))

            else:
                why_failed = self.htn_planner.failure_reason
                if self.verbose:
                    print("Plan failure_reason: {}".format(why_failed), end=" ")
                how_well = (self.beliefs.current_world_model.tick, self.beliefs.current_world_model.max_ticks,
                            int((datetime.datetime.now() - self.start_time).total_seconds() * 1000),  # milliseconds
                            self.plans)
                if self.verbose:
                    print("how_well: {}".format(how_well))
                self.done()
                self.kill()

        async def on_end(self):
            print("-- on_end")
            print("Done.")
            self.perception.destroy()
            # stop agent from behaviour
            await self.agent.stop()

        # async def _step(self):  # TODO: Need?
        #     # the bdi stuff
        #     pass

        def done(self):
            # the done evaluation
            print("-- Sender Agent: Done")

        def deliberate(self, current_beliefs, current_intentions):
            """
            Decide WHAT sate of affairs you want to achieve.
            :param current_beliefs: WordModel instance, the world model, information about the world.
            :param current_intentions: List of tuples, tasks that the agent has COMMITTED to accomplish.
            :return: List of tuples, filtered intentions that the agent COMMITS to accomplish.
            """
            # 1. Option Generation: The agent generates a set of possible alternatives.
            current_desires = current_intentions  # TODO: D := option(B, I);

            # 2. Filtering: Agent chooses between competing alternatives and commits to achieving them.
            current_intentions = self.filter_intentions(
                current_beliefs, current_desires, current_intentions)  # I := filter(B, D, I);

            return current_intentions

        def filter_intentions(self, current_beliefs, current_desires, current_intentions):
            """
            Choose between competing alternatives and COMMITTING to achieving them.
            :param current_beliefs: WordModel instance, the world model, information about the world.
            :param current_desires: List of tuples, tasks that the agent would like to accomplish.
            :param current_intentions: List of tuples, tasks that the agent has COMMITTED to accomplish.
            :return: List of tuples, filtered intentions that the agent COMMITS to accomplish.
            """
            for current_intention in current_intentions:
                if current_intention == (
                'transfer_target_object_to_container', 'arm', 'target_object', 'table', 'container') \
                        and current_beliefs.current_world_model.location["target_object"] == "container":
                    current_intentions = ""  # if goal(s) achieved, empty I
            return current_intentions

    async def setup(self):
        print(f"-- Sender Agent: PeriodicSenderAgent started at {datetime.datetime.now().time()}")
        start_at = datetime.datetime.now() + datetime.timedelta(seconds=5)
        b = self.BDIBehaviour(period=2, start_at=start_at)
        self.add_behaviour(b)


if __name__ == "__main__":
    # receiveragent = ReceiverAgent("madks2@temp3rr0r-pc", "ma121284")
    # future = receiveragent.start()
    # future.result()  # wait for receiver agent to be prepared.
    senderagent = BDIAgent("madks@temp3rr0r-pc", "ma121284")
    senderagent.start()

    # while receiveragent.is_alive():
    #     try:
    #         time.sleep(1)
    #     except KeyboardInterrupt:
    #         senderagent.stop()
    #         receiveragent.stop()
    #         break
    print("Agents finished")
