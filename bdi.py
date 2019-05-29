from world_model import WorldModel
from hierarchical_task_network_planner import HierarchicalTaskNetworkPlanner
from collections import deque
from perception import Perception
from coordination import Coordination
from monitoring import Monitoring
import datetime
from spade.agent import Agent
from spade.behaviour import PeriodicBehaviour
# from spade.behaviour import CyclicBehaviour  # TODO: Need?
# from spade.message import Message  # TODO: Need?

"""

Belief-Desire-Intention (BDI) - Practical Reasoning for resource bounded agents - Bratman 1988

practical reasoning = deliberation + means-ends reasoning

Base agent control loop
while true
  OBSERVE the world;
  UPDATE internal world model;
  DELIBERATE about what INTENTION to achieve next;
  use MEANS-END REASONING to get a PLAN for the intention;
  EXECUTE the plan;
end while
"""


class BDIAgent(Agent):

    class BDIBehaviour(PeriodicBehaviour):

        async def on_start(self):
            self.terminate = False
            self.SUCCESS = False
            self.verbose = False

            # Initialization
            self.htn_planner = HierarchicalTaskNetworkPlanner()
            self.goal = [('transfer_target_object_to_container', 'arm', 'target_object', 'table', 'container')]
            self.intentions = self.goal  # I := I0; Initial Intentions
            self.beliefs = WorldModel()  # B := B0; Initial Beliefs
            self.perception = Perception(self.beliefs)
            self.coordination = Coordination(self.beliefs)
            self.monitoring = Monitoring()

            self.coordination.control.send_requests = True  # Note: Disable all 3 coordination switches for testing
            self.coordination.control.center_init = False  # TODO: world model?
            self.coordination.control.detect_last_position = True

            self.what, self.why, self.how_well, self.what_else, self.why_failed = "", "", "", "", ""
            self.plans = []
            self.selected_plan = []
            self.percept = {}
            self.action = ""
            self.start_time = datetime.datetime.now()

        async def run(self):
            """
                Agent control loop version 7 (Intention Reconsideration)
                I := I0; Initial Intentions
                B := B0; Initial Beliefs
                while true do
                  get next percept ρ; #  OBSERVE the world
                  B:= brf(B, ρ); #  Belief revision function
                  D: = option(B, I);
                  I := filter(B, D, I);
                  π := plan(B, I); #  MEANS_END REASONING
                  while not (empty(π) or succeeded(Ι, Β) or impossible(I, B)) do  # Drop impossible or succeeded intentions
                      α := hd(π); #  Pop first action
                      execute(α);
                      π := tail(π);
                      get next percept ρ;
                      B:= brf(B, ρ);
                      if reconsider(I, B) then  # Meta-level control: explicit decision, to avoid reconsideration cost
                          D := options(B, I);
                          I := filter(B, D, I);
                      end-if
                      if not sound(π, I, B) then  # Re-activity, re-plan
                          π := plan(B, I);
                      end-if
                  end-while
                end-while
            """

            if self.verbose:
                print(f"--- arm_agent: "
                      f"PeriodicSenderBehaviour running at {datetime.datetime.now().time()}: {self.counter}")

            # while true do
            if not self.SUCCESS and not self.terminate and self.beliefs.update_tick() < self.beliefs.current_world_model.max_ticks:

                if len(self.selected_plan) == 0:
                    # msg = Message(to="madks2@temp3rr0r-pc")  # Instantiate the message  # TODO: place holder for IM
                    # msg.body = "Hello World: " + str(self.counter)  # Set the message content
                    # await self.send(msg)

                    # TODO: engrave figures: arm IK, gripper
                    self.percept = self.perception.get_percept(text_engraving=(self.why_failed, self.how_well))  # get next percept ρ; OBSERVE the world
                    self.beliefs = self.perception.belief_revision(self.beliefs, self.percept)  # B:= brf(B, ρ);
                    self.beliefs = self.monitoring.fire_events(self.beliefs, self.percept)
                    self.intentions = self.deliberate(self.beliefs,
                                                      self.intentions)  # DELIBERATE about what INTENTION to achieve next
                    self.SUCCESS = True if self.intentions == "" else False
                    self.plans = self.htn_planner.get_plans(self.beliefs.current_world_model,
                                                  self.intentions)  # π := plan(B, I); MEANS_END REASONING

                    if self.plans != False:
                        if len(self.plans) > 0:
                            self.plans.sort(key=len)  # TODO: check why sorting doesn't work on "deeper" levels
                            if self.verbose:
                                print("{}: Plan: {}".format(self.beliefs.current_world_model.tick, self.plans[0]))
                            self.selected_plan = deque(
                                self.plans[0])  # TODO: Use a "cost function" to evaluate the best plan, not shortest
                            self.why_failed = ""
                    else:
                        self.why_failed = self.htn_planner.failure_reason
                        if self.verbose:
                            print("Plan failure_reason: {}".format(self.why_failed), end=" ")
                        self.how_well = (
                        self.beliefs.current_world_model.tick, self.beliefs.current_world_model.max_ticks,
                        int((datetime.datetime.now() - self.start_time).total_seconds() * 1000),  # milliseconds
                        self.plans)
                        if self.verbose:
                            print("how_well: {}".format(self.how_well))

                else:  # while not (empty(π) or succeeded(Ι, Β) or impossible(I, B)) do
                    self.action, self.selected_plan = self.selected_plan.popleft(), self.selected_plan  # α := hd(π); π := tail(π);

                    if self.verbose:
                        print("{}: Action: {}".format(self.beliefs.current_world_model.tick, self.action))

                    self.coordination.execute_action(self.action, self.beliefs.current_world_model)  # execute(α);

                    self.what, self.why = self.action, self.intentions
                    self.how_well = (self.beliefs.current_world_model.tick, self.beliefs.current_world_model.max_ticks,
                                int((datetime.datetime.now() - self.start_time).total_seconds() * 1000),  # milliseconds
                                self.selected_plan)
                    self.what_else = self.plans[1] if len(self.plans) > 1 else self.plans[0]
                    if self.verbose:
                        self.print_answers(self.what, self.why, self.how_well, self.what_else)

                    # get next percept ρ; OBSERVE the world
                    self.percept = self.perception.get_percept(text_engraving=(self.what, self.why, self.how_well, self.what_else))
                    self.beliefs = self.perception.belief_revision(self.beliefs, self.percept)
                    self.beliefs = self.monitoring.fire_events(self.beliefs, self.percept)

                    # TODO: trigger sound percept?

                    if self.action == ('initialize', 'arm'):
                        self.percept = {"initialized": {'arm': True}}  # TODO: post conditions or monitoring
                        self.beliefs = self.perception.belief_revision(self.beliefs, self.percept)  # TODO: post conditions
                        self.beliefs = self.monitoring.fire_events(self.beliefs, self.percept)
                    elif self.action == ('move_arm', 'target_object'):
                        self.current_percept = {"distance": {'distance_to_gripper': 2.2}}  # TODO: update with monitoring
                        self.beliefs = self.perception.belief_revision(self.beliefs, self.percept)
                        self.beliefs = self.monitoring.fire_events(self.beliefs, self.percept)
                    elif self.action == ('move_arm_above', 'container'):
                        self.percept = {"location": {"target_object": "container"}, "grabbed": {'target_object': False}}
                        self.beliefs = self.perception.belief_revision(self.beliefs, self.percept)
                        self.beliefs = self.monitoring.fire_events(self.beliefs, self.percept)

                    # if reconsider(I, B) then
                    #   D := options(B, I);
                    #   I := filter(B, D, I);

                    # if not sound(π, I, B) then
                    #   π := plan(B, I)

                    # TODO: Backtracking of action?
            else:
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
            print("-- Sender Agent: Done")

            show_history = True
            print("Final World model:")
            print("-- Ticks: {}".format(self.beliefs.current_world_model.tick))
            print("-- initialized: {}".format(self.beliefs.current_world_model.initialized))
            print("-- location: {}".format(self.beliefs.current_world_model.location))
            print("-- grabbed: {}".format(self.beliefs.current_world_model.grabbed))
            if show_history:
                print()
                print("World model History:")
                for tick in range(len(self.beliefs.world_model_history)):
                    print("Tick {}:".format(tick))
                    print("-- initialized: {}".format(self.beliefs.world_model_history[tick].initialized))
                    print("-- location: {}".format(self.beliefs.world_model_history[tick].location))
                    print("-- grabbed: {}".format(self.beliefs.world_model_history[tick].grabbed))

            print()
            print("{}!".format("SUCCESS" if self.SUCCESS else "FAIL"))

        def print_answers(self, what_answer, why_answer, how_well_answer, what_else_answer):
            print("Q: What is the robot doing? A: {}\n"
                  "Q: Why is it doing it? A: {}\n"
                  "Q: How well is it doing it? A: {}\n"
                  "Q: What else could it have been doing instead? A: {}"
                  .format(what_answer, why_answer, how_well_answer, what_else_answer))

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
        print(f"--- arm_agent: PeriodicSenderAgent started at {datetime.datetime.now().time()}")
        init_world_model = WorldModel()
        start_at = datetime.datetime.now() + datetime.timedelta(seconds=init_world_model.current_world_model.init_delay_seconds["arm"])
        bdi_behaviour = self.BDIBehaviour(period=init_world_model.current_world_model.real_time_clock_period_seconds["arm"], start_at=start_at)
        self.add_behaviour(bdi_behaviour)


if __name__ == "__main__":
    arm_agent = BDIAgent("madks@temp3rr0r-pc", "ma121284")
    arm_agent.start()
