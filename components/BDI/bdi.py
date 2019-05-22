import datetime
from world_model import WorldModel
from hierarchical_task_network_planner import HierarchicalTaskNetworkPlanner
from collections import deque
from perception import Perception
from coordination import Coordination
from monitoring import Monitoring

"""

Belief-Desire-Instention (BDI) - Practical Reasoning for resource bounded agents - Bratman 1988

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


def print_answers(what_answer, why_answer, how_well_answer, what_else_answer):
    print("Q: What is the robot doing? A: {}\n"
          "Q: Why is it doing it? A: {}\n"
          "Q: How well is it doing it? A: {}\n"
          "Q: What else could it have been doing instead? A: {}"
          .format(what_answer, why_answer, how_well_answer, what_else_answer))


def filter_intentions(current_beliefs, current_desires, current_intentions):
    """
    Choose between competing alternatives and COMMITTING to achieving them.
    :param current_beliefs: WordModel instance, the world model, information about the world.
    :param current_desires: List of tuples, tasks that the agent would like to accomplish.
    :param current_intentions: List of tuples, tasks that the agent has COMMITTED to accomplish.
    :return: List of tuples, filtered intentions that the agent COMMITS to accomplish.
    """
    for current_intention in current_intentions:
        if current_intention == ('transfer_target_object_to_container', 'arm', 'target_object', 'table', 'container') \
                and current_beliefs.current_world_model.location["target_object"] == "container":
            current_intentions = ""  # if goal(s) achieved, empty I
    return current_intentions


def deliberate(current_beliefs, current_intentions):
    """
    Decide WHAT sate of affairs you want to achieve.
    :param current_beliefs: WordModel instance, the world model, information about the world.
    :param current_intentions: List of tuples, tasks that the agent has COMMITTED to accomplish.
    :return: List of tuples, filtered intentions that the agent COMMITS to accomplish.
    """
    # 1. Option Generation: The agent generates a set of possible alternatives.
    current_desires = current_intentions  # TODO: D := option(B, I);

    # 2. Filtering: Agent chooses between competing alternatives and commits to achieving them.
    current_intentions = filter_intentions(
        current_beliefs, current_desires, current_intentions)  # I := filter(B, D, I);

    return current_intentions


if __name__ == '__main__':

    terminate = False
    SUCCESS = False
    verbose = False

    # Initialization
    htn_planner = HierarchicalTaskNetworkPlanner()
    goal = [('transfer_target_object_to_container', 'arm', 'target_object', 'table', 'container')]
    intentions = goal  # I := I0; Initial Intentions
    beliefs = WorldModel()  # B := B0; Initial Beliefs
    monitoring = Monitoring()
    perception = Perception()
    coordination = Coordination()
    # Disable all 3 coordination switches for testing
    coordination.control.send_requests = True
    coordination.control.center_init = False
    coordination.control.detect_last_position = True

    what, why, how_well, what_else, why_failed = "", "", "", "", ""
    start_time = datetime.datetime.now()

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

    # while true do
    while not SUCCESS and not terminate and beliefs.update_tick() < beliefs.current_world_model.max_ticks:

        # TODO: engrave figures: arm IK, gripper
        percept = perception.get_percept(text_engraving=(why_failed, how_well))  # get next percept ρ; OBSERVE the world
        beliefs = perception.belief_revision(beliefs, percept)  # B:= brf(B, ρ);
        beliefs = monitoring.fire_events(beliefs, percept)

        intentions = deliberate(beliefs, intentions)  # DELIBERATE about what INTENTION to achieve next
        SUCCESS = True if intentions == "" else False

        plans = htn_planner.get_plans(beliefs.current_world_model, intentions)  # π := plan(B, I); MEANS_END REASONING
        if plans != False:
            if len(plans) > 0:
                plans.sort(key=len)  # TODO: check why sorting doesn't work on "deeper" levels
                if verbose:
                    print("{}: Plan: {}".format(beliefs.current_world_model.tick, plans[0]))
                selected_plan = deque(plans[0])  # TODO: Use a "cost function" to evaluate the best plan, not shortest
                why_failed = ""

                # while not (empty(π) or succeeded(Ι, Β) or impossible(I, B)) do
                while len(selected_plan) > 0 and beliefs.update_tick() < beliefs.current_world_model.max_ticks:
                    action, selected_plan = selected_plan.popleft(), selected_plan  # α := hd(π); π := tail(π);

                    if verbose:
                        print("{}: Action: {}".format(beliefs.current_world_model.tick, action))
                    coordination.execute_action(action, beliefs.current_world_model)  # execute(α);

                    what, why = action, intentions
                    how_well = (beliefs.current_world_model.tick, beliefs.current_world_model.max_ticks,
                                int((datetime.datetime.now() - start_time).total_seconds() * 1000),  # milliseconds
                                selected_plan)
                    what_else = plans[1] if len(plans) > 1 else plans[0]
                    if verbose:
                        print_answers(what, why, how_well, what_else)

                    # get next percept ρ; OBSERVE the world
                    percept = perception.get_percept(text_engraving=(what, why, how_well, what_else))
                    beliefs = perception.belief_revision(beliefs, percept)
                    beliefs = monitoring.fire_events(beliefs, percept)

                    # TODO: trigger sound percept?

                    if action == ('initialize', 'arm'):
                        percept = {"initialized": {'arm': True}}  # TODO: post conditions or monitoring
                        beliefs = perception.belief_revision(beliefs, percept)  # TODO: post conditions
                        beliefs = monitoring.fire_events(beliefs, percept)
                    elif action == ('move_arm', 'target_object'):
                        current_percept = {"distance": {'distance_to_gripper': 2.2}}  # TODO: update with monitoring
                        beliefs = perception.belief_revision(beliefs, percept)
                        beliefs = monitoring.fire_events(beliefs, percept)
                    elif action == ('move_arm_above', 'container'):
                        percept = {"location": {"target_object": "container"}, "grabbed": {'target_object': False}}
                        beliefs = perception.belief_revision(beliefs, percept)
                        beliefs = monitoring.fire_events(beliefs, percept)

                    # if reconsider(I, B) then
                    #   D := options(B, I);
                    #   I := filter(B, D, I);

                    # if not sound(π, I, B) then
                    #   π := plan(B, I)
        else:
            why_failed = htn_planner.failure_reason
            if verbose:
                print("Plan failure_reason: {}".format(why_failed), end=" ")
            how_well = (beliefs.current_world_model.tick, beliefs.current_world_model.max_ticks,
                        int((datetime.datetime.now() - start_time).total_seconds() * 1000),  # milliseconds
                        plans)
            if verbose:
                print("how_well: {}".format(how_well))

    print("Done.")
    perception.destroy()

    show_history = True
    print("Final World model:")
    print("-- Ticks: {}".format(beliefs.current_world_model.tick))
    print("-- initialized: {}".format(beliefs.current_world_model.initialized))
    print("-- location: {}".format(beliefs.current_world_model.location))
    print("-- grabbed: {}".format(beliefs.current_world_model.grabbed))
    if show_history:
        print()
        print("World model History:")
        for tick in range(len(beliefs.world_model_history)):
            print("Tick {}:".format(tick))
            print("-- initialized: {}".format(beliefs.world_model_history[tick].initialized))
            print("-- location: {}".format(beliefs.world_model_history[tick].location))
            print("-- grabbed: {}".format(beliefs.world_model_history[tick].grabbed))

    print()
    print("{}!".format("SUCCESS" if SUCCESS else "FAIL"))
