from world_model import WorldModel
from hierarchical_task_network_planner import HierarchicalTaskNetworkPlanner
from collections import deque

max_ticks = 20  # TODO: set it in the property graph
terminate = False
htn_planner = HierarchicalTaskNetworkPlanner()

goal = [('transfer_ball_to_container', 'arm', 'ball', 'table', 'container')]
intentions = goal  # I = I0 Initial Intentions
beliefs = WorldModel()  # B = B0 Initial Beliefs


def filter_intentions(current_beliefs, current_desires, current_intentions):
    for current_intention in current_intentions:
        if current_intention == ('transfer_ball_to_container', 'arm', 'ball', 'table', 'container') \
                and current_beliefs.current_world_model.location["ball"] == "container":
            current_intentions = ""  # if goal(s) achieved, empty I
    return current_intentions


def deliberate(current_beliefs, current_intentions):

    current_desires = current_intentions  # 1. Option Generation: The agent generates a set of possible alternatives.

    # 2. Filtering: Agent chooses between competing alternatives and commits to achieving them.
    current_intentions = filter_intentions(current_beliefs, current_desires, current_intentions)

    return current_intentions


while not terminate and beliefs.update_tick() < max_ticks:

    percept = ""
    # get next percept ρ # TODO: OBSERVE the world

    intentions = deliberate(beliefs, intentions)  # DELIBERATE about what INTENTION to achieve next

    # π = plan(B, I) MEANS_END REASONING to PLAN for the intention
    plans = htn_planner.get_plans(beliefs.current_world_model, intentions)
    if len(plans) > 0:
        plans.sort(key=len)  # TODO: check why sorting doesn't work on "deeper" levels
        print("{}: Plan: {}".format(beliefs.current_world_model.tick, plans[0]))
        selected_plan = deque(plans[0])

        while len(selected_plan) > 0 and beliefs.update_tick() < max_ticks:
            action, selected_plan = selected_plan.popleft(), selected_plan  # action = hd(π); π = tail(π);

            print("{}: Action: {}".format(beliefs.current_world_model.tick, action))
            # execute(action)  # TODO: do execute the action

            # get next percept ρ # TODO: OBSERVE the world
            if action == ('initialize', 'arm'):  # TODO: for testing only
                percept = {"initialized": {'arm': True}}
                beliefs = beliefs.belief_revision(percept)
            elif action == ('grab', 'arm', 'ball', 'table'):
                percept = {"grabbed": {'ball': True}, "initialized": {'arm': False}}
                beliefs = beliefs.belief_revision(percept)
            elif action == ('put', 'arm', 'ball', 'container'):
                percept = {"location": {"ball": "container"}}
                beliefs = beliefs.belief_revision(percept)
                percept = {"grabbed": {'ball': False}}
                beliefs = beliefs.belief_revision(percept)

            # if not sound(π, I, B) then
            #   π = plan(B, I)


print("Done.")
print("Final World model:")
print("-- Ticks: {}".format(beliefs.current_world_model.tick))
print("-- initialized: {}".format(beliefs.current_world_model.initialized))
print("-- location: {}".format(beliefs.current_world_model.location))
print("-- grabbed: {}".format(beliefs.current_world_model.grabbed))
print()
print("World model History:")
for tick in range(len(beliefs.world_model_history)):
    print("Tick {}:".format(tick))
    print("-- initialized: {}".format(beliefs.world_model_history[tick].initialized))
    print("-- location: {}".format(beliefs.world_model_history[tick].location))
    print("-- grabbed: {}".format(beliefs.world_model_history[tick].grabbed))
