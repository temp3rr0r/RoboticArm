from world_model import WorldModel
from hierarchical_task_network_planner import HierarchicalTaskNetworkPlanner
from collections import deque

max_ticks = 20  # TODO: set it in the property graph
terminate = False

beliefs = WorldModel()  # B = B0 # TODO: Initial Beliefs
goal = [('transfer_ball_to_container', 'arm', 'ball', 'table', 'container')]  # I = I0 # TODO: Initial Beliefs
I = goal
htn_planner = HierarchicalTaskNetworkPlanner()


def deliberate(current_beliefs, current_intention):
    # if goal achieved from world model, don't desire/intent it any more
    if current_intention == [('transfer_ball_to_container', 'arm', 'ball', 'table', 'container')] \
            and current_beliefs.current_world_model.location["ball"] == "container":  # I = I0 # TODO: Initial Beliefs
        print("ok3")
        current_intention = ""
    return current_intention


while not terminate and beliefs.update_tick() < max_ticks:

    percept = ""
    # get next percept ρ # TODO: OBSERVE the world
    # if beliefs.current_world_model.tick == 1:  # TODO: for testing
    #     print("Ok1")
    #     percept = {"initialized": {'arm': True}}
    #     beliefs = beliefs.belief_revision(percept)

    # TODO: deliberation: if goal achieved, empty I
    I = deliberate(beliefs, I)
    # I = deliberate(B) # TODO: DELIBERATE about what INTENTION to achieve next
        # 1. Option Generation # TODO: The agent generates a set of possible alternatives.
        # 2. Filtering # TODO: Agent choses between competing alternatives and commits to achieving them.

    plans = htn_planner.get_plans(beliefs.current_world_model, I)  # π = plan(B, I) # TODO: use MEANS_END REASONING to get a PLAN for the intention
    if len(plans) > 0:
        plans.sort(key=len)  # TODO: check why sorting doesn't work on "deeper" levels
        print("{}: Plan: {}".format(beliefs.current_world_model.tick, plans[0]))
        selected_plan = deque(plans[0])

        while len(selected_plan) > 0 and beliefs.update_tick() < max_ticks:
            action, selected_plan = selected_plan.popleft(), selected_plan

            # execute(action)  # TODO: action = hd(π); π = tail(π);
            print("{}: Action: {}".format(beliefs.current_world_model.tick, action))

            # get next percept ρ # TODO: OBSERVE the world
            if action == ('put', 'arm', 'ball', 'container'):
                percept = {"location": {"ball": "container"}}
                beliefs = beliefs.belief_revision(percept)
                percept = {"grabbed": {'ball': False}}
                beliefs = beliefs.belief_revision(percept)
            elif action == ('grab', 'arm', 'ball', 'table'):
                percept = {"grabbed": {'ball': True}, "initialized": {'arm': False}}
                beliefs = beliefs.belief_revision(percept)
            elif action == ('initialize', 'arm'):
                percept = {"initialized": {'arm': True}}
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

