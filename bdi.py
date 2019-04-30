from world_model import WorldModel

max_ticks = 100  # TODO: set it in the property graph
terminate = False

beliefs = WorldModel()  # B = B0 # TODO: Initial Beliefs

while not terminate and beliefs.update_tick() < max_ticks:

    # get next percept ρ # TODO: OBSERVE the world
    if beliefs.current_world_model.tick == 55:
        percept = {"grabbed": {'ball': True}}
        beliefs = beliefs.belief_revision(percept)

    if beliefs.current_world_model.tick == 75:
        percept = {"grabbed": {'ballz': True}}
        beliefs = beliefs.belief_revision(percept)

    # I = deliberate(B) # TODO: DELIBERATE about what INTENTION to achieve next
        # 1. Option Generation # TODO: The agent generates a set of possible alternatives.
        # 2. Filtering # TODO: Agent choses between competing alternatives and commits to achieving them.
    # π = plan(B, I) # TODO: use MEANS_END REASONING to get a PLAN for the intention
    # execute(π) # TODO: EXECUTE the plan

print("Done.")

print("2 'a': {}".format(beliefs.current_world_model.grabbed))
print("0 'a': {}".format(beliefs.world_model_history[0].grabbed))
print("1 'a': {}".format(beliefs.world_model_history[1].grabbed))
