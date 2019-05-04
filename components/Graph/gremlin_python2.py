from gremlin_python import statics
import os
from gremlin_python.process.anonymous_traversal import traversal
from gremlin_python.driver.driver_remote_connection import DriverRemoteConnection
from gremlin_python.process.traversal import Bindings

current_work_directory = os.getcwd()
g = traversal().withRemote(DriverRemoteConnection('ws://localhost:8182/gremlin', 'g'))
statics.load_statics(globals())

load_graph = False
if load_graph:
    g.io(current_work_directory + "/" + "world_model.xml").read().iterate()

create_graph = False
if create_graph:
    # {
    #     "variables": {
    #         "servo1": 1500,
    #         "servo2": 600,
    #         "servo3": 1665,
    #         "servo4": 1656,
    #         "servo5": 909,
    #         "servo6": 500,
    #         "servo_limit_min": 500,
    #         "servo_limit_max": 2500,
    #         "servo_speed": 150,
    #         "servo_delay_after": 5
    #     },
    #     "id": "33",
    #     "name": "le_arm_esp32",
    #     "hardware": "esp32",
    #     "connected": true
    # }
    v_esp32 = g.addV("esp32").property(id, 3).property("name", "le_arm_esp32").property("hardware", "esp32").property("connected", True).next()
    v_variables = g.addV("variables").property(id, 4).property("servo1", 1500).property("servo2", 1500).property("servo3", 1500).property("servo4", 1500).property("servo5", 1500).property("servo6", 1500).property("servo_limit_min", 500).property("servo_limit_max", 2500).property("servo_speed", 150).property("servo_delay_after", 5).next()
    l = g.V(Bindings.of('id', v_esp32)).addE('stores').to(v_variables).iterate()

    # send_requests = False
    # command_delay = 0.05  # seconds
    # center_init = True
    # angle_degree_limit = 75  # degrees
    # trajectory_steps = 10
    # current_servo_monotony = [-1.0, -1.0, 1.0, -1.0, -1.0, -1.0]
    # active_links_mask = [True, True, True, True, False, True]  # Enabled/disabled links
    # gripper_servo = 2
    # gripper_open = 600
    # target_position = [-20, -20, 25]
    # init_position = [0, 0, 1]
    # min_steps = 1
    # max_steps = 5000
    # detect_last_position = False
    # init_servo_values = [1500, 1500, 1500, 1500, 1500, 1500]
    # url = "http://ESP_02662E/"
    v_kinematic_model = g.addV("model").property(id, 6).property("name", "kinematic").property("scale", 0.04)\
        .property("servo_count", 6) \
        .property("send_requests", False) \
        .property("command_delay", 0.05) \
        .property("center_init", True) \
        .property("angle_degree_limit", 75) \
        .property("trajectory_steps", 10) \
        .property("current_servo_monotony", [-1.0, -1.0, 1.0, -1.0, -1.0, -1.0]) \
        .property("active_links_mask", [True, True, True, True, False, True]) \
        .property("gripper_servo", 2) \
        .property("gripper_open", 600) \
        .property("target_position", [-20, -20, 25])\
        .property("init_position", [0, 0, 1]).property("min_steps", 1).property("max_steps", 5000)\
        .property("detect_last_position", False).property("init_servo_values", [1500, 1500, 1500, 1500, 1500, 1500])\
        .property("url", "http://ESP_02662E/")\
        .next()

    # le_arm_chain = Chain(name='le_arm')
    v_chain = g.addV("chain").property("name", "le_arm_chain").next()
    l2 = g.V(Bindings.of('id', v_kinematic_model)).addE('simulates').to(v_chain).iterate()

    # # Link lengths in centimeters  # TODO: add units of measurement
    # link6 = [0, 0, 7.0]
    # link5 = [0, 0, 3.0]
    # link4 = [0, 0, 10.5]
    # link3 = [0, 0, 9.0]
    # link2 = [0, 0, 7.0]
    # link1 = [0, 0, 10.0]
    # # Joint rotation axis
    # rotation6 = [0, 0, 1]
    # rotation5 = [0, 1, 0]
    # rotation4 = [0, 1, 0]
    # rotation3 = [0, 1, 0]
    # rotation2 = [0, 0, 1]
    # rotation1 = [0, 0, 1]
    # # Link bounds (degrees
    # bounds6 = [-75, 75]
    # bounds5 = [-75, 75]
    # bounds4 = [-75, 75]
    # bounds3 = [-75, 75]
    # bounds2 = [-75, 75]
    # bounds1 = [-75, 75]
    vlink6 = g.addV("link").property("name", "link6").property("type", "URDFLink")\
        .property("translation_vector", [0, 0, 7.0]).property("orientation", [0, 0, 0])\
        .property("rotation", [0, 0, 1]).property("bounds", [-75, 75]).next()
    vlink5 = g.addV("link").property("name", "link5").property("type", "URDFLink")\
        .property("translation_vector", [0, 0, 3.0]).property("orientation", [0, 0, 0])\
        .property("rotation", [0, 1, 0]).property("bounds", [-75, 75]).next()
    vlink4 = g.addV("link").property("name", "link4").property("type", "URDFLink")\
        .property("translation_vector", [0, 0, 10.5]).property("orientation", [0, 0, 0])\
        .property("rotation", [0, 1, 0]).property("bounds", [-75, 75]).next()
    vlink3 = g.addV("link").property("name", "link3").property("type", "URDFLink")\
        .property("translation_vector", [0, 0, 9.0]).property("orientation", [0, 0, 0])\
        .property("rotation", [0, 1, 0]).property("bounds", [-75, 75]).next()
    vlink2 = g.addV("link").property("name", "link2").property("type", "URDFLink")\
        .property("translation_vector", [0, 0, 7.0]).property("orientation", [0, 0, 0])\
        .property("rotation", [0, 0, 1]).property("bounds", [-75, 75]).next()
    vlink1 = g.addV("link").property("name", "link1").property("type", "URDFLink")\
        .property("translation_vector", [0, 0, 10.0]).property("orientation", [0, 0, 0])\
        .property("rotation", [0, 0, 1]).property("bounds", [-75, 75]).next()
    l_tolink6 = g.V(Bindings.of('id', v_chain)).addE('connects_to').to(vlink6).iterate()
    l_tolink5 = g.V(Bindings.of('id', vlink6)).addE('connects_to').to(vlink5).iterate()
    l_tolink4 = g.V(Bindings.of('id', vlink5)).addE('connects_to').to(vlink4).iterate()
    l_tolink3 = g.V(Bindings.of('id', vlink4)).addE('connects_to').to(vlink3).iterate()
    l_tolink2 = g.V(Bindings.of('id', vlink3)).addE('connects_to').to(vlink2).iterate()
    l_tolink1 = g.V(Bindings.of('id', vlink2)).addE('connects_to').to(vlink1).iterate()

    # world model
    # current_world_model = pyhop.State('current_world_model')
    # current_world_model.iteration = 0
    # current_world_model.timestamp = time.time()
    # current_world_model.location = {'ball': 'table'}
    # current_world_model.grabbed = {'ball': True}
    # current_world_model.initialized = {'arm': True}
    # current_world_model.min_bounds = {'xyz': [-25, -25, -25]}
    # current_world_model.max_bounds = {'xyz': [25, 25, 25]}
    # current_world_model.plans = "None"
    v_world_model = g.addV("model").property(id, 9)\
        .property("type", "world_model")\
        .property("name", "current_world_model")\
        .property("State", "current_world_model")\
        .property("iteration", 0)\
        .property("timestamp", 1556504401)\
        .property("location", {"ball": "table"})\
        .property("grabbed", {"ball": True}).property("initialized", {"arm": True})\
        .property("min_bounds", {"xyz": [-25, -25, -25]})\
        .property("max_bounds", {"xyz": [25, 25, 25]})\
        .property("plans", "None").next()

    # Planner
    # pyhop.failure_reason = "can't get xyz of {}".format(actee)
    # htn_plans = pyhop.pyhop(current_world_model, [('transfer_ball_to_container', 'arm', 'ball', 'table',
    # 'container')], verbose=1, all_plans=True, sort_asc=True)
    v_planner = g.addV("planner").property(id, 10) \
        .property("type", "Hierarchical Task Network") \
        .property("name", "pyhop") \
        .property("failure_reason", "") \
        .property("world_model", "current_world_model") \
        .property("goal", [('transfer_ball_to_container', 'arm', 'ball', 'table', 'container')], ) \
        .property("verbose", 1) \
        .property("all_plans", True) \
        .property("sort_asc", True) \
        .next()
    l_planner_world_model = g.V(Bindings.of('id', v_planner)).addE('uses').to(v_world_model).iterate()

    # pyhop.declare_operators(initialize, grab, put, move_arm, close_hand, open_hand)
    v_declared_operators = g.addV("declare").property("type", "operators")\
        .property("operators", ["initialize", "grab", "put", "move_arm", "close_hand", "open_hand"])\
        .next()
    l_planner_v_declared_operators = g.V(Bindings.of('id', v_planner)).addE('declares').to(
        v_declared_operators).iterate()

    # pyhop.declare_methods('transfer_ball_to_container', initialize_transfer, put_grabbed, transfer)
    v_declared_methods = g.addV("declare").property("type", "methods")\
        .property("task", "transfer_ball_to_container")\
        .property("methods", ["initialize_transfer", "put_grabbed", "transfer"])\
        .next()
    l_planner_v_declared_methods = g.V(Bindings.of('id', v_planner)).addE('declares').to(v_declared_methods).iterate()

    # # TODO: tasks and stuff
    # # helper_methods
    # def get_xyz(actee):
    # is_within_bounds(location1, min_bounds, max_bounds):
    # get_diameter(actee):
    # center_servos(state):
    # # Operators (primitive tasks)
    # def move_arm(state, to_):
    # def close_hand(distance):
    # def open_hand(distance):
    # def initialize(state, actor):
    # def grab(state, actor, actee, from_):
    # def put(state, actor, actee, to_):
    # # Methods (compound tasks)
    # def put_grabbed(state, actor, actee, from_, to_):
    # def intialize_transfer(state, actor, actee, from_, to_):
    # def transfer(state, actor, actee, from_, to_):

    # Store to disk
    g.io(current_work_directory + "/" + "world_model.xml").write().iterate()
    g.io(current_work_directory + "/" + "world_model.json").write().iterate()


print("servo_limit_min: {}".format(g.V().has("hardware", "esp32").out().values("servo_limit_min").next()))
print("servo_limit_max: {}".format(g.V().has("hardware", "esp32").has("name", "le_arm_esp32").out().values("servo_limit_max").next()))

servo_values = g.V().has("name", "le_arm_esp32").out().values('servo1', 'servo2', 'servo3', 'servo4', 'servo5',
                                                                'servo6').toList()
print("servos: {}".format(servo_values))

g.V().has("name", "le_arm_esp32").out().property("servo_speed", 150).next()  # Write data
servo_speed = g.V().has("name", "le_arm_esp32").out().values("servo_speed").next()  # Read data
print("servo_speed: {}".format(servo_speed))

servo_count = g.V().has("name", "kinematic").values("servo_count").next()  # Read data
print("servo_count: {}".format(servo_count))

servo_count = g.V().has("name", "kinematic").values("servo_count").next()  # Read data
print("servo_count: {}".format(servo_count))

print("url: {}".format(g.V().has("name", "kinematic").values("url").next()))
print("link3 translation_vector: {}".format(g.V().has("name", "link3").values("translation_vector").next()))

print("world_model min_bounds: {}".format(g.V().has("name", "current_world_model").values("min_bounds").next()))
print("pyhop goal: {}".format(g.V().has("name", "pyhop").values("goal").next()))
print("task transfer_ball_to_container: {}".format(g.V().has(
    "type", "methods").has("task", "transfer_ball_to_container").values("methods").next()))
