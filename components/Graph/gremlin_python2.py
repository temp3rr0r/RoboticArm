from gremlin_python import statics
import os
from gremlin_python.process.anonymous_traversal import traversal
from gremlin_python.driver.driver_remote_connection import DriverRemoteConnection
from gremlin_python.process.traversal import Order
from gremlin_python.process.traversal import P
from gremlin_python.process.traversal import Bindings

current_work_directory = os.getcwd()
g = traversal().withRemote(DriverRemoteConnection('ws://localhost:8182/gremlin', 'g'))
statics.load_statics(globals())

loaded_graph = True
if not loaded_graph:
    g.io(current_work_directory + "/" + "world_model.xml").read().iterate()

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
#     "name": "le_arm_esp8266",
#     "hardware": "esp8266",
#     "connected": true
# }

create_graph = False
if create_graph:
    v0 = g.addV("esp8266").property(id, 3).property("name", "le_arm_esp8266").property("hardware", "esp8266").property("connected", True).next()
    v1 = g.addV("variables").property(id, 4).property("servo1", 1500).property("servo2", 1500).property("servo3", 1500).property("servo4", 1500).property("servo5", 1500).property("servo6", 1500).property("servo_limit_min", 500).property("servo_limit_max", 2500).property("servo_speed", 150).property("servo_delay_after", 5).next()
    l = g.V(Bindings.of('id', v0)).addE('stores').to(v1).iterate()

store_graph = False
if store_graph:
    g.io(current_work_directory + "/" + "world_model.xml").write().iterate()
    g.io(current_work_directory + "/" + "world_model.json").write().iterate()


print("servo1: {}".format(g.V().has("hardware", "esp8266").out().values("servo1").next()))
print("servo_limit_min: {}".format(g.V().has("hardware", "esp8266").out().values("servo_limit_min").next()))
print("servo_limit_max: {}".format(g.V().has("hardware", "esp8266").has("name", "le_arm_esp8266").out().values("servo_limit_max").next()))

servo_values = g.V().has("name", "le_arm_esp8266").out().values('servo1', 'servo2', 'servo3', 'servo4', 'servo5',
                                                                'servo6').toList()
print("servos: {}".format(servo_values))

g.V().has("name", "le_arm_esp8266").out().property("servo_speed", 150).next()  # Write data
servo_speed = g.V().has("name", "le_arm_esp8266").out().values("servo_speed").next()  # Read data
print("servo_speed: {}".format(servo_speed))


