from gremlin_python import statics
import os
from gremlin_python.process.anonymous_traversal import traversal
from gremlin_python.driver.driver_remote_connection import DriverRemoteConnection
from gremlin_python.process.traversal import Order
from gremlin_python.process.traversal import P
from gremlin_python.process.traversal import Bindings

g = traversal().withRemote(DriverRemoteConnection('ws://localhost:8182/gremlin', 'g'))
statics.load_statics(globals())

v1 = g.addV('model').property('name', 'marko').next()
v2 = g.addV('person').property('name', 'stephen').next()

print("count: ", g.V(v2).properties('name').count())

l = g.V(Bindings.of('id',v1)).addE('knows').to(v2).property('weight',0.75).iterate()
print(l)

marko = g.V().has('person','name','marko').next()
peopleMarkoKnows = g.V().has('person','name','marko').out('knows').toList()

print("marko: ", marko)
print("peopleMarkoKnows: ", peopleMarkoKnows)

print("{}".format(g.V(v1).properties()))



current_work_directory = os.getcwd()
k = g.io(current_work_directory + "/" + "world_model.xml").write().iterate()
k = g.io(current_work_directory + "/" + "world_model.json").write().iterate()
