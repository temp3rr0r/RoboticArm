import time
from spade.agent import Agent
from spade.behaviour import OneShotBehaviour
from spade.message import Message
from spade.template import Template

print("1 -- Templates")

template = Template()
# template.sender = "sender1@host"
template.sender = "madks@Temp3rr0r-pc"
# template.to = "recv1@host"
template.to = "admin@Temp3rr0r-pc"
template.body = "Hello World"
template.thread = "thread-id"
template.metadata = {"performative": "query"}

message = Message()
# message.sender = "sender1@host"
message.sender = "madks@Temp3rr0r-pc"
# message.to = "recv1@host"
message.to = "admin@Temp3rr0r-pc"
message.body = "Hello World"
message.thread = "thread-id"
message.set_metadata("performative", "query")

assert template.match(message)


t1 = Template()
# t1.sender = "sender1@host"
t1.sender = "madks@Temp3rr0r-pc"
t2 = Template()
# t2.to = "recv1@host"
t2.to = "admin@Temp3rr0r-pc"
t2.metadata = {"performative": "query"}

m = Message()
# m.sender = "sender1@host"
m.sender = "madks@Temp3rr0r-pc"
# m.to = "recv1@host"
m.to = "admin@Temp3rr0r-pc"
m.metadata = {"performative": "query"}

# And AND operator
assert (t1 & t2).match(m)

t3 = Template()
# t3.sender = "not_valid_sender@host"
t3.sender = "not_valid_sender@Temp3rr0r-pc"

# A NOT complement operator
assert (~t3).match(m)
