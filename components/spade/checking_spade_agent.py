from spade import agent

# print("1 -- Dummy agent")
#
# class DummyAgent(agent.Agent):
#     async def setup(self):
#         print("Hello World! I'm agent {}".format(str(self.jid)))
#
#
# # dummy = DummyAgent("madks@chat.sum7.eu", "ma121284")
# dummy = DummyAgent("madks@Temp3rr0r-pc", "ma121284")
# dummy.start()
# dummy.stop()

print("2 -- Agent with behaviour")

import time
import asyncio
from spade.agent import Agent
from spade.behaviour import CyclicBehaviour

class DummyAgent(Agent):
    class MyBehav(CyclicBehaviour):
        async def on_start(self):
            print("Starting behaviour . . .")
            self.counter = 0

        async def run(self):
            print("Counter: {}".format(self.counter))
            self.counter += 1
            await asyncio.sleep(1)

    async def setup(self):
        print("Agent starting . . .")
        b = self.MyBehav()
        self.add_behaviour(b)

# dummy = DummyAgent("madks@madks-pc", "ma121284")
dummy = DummyAgent("madks@Temp3rr0r-pc", "ma121284")
dummy.start()
dummy.web.start(hostname="127.0.0.1", port="10000")

print("Wait until user interrupts with ctrl+C")
while True:
    try:
        time.sleep(1)
    except KeyboardInterrupt:
        break
dummy.stop()

# print("3 -- Finishing a behaviour")
#
# import time
# import asyncio
# from spade.agent import Agent
# from spade.behaviour import CyclicBehaviour
#
# class DummyAgent(Agent):
#     async def setup(self):
#         self.my_behav = self.MyBehav()
#         self.add_behaviour(self.my_behav)
#         print("Agent starting . . .")
#
#     class MyBehav(CyclicBehaviour):
#         async def on_start(self):
#             print("Starting behaviour . . .")
#             self.counter = 0
#
#         async def run(self):
#             print("Counter: {}".format(self.counter))
#             self.counter += 1
#             if self.counter >= 3:
#                 self.kill(exit_code=10)
#                 return
#             await asyncio.sleep(1)
#
#         async def on_end(self):
#             print("Behaviour finished with exit code {}.".format(self.exit_code))
#
#
# if __name__ == "__main__":
# # dummy = DummyAgent("your_jid@your_xmpp_server", "your_password")
#     dummy = DummyAgent("madks@Temp3rr0r-pc", "ma121284")
#     dummy.start()
#
#     # wait until user interrupts with ctrl+C
#     while not dummy.my_behav.is_killed():
#         try:
#             time.sleep(1)
#         except KeyboardInterrupt:
#             break
#     dummy.stop()

# print("4 -- Creating an agent from within another agent")
#
# import time
# import asyncio
# from spade.agent import Agent
# from spade.behaviour import CyclicBehaviour, OneShotBehaviour
#
# class CreateBehav(OneShotBehaviour):
#     async def run(self):
#         # agent2 = Agent("agent2@fake_server", "fake_password")
#         agent2 = Agent("madks@Temp3rr0r-pc", "ma121284")
#         # This start is inside an async def, so it must be awaited
#         await agent2.start(auto_register=False)
#
# # agent1 = Agent("agent1@fake_server", "fake_password")
# agent1 = Agent("admin@Temp3rr0r-pc", "ma121284")
# agent1.add_behaviour(CreateBehav())
# # This start is in a synchronous piece of code, so it must NOT be awaited
# agent1.start(auto_register=False)