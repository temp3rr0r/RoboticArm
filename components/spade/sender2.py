import time
from spade.agent import Agent
from spade.behaviour import OneShotBehaviour
from spade.message import Message
from spade.template import Template


class SenderAgent(Agent):
    class InformBehav(OneShotBehaviour):
        async def run(self):
            print("InformBehav running")
            # msg = Message(to="receiver@your_xmpp_server")     # Instantiate the message
            msg = Message(to="admin@Temp3rr0r-pc")     # Instantiate the message
            msg.set_metadata("performative", "inform")  # Set the "inform" FIPA performative
            msg.body = "Hello World"                    # Set the message content

            await self.send(msg)
            print("Message sent!")

            # stop agent from behaviour
            await self.agent.stop()

    async def setup(self):
        print("SenderAgent started")
        b = self.InformBehav()
        self.add_behaviour(b)

class ReceiverAgent(Agent):
    class RecvBehav(OneShotBehaviour):
        async def run(self):
            print("RecvBehav running")

            msg = await self.receive(timeout=10) # wait for a message for 10 seconds
            if msg:
                print("Message received with content: {}".format(msg.body))
            else:
                print("Did not received any message after 10 seconds")

            # stop agent from behaviour
            await self.agent.stop()

    async def setup(self):
        print("ReceiverAgent started")
        b = self.RecvBehav()
        template = Template()
        template.set_metadata("performative", "inform")
        self.add_behaviour(b, template)



if __name__ == "__main__":
    # receiveragent = ReceiverAgent("receiver@your_xmpp_server", "receiver_password")
    receiveragent = ReceiverAgent("admin@Temp3rr0r-pc", "ma121284")
    future = receiveragent.start()
    future.result() # wait for receiver agent to be prepared.
    # senderagent = SenderAgent("sender@your_xmpp_server", "sender_password")
    senderagent = SenderAgent("madks@Temp3rr0r-pc", "ma121284")
    senderagent.start()

    while receiveragent.is_alive():
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            senderagent.stop()
            receiveragent.stop()
            break
    print("Agents finished")