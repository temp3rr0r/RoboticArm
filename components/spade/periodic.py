import time
import datetime
from spade.agent import Agent
from spade.behaviour import CyclicBehaviour, PeriodicBehaviour
from spade.message import Message


class PeriodicSenderAgent(Agent):
    class InformBehav(PeriodicBehaviour):
        async def run(self):
            print(f"PeriodicSenderBehaviour running at {datetime.datetime.now().time()}: {self.counter}")
            # msg = Message(to="receiver@your_xmpp_server")  # Instantiate the message
            msg = Message(to="admin@Temp3rr0r-pc")     # Instantiate the message
            msg.body = "Hello World"  # Set the message content

            await self.send(msg)
            print("Message sent!")

            if self.counter == 5:
                self.kill()
            self.counter += 1

        async def on_end(self):
            # stop agent from behaviour
            await self.agent.stop()

        async def on_start(self):
            self.counter = 0

    async def setup(self):
        print(f"PeriodicSenderAgent started at {datetime.datetime.now().time()}")
        start_at = datetime.datetime.now() + datetime.timedelta(seconds=5)
        b = self.InformBehav(period=2, start_at=start_at)
        self.add_behaviour(b)


class ReceiverAgent(Agent):
    class RecvBehav(CyclicBehaviour):
        async def run(self):
            print("RecvBehav running")
            msg = await self.receive(timeout=10)  # wait for a message for 10 seconds
            if msg:
                print("Message received with content: {}".format(msg.body))
            else:
                print("Did not received any message after 10 seconds")
                self.kill()

        async def on_end(self):
            await self.agent.stop()

    async def setup(self):
        print("ReceiverAgent started")
        b = self.RecvBehav()
        self.add_behaviour(b)


if __name__ == "__main__":
    # receiveragent = ReceiverAgent("receiver@your_xmpp_server", "receiver_password")
    receiveragent = ReceiverAgent("admin@Temp3rr0r-pc", "ma121284")
    future = receiveragent.start()
    future.result() # wait for receiver agent to be prepared.
    # senderagent = PeriodicSenderAgent("sender@your_xmpp_server", "sender_password")
    senderagent = PeriodicSenderAgent("madks@Temp3rr0r-pc", "ma121284")
    senderagent.start()

    while receiveragent.is_alive():
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            senderagent.stop()
            receiveragent.stop()
            break
    print("Agents finished")