"""Every UDP message the simulator emits must be one the Unity client accepts.

The two sides drifted without anyone noticing: `udp.py` defined
transmit_order_create/transmit_order_complete that nothing ever called, and the
Unity client had no ORDERCREATE/ORDERCOMPLETE to receive them with, so orders --
the thing the schedulers exist to fulfil -- were invisible in the visualisation.

The schema below mirrors `viz/Assets/Scripts/SimulationCommand.cs`. Unity parses
with JsonUtility into typed fields and then runs VerifyCommand(), so a message
with a missing name or a string where an int belongs is dropped with nothing but
a console line. Keeping that schema asserted here is what stops the two sides
drifting apart again.
"""
import os
os.environ.setdefault("ROBOTSIM_TRANSMIT", "False")

import json
import socket
import threading
import unittest

from mrws.exceptions import SimulationError
from mrws.engine.warehouse import Warehouse
from mrws.io import transport

DATA_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data")
WHOUSE = os.path.join(DATA_DIR, "whouse.txt")

HOST = transport.HOST
PORT = transport.PORT

# --- mirror of SimulationCommand.cs -----------------------------------------
KNOWN_COMMANDS = {
    "START", "RESET", "WAREHOUSESIZE",
    "CREATEROBOT", "MOVEROBOT",
    "CREATESHELF", "CREATEGOAL",
    "ITEM", "ITEMGAINED", "ITEMLOST", "CLEARINV",
    "ORDERCREATE", "ORDERCOMPLETE",
    "ROBOTFAULT", "ROBOTRECOVERED",
}
NEEDS_POSITION = {"WAREHOUSESIZE", "CREATEROBOT", "CREATESHELF", "CREATEGOAL"}
NEEDS_OBJ_NAME = {"CREATEROBOT", "MOVEROBOT", "CREATESHELF", "CREATEGOAL",
                  "ITEMGAINED", "ITEMLOST", "CLEARINV",
                  "ORDERCREATE", "ORDERCOMPLETE", "ROBOTFAULT", "ROBOTRECOVERED"}
NEEDS_ITEM_NAME = {"CREATESHELF", "ITEM", "ITEMGAINED", "ITEMLOST", "ORDERCREATE"}
# JsonUtility deserialises into `public int`; a string there does not parse.
INT_FIELDS = ("posX", "posY", "prio")


def capture_simulation(whouse=WHOUSE, num_items=10, mode="multi-robot",
                       fault_rates=(0, 0, 0, 0), step_limit=2000, tolerate_errors=False):
    """Run one simulation and return every message it tried to send.

    This intercepts at `transport.send_message` rather than reading the socket,
    because the socket loses messages: the simulator bursts thousands of small
    datagrams with no flow control, and the kernel receive buffer caps out at
    net.core.rmem_max (208KB here), so a real capture drops a chunk of them.
    See TestTransportDelivery, which covers the wire itself. What is asserted
    here is the message stream the client is expected to understand.

    With `tolerate_errors`, a run that dies part-way still yields what it sent.
    A fault-heavy run is expected to end in a collision or an overrun, and the
    traffic up to that point is still traffic the client has to cope with.
    """
    sent = []
    original_send = transport.send_message
    transport.send_message = sent.append

    warehouse = None
    try:
        warehouse = Warehouse(whouse, num_items, 3, mode, list(fault_rates), True, step_limit)
        keep_stepping = True
        while keep_stepping:
            keep_stepping = not warehouse.step()
    except SimulationError:
        if not tolerate_errors:
            raise
    finally:
        transport.send_message = original_send

    return sent, warehouse


class UdpProtocolTestCase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.messages, cls.warehouse = capture_simulation()
        cls.decoded = [json.loads(raw) for raw in cls.messages]

    def commands_named(self, name):
        return [msg for msg in self.decoded if msg.get("command") == name]


class TestMessagesAreWellFormed(UdpProtocolTestCase):
    def test_a_simulation_emits_messages_at_all(self):
        self.assertGreater(len(self.messages), 0)

    def test_every_message_is_valid_json_with_a_known_command(self):
        for msg in self.decoded:
            self.assertIn("command", msg)
            self.assertIn(msg["command"], KNOWN_COMMANDS,
                          "Unity has no handler for %r" % msg["command"])

    def test_commands_carry_the_fields_the_client_requires(self):
        for msg in self.decoded:
            command = msg["command"]
            if command in NEEDS_POSITION:
                for field in ("posX", "posY"):
                    self.assertIn(field, msg, "%s needs %s" % (command, field))
                    # VerifyCommand rejects the -1 sentinel as "no position given".
                    self.assertNotEqual(msg[field], -1, "%s sent sentinel %s" % (command, field))
            if command in NEEDS_OBJ_NAME:
                self.assertTrue(msg.get("objName"), "%s needs objName" % command)
            if command in NEEDS_ITEM_NAME:
                self.assertTrue(msg.get("itemName"), "%s needs itemName" % command)

    def test_integer_fields_are_sent_as_integers(self):
        # JsonUtility maps these onto `public int`; a quoted number does not parse.
        for msg in self.decoded:
            for field in INT_FIELDS:
                if field in msg:
                    self.assertIsInstance(
                        msg[field], int,
                        "%s sent %s as %r" % (msg["command"], field, type(msg[field]).__name__))


class TestSessionLifecycle(UdpProtocolTestCase):
    def test_the_session_is_reset_before_it_starts(self):
        # The client ignores everything until START, and accumulates objects
        # across runs unless it is told to clear first.
        commands = [msg["command"] for msg in self.decoded]
        self.assertIn("RESET", commands)
        self.assertIn("START", commands)
        self.assertLess(commands.index("RESET"), commands.index("START"))

    def test_entities_are_created_before_they_are_referenced(self):
        created = set()
        for msg in self.decoded:
            command = msg["command"]
            if command in ("CREATEROBOT", "CREATESHELF", "CREATEGOAL"):
                created.add(msg["objName"])
            elif command == "ITEM":
                created.add(msg["itemName"])
            elif command == "MOVEROBOT":
                self.assertIn(msg["objName"], created,
                              "moved %s before creating it" % msg["objName"])
            elif command == "CREATESHELF":
                self.assertIn(msg["itemName"], created)


class TestOrdersAreVisible(UdpProtocolTestCase):
    def test_every_order_is_announced(self):
        announced = {msg["objName"] for msg in self.commands_named("ORDERCREATE")}
        order_manager = self.warehouse.get_order_manager()
        expected = {"order%s" % order_id for order_id in order_manager._all_orders}
        self.assertTrue(expected.issubset(announced),
                        "orders never announced to the client: %r" % (expected - announced))

    def test_an_order_carries_its_priority_and_items(self):
        for msg in self.commands_named("ORDERCREATE"):
            self.assertIn("prio", msg)
            self.assertGreaterEqual(msg["prio"], 1)
            self.assertTrue(msg["itemName"].startswith("item"))

    def test_completed_orders_are_announced(self):
        completed = self.warehouse.get_order_manager().get_order_finish_work_times()
        self.assertTrue(completed, "simulation completed no orders, nothing to assert")
        announced = {msg["objName"] for msg in self.commands_named("ORDERCOMPLETE")}
        for order_id in completed:
            self.assertIn("order%s" % order_id, announced)

    def test_an_order_is_not_completed_before_it_is_created(self):
        seen = set()
        for msg in self.decoded:
            if msg["command"] == "ORDERCREATE":
                seen.add(msg["objName"])
            elif msg["command"] == "ORDERCOMPLETE":
                self.assertIn(msg["objName"], seen)


class TestFaultsAreVisible(unittest.TestCase):
    def test_a_faulting_robot_is_announced_with_its_fault_type(self):
        # Faults certain enough that a short run produces some.
        messages, _ = capture_simulation(fault_rates=(0.0, 0.01, 0.01, 0.01),
                                         tolerate_errors=True)
        faults = [json.loads(m) for m in messages if '"ROBOTFAULT"' in m]
        self.assertTrue(faults, "no ROBOTFAULT emitted despite high fault rates")
        for msg in faults:
            self.assertTrue(msg["objName"].startswith("robot"))
            self.assertIn(msg["faultType"],
                          {"battery_critical", "battery_low", "actuator", "sensor"})


class TestTransportDelivery(unittest.TestCase):
    """The schema tests intercept before the socket; these cover the wire.

    The transport is TCP because UDP lost messages: the simulator outruns the
    receiver's buffer and the kernel silently drops the overflow.
    """

    def setUp(self):
        self.server = None
        self.lines = []
        self.reader = None
        transport.reset_connection()

    def tearDown(self):
        transport.reset_connection()
        if self.server is not None:
            self.server.close()
        os.environ["ROBOTSIM_TRANSMIT"] = "False"

    def start_listener(self):
        """Accept one connection and collect newline-framed messages."""
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.server.bind((HOST, PORT))
        except OSError:
            self.server.close()
            self.server = None
            raise unittest.SkipTest("TCP port %d in use (Unity running?)" % PORT)
        self.server.listen(1)

        def accept_and_read():
            try:
                conn, _ = self.server.accept()
            except OSError:
                return
            buffer = b""
            with conn:
                while True:
                    try:
                        chunk = conn.recv(65536)
                    except OSError:
                        return
                    if not chunk:
                        break
                    buffer += chunk
                    while b"\n" in buffer:
                        line, buffer = buffer.split(b"\n", 1)
                        self.lines.append(line.decode("utf-8"))

        self.reader = threading.Thread(target=accept_and_read, daemon=True)
        self.reader.start()

    def finish_listener(self):
        transport.close_connection()
        if self.reader is not None:
            self.reader.join(timeout=5)

    def test_messages_reach_the_client_and_are_newline_framed(self):
        self.start_listener()
        os.environ["ROBOTSIM_TRANSMIT"] = "True"

        transport.transmit_reset()
        transport.transmit_start()
        transport.transmit_warehouse_size(4, 5)
        self.finish_listener()

        self.assertEqual([json.loads(line)["command"] for line in self.lines],
                         ["RESET", "START", "WAREHOUSESIZE"])

    def test_a_whole_run_arrives_without_losing_a_single_message(self):
        # The point of the exercise. Over UDP a run of this size lost about a
        # fifth of its messages, including order completions.
        self.start_listener()
        os.environ["ROBOTSIM_TRANSMIT"] = "True"

        sent = []
        original_send = transport.send_message

        def counting_send(message):
            sent.append(message)
            original_send(message)

        transport.send_message = counting_send
        try:
            warehouse = Warehouse(WHOUSE, 10, 3, "multi-robot", [0, 0, 0, 0], True, 2000)
            keep_stepping = True
            while keep_stepping:
                keep_stepping = not warehouse.step()
        finally:
            transport.send_message = original_send
            self.finish_listener()

        self.assertGreater(len(sent), 300, "run was too small to be a fair test")
        self.assertEqual(len(self.lines), len(sent),
                         "lost %d of %d messages" % (len(sent) - len(self.lines), len(sent)))
        self.assertEqual(self.lines, sent, "messages arrived altered or out of order")

    def test_nothing_is_sent_when_transmission_is_disabled(self):
        self.start_listener()
        os.environ["ROBOTSIM_TRANSMIT"] = "False"

        transport.transmit_start()
        self.finish_listener()

        self.assertEqual(self.lines, [])

    def test_no_viewer_listening_is_not_an_error(self):
        # Batch runs and tests must not care whether Unity happens to be up.
        os.environ["ROBOTSIM_TRANSMIT"] = "True"
        transport.reset_connection()

        transport.transmit_start()
        transport.transmit_warehouse_size(1, 1)  # still silent, no retry storm

        self.assertIsNone(transport._connection)


if __name__ == "__main__":
    unittest.main()
