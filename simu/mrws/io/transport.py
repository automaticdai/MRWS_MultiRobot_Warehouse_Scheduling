"""Streams simulator events to the Unity visualiser.

This was UDP, and UDP lost messages. The simulator emits thousands of small
datagrams with no flow control, and the receiver's buffer caps out at
net.core.rmem_max (208KB on a stock Linux), so once a run gets going the kernel
simply drops what will not fit. A capture of one short run received 357 of the
446 messages sent -- including two of the four ORDERCOMPLETEs. The viewer had no
way to know it had missed anything, so it silently desynced from the simulator,
and `slow_for_transmit`'s 200ms-per-step delay existed largely to paper over it.

TCP fixes that at the cost of coupling: while a viewer is connected, the
simulator runs no faster than the viewer consumes. That is the trade being made
deliberately -- a correct picture is worth more than an unblocked simulation.

Framing: one JSON object per line. TCP is a stream, so without a delimiter two
messages arriving together would be read as one. `json.dumps` never emits a raw
newline, so a newline is an unambiguous terminator.

When no viewer is listening, transmission is skipped silently: batch experiments
and tests must not care whether Unity happens to be running.
"""

import json
import os
import socket

HOST = "127.0.0.1"
PORT = 35891

# How long to wait for the viewer before giving up on it. Blocking is the point
# -- it is what stops messages being dropped -- but a viewer that has hung or
# gone away must never hang a 1000-run batch, so the wait is bounded.
SEND_TIMEOUT_S = 5.0

_connection = None
_connection_unavailable = False


def transmission_enabled() -> bool:
    return os.environ.get("ROBOTSIM_TRANSMIT") == "True"


def _connect():
    """Return the live connection, opening one if needed, or None."""
    global _connection, _connection_unavailable

    if _connection is not None:
        return _connection
    if _connection_unavailable:
        return None

    try:
        sock = socket.create_connection((HOST, PORT), timeout=SEND_TIMEOUT_S)
        sock.settimeout(SEND_TIMEOUT_S)
        # Small messages sent back to back; Nagle would add latency for nothing.
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        _connection = sock
    except OSError:
        # No viewer listening. Stop trying for the rest of this run.
        _connection_unavailable = True
        return None

    return _connection


def close_connection():
    """Drop the connection, if any. Safe to call when there is none."""
    global _connection
    if _connection is not None:
        try:
            _connection.close()
        except OSError:
            pass
        _connection = None


def reset_connection():
    """Drop the connection and forget that a viewer was unavailable."""
    global _connection_unavailable
    close_connection()
    _connection_unavailable = False


def _allow_reconnect():
    """Let a new run look for a viewer again, without disturbing a live
    connection -- a run opens with RESET immediately followed by START, and
    closing the socket between them would lose everything after the RESET."""
    global _connection_unavailable
    _connection_unavailable = False


def send_message(message: str):
    global _connection_unavailable

    if not transmission_enabled():
        return

    connection = _connect()
    if connection is None:
        return

    try:
        connection.sendall((message + "\n").encode("utf-8"))
    except OSError:
        # The viewer went away or stopped reading. Give up on it rather than
        # stalling the simulation on every subsequent message.
        close_connection()
        _connection_unavailable = True


def transmit_start():
    send_message(json.dumps({"command": "START"}))


def transmit_reset():
    # RESET opens a run, so this is where a new run gets a fresh attempt at
    # finding a viewer if an earlier one gave up.
    _allow_reconnect()
    send_message(json.dumps({"command": "RESET"}))


def transmit_warehouse_size(x: int, y: int):
    send_message(json.dumps({"command": "WAREHOUSESIZE", "posX": x, "posY": y}))


def transmit_robot_position(name: str, x: int, y: int):
    send_message(json.dumps({"command": "MOVEROBOT", "posX": x, "posY": y, "objName": name}))


def transmit_robot_creation(name: str, x: int, y: int):
    send_message(json.dumps({"command": "CREATEROBOT", "posX": x, "posY": y, "objName": name}))


def transmit_shelf_creation(name: str, item: str, x: int, y: int):
    send_message(json.dumps({"command": "CREATESHELF", "posX": x, "posY": y,
                             "objName": name, "itemName": item}))


def transmit_goal_creation(name: str, x: int, y: int):
    send_message(json.dumps({"command": "CREATEGOAL", "posX": x, "posY": y, "objName": name}))


def transmit_item_existence(name: str):
    send_message(json.dumps({"command": "ITEM", "itemName": name}))


def transmit_item_gained(objname: str, item_name: str):
    send_message(json.dumps({"command": "ITEMGAINED", "objName": objname, "itemName": item_name}))


def transmit_item_lost(objname: str, item_name: str):
    send_message(json.dumps({"command": "ITEMLOST", "objName": objname, "itemName": item_name}))


def transmit_clear_inventory(objname: str):
    send_message(json.dumps({"command": "CLEARINV", "objName": objname}))


def transmit_order_create(orderid: int, prio: int, items: list[str]):
    # Priority travels in its own int field. It used to be sent as a string in
    # posX, which Unity deserialises into `public int posX` -- so it never
    # parsed. Nothing called this, so the breakage was never noticed.
    send_message(json.dumps({"command": "ORDERCREATE", "objName": order_object_name(orderid),
                             "prio": prio, "itemName": "|".join(items)}))


def transmit_order_complete(orderid: int):
    send_message(json.dumps({"command": "ORDERCOMPLETE", "objName": order_object_name(orderid)}))


def transmit_robot_fault(robot_name: str, fault_type: str):
    send_message(json.dumps({"command": "ROBOTFAULT", "objName": robot_name,
                             "faultType": fault_type}))


def transmit_robot_recovered(robot_name: str):
    send_message(json.dumps({"command": "ROBOTRECOVERED", "objName": robot_name}))


def order_object_name(orderid: int) -> str:
    """Orders are named like every other entity, so the client can key on them."""
    return "order%s" % orderid
