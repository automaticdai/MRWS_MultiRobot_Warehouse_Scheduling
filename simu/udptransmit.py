import socket
import os
IP = "127.0.0.1"
PORT = 35891
SOCK = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
commands = {}


def send_udp_message(message: str):
    if os.environ["ROBOTSIM_TRANSMIT"] == "True":
        print("sending")
        SOCK.sendto(bytes(message, "utf-8"), (IP, PORT))


def transmit_start():
    message = '{"command":"START"}'
    send_udp_message(message)


def transmit_warehouse_size(x: int, y: int):
    message = '{"command":"WAREHOUSESIZE", "posX":%s, "posY":%s}' % (x, y)
    send_udp_message(message)


def transmit_robot_position(name: str, x: int, y: int):
    message = '{"command":"MOVEROBOT", "posX":%s, "posY":%s, "objName":"%s"}' % (x, y, name)
    send_udp_message(message)


def transmit_robot_creation(name: str, x: int, y: int):
    message = '{"command":"CREATEROBOT", "posX":%s, "posY":%s, "objName":"%s"}' % (x, y, name)
    send_udp_message(message)


def transmit_shelf_creation(name: str, item: str, x: int, y: int):
    message = '{"command":"CREATESHELF", "posX":%s, "posY":%s, "objName":"%s", "itemName":"%s"}' % (x, y, name, item)
    send_udp_message(message)


def transmit_goal_creation(name: str, x: int, y: int):
    message = '{"command":"CREATEGOAL", "posX":%s, "posY":%s, "objName":"%s"}' % (x, y, name)
    send_udp_message(message)


def transmit_item_existence(name: str):
    message = '{"command":"ITEM", "itemName":"%s"}' % name
    send_udp_message(message)


def transmit_item_gained(objname: str, item_name: str):
    message = '{"command":"ITEMGAINED", "objName":"%s", "itemName":"%s"}' % (objname, item_name)
    send_udp_message(message)


def transmit_item_lost(objname: str, item_name: str):
    message = '{"command":"ITEMLOST", "objName":"%s", "itemName":"%s"}' % (objname, item_name)
    send_udp_message(message)


def transmit_clear_inventory(objname: str):
    message = '{"command":"CLEARINV", "objName":"%s"}' % objname
    send_udp_message(message)


def transmit_order_create(orderid: int, prio:int, items:list[str]):
    item_string = ""
    for item in items:
        item_string += item
        item_string += "|"
    item_string = item_string[:-1]
    message = '{"command":"ORDERCREATE", "objName":"%s", "posX":"%s", "itemName":"%s"}' % (orderid, prio, item_string)
    send_udp_message(message)

def transmit_order_active(orderid:int):
    message = '{"command":"ORDERACTIVE", "objName":"%s"}'

def transmit_order_complete(orderid: int):
    message = '{"command":"ORDERCOMPLETE", "objName":"%s"}' % orderid
    send_udp_message(message)





