from mrws.entities.inventory import InventoryEntity
from mrws.io import transport
import math

class OrderStation(InventoryEntity):
    def __init__(self, x_pos: int, y_pos: int, name: str, get_scheduler, get_order_manager, get_total_steps):
        self._x = x_pos
        self._y = y_pos
        self._get_scheduler = get_scheduler
        self._get_order_manager = get_order_manager
        self._get_total_steps = get_total_steps
        super().__init__(name, math.inf)

    def transmit_creation(self):
        transport.transmit_goal_creation(self._name, self._x, self._y)

    def interact(self, obj):
        #print("Robot %s interacting with order station %s" % (obj.get_name(), self._name))
        received = obj.transfer_inventory()
        #print("Order station %s recieved %s" % (self.get_name(), received))
        #print("Already had %s" % self._inventory)
        self.receive_inventory(received)

        if self._get_scheduler().is_this_a_complete_order(self.report_inventory(),
                                                          self._get_order_manager(),
                                                          obj,
                                                          self._name,
                                                          self._get_total_steps()):
            self.clear_inventory()

        flag_maybe = obj.consume_flag()
        if flag_maybe is not None:
            self._get_scheduler().add_flag(flag_maybe)

    def get_position(self):
        return self._x, self._y

    def get_name(self):
        return self._name

    def __repr__(self):
        return self._name
