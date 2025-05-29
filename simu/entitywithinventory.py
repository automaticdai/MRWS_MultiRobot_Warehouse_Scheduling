import customexceptions
import item
import math

import udptransmit
import copy

# Models an entity with a LIFO stack inventory, where items within the stack must follow a dependency
class InventoryEntity:
    def __init__(self, name: str, max_inventory_size, is_simulation_obj=True):
        self._name = name
        self._inventory = []
        self.last_item_dep = math.inf
        self._max_inv = max_inventory_size
        self._amount_items_transfer_next_time = None

        # This is used by the genetic algorithm to stop the simplified simulator from transmitting
        self._should_transmit = is_simulation_obj

    def add_item_to_inventory(self, item_to_add: item.Item):
        new_dep = item_to_add.get_dependency()
        if new_dep <= self.last_item_dep:
            self._inventory.append(item_to_add)
            self.last_item_dep = new_dep

            if len(self._inventory) > self._max_inv:
                raise customexceptions.SimulationError("Inventory of object %s overfilled" % self._name)

            if self._should_transmit:
                udptransmit.transmit_item_gained(self._name, item_to_add.get_name())
        else:
            raise customexceptions.SimulationError("Item dependency rule violated by object %s" % self._name)

    def pop_item_from_inventory(self) -> item.Item:
        if len(self._inventory) == 0:
            raise customexceptions.SimulationError("Tried to pop from an empty inventory")
        popped_item = self._inventory.pop()
        if self._should_transmit:
            udptransmit.transmit_item_lost(self._name, popped_item.get_name())
        if len(self._inventory) == 0:
            self.last_item_dep = math.inf
        else:
            self.last_item_dep = self._inventory[-1].get_dependency()
        return popped_item

    def clear_inventory(self):
        self._inventory = []
        if self._should_transmit:
            udptransmit.transmit_clear_inventory(self._name)
        self.last_item_dep = math.inf


    def set_amount_of_items_to_transfer_next_time(self, num):
        self._amount_items_transfer_next_time = num


    def peek_inventory(self):
        return copy.deepcopy(self._inventory[-1])

    def transfer_inventory(self):
        if len(self._inventory) == 0:
            raise customexceptions.SimulationError("Tried to transfer an empty inventory")

        if self._amount_items_transfer_next_time is None:
            inventory_copy = copy.deepcopy(self._inventory)
            self._inventory = []
            if self._should_transmit:
                udptransmit.transmit_clear_inventory(self._name)
            self.last_item_dep = math.inf
            return inventory_copy
        else:
            transfer_inv = []
            for i in range(self._amount_items_transfer_next_time):
                transfer_inv.append(self.pop_item_from_inventory())
            self._amount_items_transfer_next_time = None
            transfer_inv.reverse()
            return transfer_inv

    def receive_inventory(self, items):
        for itm in items:
            self.add_item_to_inventory(itm)

    # For each item in the inventory
    # The dependency number must be smaller than any item previously
    # Returns false if dependency check fails, true otherwise
    def validate_complete_inventory(self) -> bool:
        previous = math.inf
        for itm in self._inventory:
            current = itm.get_dependency()
            if current <= previous:
                previous = current
            else:
                return False
        return True

    def report_inventory(self):
        return copy.deepcopy(self._inventory)

    def report_inventory_item_names(self):
        item_names = []
        for item in self._inventory:
            item_names.append(item.get_name())
        return item_names

    def get_inventory_usage(self):
        return len(self._inventory)