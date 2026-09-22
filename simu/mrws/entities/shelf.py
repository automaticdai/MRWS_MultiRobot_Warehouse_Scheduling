from mrws.models.item import Item
from mrws.io import transport


class Shelf:
    def __init__(self, x_pos: int, y_pos: int, name: str, item_type: Item = None):
        self._x = x_pos
        self._y = y_pos
        self._item = item_type
        self._name = name

    def transmit_creation(self):
        transport.transmit_shelf_creation(self._name, self._item.get_name(), self._x, self._y)

    def interact(self, obj):
        obj.add_item_to_inventory(self._item)

    def get_position(self):
        return self._x, self._y

    def get_name(self):
        return self._name

    def get_item(self):
        return self._item

    def __repr__(self):
        return self._name
