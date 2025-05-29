import item
import udptransmit


class Shelf:
    def __init__(self, x_pos: int, y_pos: int, name: str, item_type: item.Item = None):
        self._x = x_pos
        self._y = y_pos
        self._item = item_type
        self._name = name

    def transmit_creation(self):
        udptransmit.transmit_shelf_creation(self._name, self._item.get_name(), self._x, self._y, )

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
