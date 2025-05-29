class Item:
    def __init__(self, item_name: str, item_dep: int):
        self._name = item_name
        self._dep = item_dep

    def __eq__(self, other):
        return self._name == other.get_name() and self._dep == other.get_dependency()

    def get_dependency(self):
        return self._dep

    def get_name(self):
        return self._name

    def __repr__(self):
        return "Item name %s Item dep %s" % (self._name, self._dep)
