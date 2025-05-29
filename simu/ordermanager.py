import order
import random
import time

class OrderManager:
    def __init__(self, num_init_orders: int, num_dynamic_orders: int, item_set: dict, dynamic_deadline:int):
        self._num_init_orders = num_init_orders
        self._num_dynamic_orders = num_dynamic_orders
        self._item_set = item_set
        self._dynamic_deadline = dynamic_deadline

        self._init_orders = []
        self._dynamic_orders = []

        self._order_intro_times = {}
        self._order_work_start_times = {}
        self._order_completion_times = {}
        self._all_orders = {}
        self.generate_orders_uniform(num_init_orders, num_dynamic_orders, 4)

        for ordr in self._init_orders:
            self._order_intro_times[ordr.get_id()] = 0

        self._dynamic_orders_intro_steps = {}
        self.generate_dynamic_order_introduction_times_uniform(self._dynamic_deadline)


    def get_dynamic_deadline(self):
        return self._dynamic_deadline
    def set_order_start_work_time(self, order_id, step_value):
        self._order_work_start_times[order_id] = step_value

    def set_order_completion_time(self, ordr: order.Order, step_value:int):
        self._order_completion_times[ordr.get_id()] = step_value

    def generate_orders_uniform(self, num_init_orders: int, num_dynamic_orders: int, order_size: int):
        order_id_ctr = 0
        for i in range(num_init_orders):
            current_items = []
            for j in range(order_size):
                selected_item_index = random.randint(0, len(self._item_set.keys()) - 1)
                current_items.append(self._item_set["item%s" % selected_item_index])
            order_prio = random.randint(1, 5)
            current_order = order.Order(current_items, order_prio, order_id_ctr)
            self._all_orders[order_id_ctr] = current_order
            self._init_orders.append(current_order)
            order_id_ctr = order_id_ctr + 1

        for i2 in range(num_dynamic_orders):
            current_items = []
            for j2 in range(order_size):
                selected_item_index = random.randint(0, len(self._item_set.keys()) - 1)
                current_items.append(self._item_set["item%s" % selected_item_index])
            order_prio = random.randint(1, 5)
            current_order = order.Order(current_items, order_prio, order_id_ctr)
            self._all_orders[order_id_ctr] = current_order
            self._dynamic_orders.append(current_order)
            order_id_ctr = order_id_ctr + 1

    def generate_dynamic_order_introduction_times_uniform(self, deadline):
        for ordr in self._dynamic_orders:
            selected_step = random.randint(1, deadline - 1)
            while selected_step in self._dynamic_orders_intro_steps.keys():
                selected_step = random.randint(1, deadline - 1)
            self._dynamic_orders_intro_steps[selected_step] = ordr
            self._order_intro_times[ordr.get_id()] = selected_step

    def possibly_introduce_dynamic_order(self, step):
        if step in self._dynamic_orders_intro_steps.keys():
            return self._dynamic_orders_intro_steps[step]

    def get_init_orders(self):
        return self._init_orders

    def print_orders(self):
        print("Initial Orders:")
        for order_obj in self._init_orders:
            print(order_obj)
        print("Dynamic Orders:")
        for order_obj in self._dynamic_orders:
            print(order_obj)

    def print_order_completion_times(self):
        print("Order Completion times:")
        for order_id in self._order_intro_times.keys():
            print("Order id %s" % order_id)
            if order_id in self._order_completion_times.keys():
                print("Took %s steps to complete after its introduction" %
                      (self._order_completion_times[order_id] - self._order_intro_times[order_id]))
                print("The priority was %s" % self._all_orders[order_id].get_prio())
            else:
                print("Has not finished yet")
            print("")

    def return_mapping_prio_to_completion_times(self):
        ret = []
        for order_id, completion in self._order_completion_times.items():
            ret.append((self._all_orders[order_id].get_prio(), completion))
        return ret

    def get_order_start_work_times(self):
        return self._order_work_start_times

    def get_order_finish_work_times(self):
        return self._order_completion_times








