import random

import udptransmit
import entitywithinventory
import math
import shelf
import orderstation
import customexceptions

class Robot(entitywithinventory.InventoryEntity):
    def __init__(self, name: str, x: int, y: int, max_inv_size: int, fault_rates: list):
        self._x = x
        self._y = y
        self._home_x = x
        self._home_y = y
        self.wait_steps = 0
        self._assigned_order = None
        self._movement_path = []
        self._current_target = None
        self._steps_halted = 0
        self._prio = None

        self._battery_critical_fault_rate = fault_rates[0]
        self._battery_low_fault_rate = fault_rates[1]
        self._actuator_fault_rate = fault_rates[2]
        self._sensor_fault_rate = fault_rates[3]

        self._goal_visit_flag = None

        self.battery_faulted = False
        self.gone_home_to_clear_inv = False

        self.charge_time = 50
        self.apply_charge_wait_upon_reaching_home = False

        self.battery_faulted_critical = False
        self.sensors_faulted = False
        self.actuators_faulted = False

        super().__init__(name, max_inv_size)

    def set_assigned_order(self, id_num):
        self._assigned_order = id_num

    def set_flag(self, flag: str):
        self._goal_visit_flag = flag

    def consume_flag(self):
        return_val = self._goal_visit_flag
        self._goal_visit_flag = None
        return return_val

    def get_assigned_order(self):
        return self._assigned_order

    def get_position(self):
        return self._x, self._y

    def set_prio(self, prio):
        self._prio = prio

    def get_prio(self):
        return self._prio

    def add_wait_steps(self, step_amount):
        self.wait_steps = self.wait_steps + step_amount

    def get_wait_steps(self):
        return self.wait_steps

    def decrement_wait_steps(self):
        if self.wait_steps == math.inf:
            return False

        self.wait_steps = self.wait_steps - 1

        # Reset the states of the faults that involve waiting after the wait is over
        if self.wait_steps == 0:
            self.actuators_faulted = False
            if (self._home_x == self._x) and (self._home_y == self._y) and self.battery_faulted:
                self.battery_faulted = False
                return True


    def set_position(self, x, y):
        if self.wait_steps != 0:
            raise customexceptions.SimulationError("Cannot move a robot that is waiting")
        self._steps_halted = 0
        self._x = x
        self._y = y

    def increment_steps_halted(self):
        self._steps_halted = self._steps_halted + 1
        if self._steps_halted > 10:
            self._steps_halted = 0

    def get_steps_halted(self):
        return self._steps_halted

    def set_movement_path(self, path):
        self._movement_path = path

    def get_movement_path(self):
        return self._movement_path

    def set_target(self, target):
        self._current_target = target
        if target is None:
            self._prio = None

    def get_target(self):
        return self._current_target

    def interact_with_target(self):
        if self.get_position() != self._current_target.get_position():
            message = "Robot %s tried to interact with object %s, when they were not occupying the same cell."
            raise customexceptions.SimulationError(message % (self._name, self._current_target.get_name()))

        self._current_target.interact(self)

        self._current_target = None

    def is_at_target(self):
        if self._current_target is None:
            return False

        target_x, target_y = self._current_target.get_position()
        if (self._x == target_x) and (self._y == target_y):
            return True
        else:
            return False

    def get_name(self):
        return self._name

    def transmit_creation(self):
        udptransmit.transmit_robot_creation(self._name, self._x, self._y)

    def maybe_introduce_fault(self):
        # Battery fault - 2 types:
        # Low battery - robot must return to its home and becomes unavailable for a certain number of steps
        # Battery failure - robot breaks unrecoverable.
        # Actuator fault - robot cannot move for a certain number of steps - Models overheating
        # Sensor fault - robot cannot determine what is around it - Is permanent
        # The robot cannot take any actions if the battery has failed
        f0, f1, f2, f3 = False, False, False, False
        if self.battery_faulted_critical:
            return []
        if random.random() < self._battery_critical_fault_rate:
            self.battery_faulted_critical = True
            #print("FAULT %s BATTERY CRITICAL" % self._name)
            f0 = True
        else:
            if random.random() < self._battery_low_fault_rate and not self.battery_faulted:
                #print("FAULT %s BATTERY RECHARGE" % self._name)
                self.battery_faulted = True
                f1 = True
            if random.random() < self._actuator_fault_rate and not self.actuators_faulted:
                self.actuators_faulted = True
                #print("FAULT %s ACTUATOR OVERHEAT" % self._name)
                f2 = True
            if random.random() < self._sensor_fault_rate and not self.sensors_faulted:
                self.sensors_faulted = True
                #print("FAULT %s SENSOR FAILURE" % self._name)
                f3 = True
        return [f0, f1, f2, f3]


    def __repr__(self):
        return self._name
