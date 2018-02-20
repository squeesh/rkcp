from threading import Thread
from collections import defaultdict
from datetime import datetime
import math


class CallbackManager(object):
    _callbacks = defaultdict(list)

    def __init__(self):
        from controller import Controller
        from callback import has_fuel_callback

        ctrl = Controller.get()

        for part in ctrl.vessel.parts.all:
            if part.engine:
                self.register_callback('fuel', ctrl.connection.add_stream(getattr, part.engine, 'has_fuel'), has_fuel_callback)
                # e_stream = self.connection.add_stream(getattr, part.engine, 'has_fuel')
                # e_stream.add_callback(has_fuel_callback)
                # e_stream.start()

    def register_callback(self, name, stream, callback):
        stream.add_callback(callback)
        stream.start()
        self._callbacks[name].append(stream)


class PitchManager(object):
    FIXED_UP = 0
    FIXED_POINT = 1
    SURFACE_PROGRADE = 2
    ORBIT_PROGRADE = 3
    curr_follow = FIXED_UP

    def __init__(self):
        from controller import Controller
        self.ctrl = Controller.get()
        self._thread = Thread(target=self.run)
        self._thread.daemon = True
        self._thread.start()

    def run(self):
        while True:
            vessel = self.ctrl.vessel
            follow = self.ctrl.pitch_follow

            if follow == self.FIXED_UP:
                vessel.auto_pilot.reference_frame = vessel.surface_reference_frame
                vessel.auto_pilot.target_pitch_and_heading(90, 90)
            elif follow == self.FIXED_POINT:
                vessel.auto_pilot.reference_frame = vessel.surface_reference_frame
                vessel.auto_pilot.target_pitch_and_heading(80, 90)
            elif follow == self.SURFACE_PROGRADE:
                vessel.auto_pilot.reference_frame = vessel.surface_velocity_reference_frame
                vessel.auto_pilot.target_direction = (0, 1, 0)
                # if vessel.auto_pilot.target_roll == 180:
                #     vessel.auto_pilot.target_pitch_and_heading(flight.pitch + flight.angle_of_attack, 90)
                # else:
                #     vessel.auto_pilot.target_pitch_and_heading(flight.pitch - flight.angle_of_attack, 90)
            elif follow == self.ORBIT_PROGRADE:
                # orb_prograde, _ = get_pitch_heading(flight.prograde)
                # vessel.auto_pilot.target_pitch_and_heading(orb_prograde, 90)
                flight = self.ctrl.vessel.flight()
                vessel.auto_pilot.reference_frame = vessel.surface_reference_frame
                vessel.auto_pilot.target_direction = flight.prograde


class ThrottleManager(object):
    _old_throttle = 0
    _throttle = 0

    def __init__(self):
        from controller import Controller
        self.ctrl = Controller.get()
        self._thread = Thread(target=self.run)
        self._thread.daemon = True
        self._thread.start()

    def run(self):
        while True:
            if math.fabs(self._throttle - self._old_throttle) > 0.01:
                self.ctrl.vessel.control.throttle = self._throttle
                self._old_throttle = self._throttle

    def get_throttle(self):
        return self._throttle

    def set_throttle(self, val):
        # # if self._throttle != val:
        # if math.fabs(self._throttle - val) > 0.01:
        self._throttle = val
            # self.ctrl.vessel.control.throttle = val
