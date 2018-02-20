from time import sleep
from threading import Thread
import krpc
from datetime import datetime
from collections import defaultdict

from util import SingletonMixin, get_current_stage, get_current_decouple_stage
from state import AscentState, CoastToApoapsis, Circularized
from manager import CallbackManager, PitchManager, ThrottleManager


class Controller(SingletonMixin, object):
    ALTITUDE_TURN_START = 250
    ALTITUDE_TARGET = 175000

    pitch_follow = PitchManager.FIXED_UP

    current_state = None
    connection = None
    space_center = None

    _NextStateCls = None  # Expected class object hence the camel case

    _parts_in_stage = None

    def __init__(self):
        super(Controller, self).__init__()

        # DO NOT PUT THINGS IN HERE THAT CALL Controller.get()
        self.connection = krpc.connect(
            name='kPRC KSP Controller',
            address='127.0.0.1',
            rpc_port=50000, stream_port=50001)
        self.space_center = self.connection.space_center
        self._ut = self.connection.add_stream(getattr, self.connection.space_center, 'ut')
        self._altitude = self.connection.add_stream(getattr, self.vessel.flight(), 'mean_altitude')
        self._mach = self.connection.add_stream(getattr, self.vessel.flight(), 'mach')
        self._angle_of_attack = self.connection.add_stream(getattr, self.vessel.flight(), 'angle_of_attack')
        self._mass = self.connection.add_stream(getattr, self.vessel, 'mass')
        self._specific_impulse = self.connection.add_stream(getattr, self.vessel, 'specific_impulse')
        self._available_thrust = self.connection.add_stream(getattr, self.vessel, 'available_thrust')

        self._time_to_apoapsis = self.connection.add_stream(getattr, self.orbit, 'time_to_apoapsis')
        self._apoapsis_altitude = self.connection.add_stream(getattr, self.orbit, 'apoapsis_altitude')
        self._periapsis_altitude = self.connection.add_stream(getattr, self.orbit, 'periapsis_altitude')
        self._apoapsis = self.connection.add_stream(getattr, self.orbit, 'apoapsis')
        self._periapsis = self.connection.add_stream(getattr, self.orbit, 'periapsis')

        self._surface_gravity = self.connection.add_stream(getattr, self.body, 'surface_gravity')
        self._equatorial_radius = self.connection.add_stream(getattr, self.body, 'equatorial_radius')

        self.current_stage = get_current_stage(self.vessel)
        self.current_decouple_stage = get_current_decouple_stage(self.vessel)

        self._parts_in_stage = {
            'all': {},
            'engine': defaultdict(list),
        }
        self._parts_in_decouple_stage = {
            'all': {},
            'engine': defaultdict(list),
        }
        self._part_streams = defaultdict(dict)

        for i in range(self.current_stage, 0, -1):
            parts_in_stage = self.vessel.parts.in_stage(i)
            self._parts_in_stage['all'][i] = parts_in_stage

            for part in parts_in_stage:
                if part.engine:
                    self._parts_in_stage['engine'][i].append(part)
                    self._part_streams[part]['available_thrust'] = self.connection.add_stream(getattr, part.engine, 'available_thrust')

        for i in range(self.current_decouple_stage, 0, -1):
            parts_in_decouple_stage = self.vessel.parts.in_decouple_stage(i)
            self._parts_in_decouple_stage['all'][i] = parts_in_decouple_stage

            for part in parts_in_decouple_stage:
                if part.engine:
                    self._parts_in_decouple_stage['engine'][i].append(part)

    def run(self):
        self.callback_manager = CallbackManager()
        self.pitch_manager = PitchManager()
        self.throttle_manager = ThrottleManager()

        # self.current_state = AscentState()
        self.current_state = CoastToApoapsis()
        while True:
            if not self.current_state._thread.isAlive():
                NextStateCls = self.get_NextStateCls()
                if NextStateCls is None:
                    break
                self.set_NextStateCls(None)
                print 'switch state to: ', NextStateCls
                self.current_state = NextStateCls()

        sleep(5)

    @property
    def vessel(self):
        return self.space_center.active_vessel

    @property
    def orbit(self):
        return self.vessel.orbit

    @property
    def body(self):
        return self.orbit.body

    @property
    def ut(self):
        return self._ut()

    @property
    def altitude(self):
        return self._altitude()

    @property
    def mach(self):
        return self._mach()

    @property
    def angle_of_attack(self):
        return self._angle_of_attack()

    @property
    def time_to_apoapsis(self):
        return self._time_to_apoapsis()

    @property
    def apoapsis_altitude(self):
        return self._apoapsis_altitude()

    @property
    def periapsis_altitude(self):
        return self._periapsis_altitude()

    @property
    def apoapsis(self):
        return self._apoapsis()

    @property
    def periapsis(self):
        return self._periapsis()

    @property
    def mass(self):
        return self._mass()

    @property
    def specific_impulse(self):
        return self._specific_impulse()

    @property
    def surface_gravity(self):
        return self._surface_gravity()

    @property
    def available_thrust(self):
        return self._available_thrust()

    @property
    def equatorial_radius(self):
        return self._equatorial_radius()

    def get_NextStateCls(self):
        return self._NextStateCls

    def set_NextStateCls(self, NextStateCls):
        self._NextStateCls = NextStateCls

    def get_parts_in_stage(self, stage_num, part_type='all'):
        return self._parts_in_stage[part_type][stage_num]
        # stage_parts = self._parts_in_stage.get(stage_num, None)
        #
        # if stage_parts is None:
        #     stage_parts = self.vessel.parts.in_stage(stage_num)
        #     self._parts_in_stage[stage_num][] = stage_parts
        #
        # return stage_parts

    def get_parts_in_decouple_stage(self, stage_num, part_type='all'):
        return self._parts_in_decouple_stage[part_type][stage_num]

    def get_available_thrust(self, part):
        # part_data = self._part_streams.get(part, {'available_thrust': lambda: None})
        # if part_data is None:
        #     return None
        # return part_data['available_thrust']()
        return self._part_streams.get(part, {'available_thrust': lambda: None})['available_thrust']()

    def get_throttle(self):
        return self.throttle_manager.get_throttle()

    def set_throttle(self, val):
        self.throttle_manager.set_throttle(val)

