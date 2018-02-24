from time import sleep
from threading import Thread, Lock, Condition
import krpc
from datetime import datetime
from collections import defaultdict
from functools import partial
from itertools import chain

from util import SingletonMixin, get_current_stage, get_current_decouple_stage, get_vessel_pitch_heading
from state import PreLaunch, AscentState, CoastToApoapsis, InOrbit, SurfaceHover, TransitionToHover
from manager import StagingManager, PitchManager, ThrottleManager, BurnManager


class Controller(SingletonMixin, object):
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

        self._streams_open = 0

        stream_funcs = (
            ('surface_altitude', (getattr, self.vessel.flight(), 'surface_altitude')),
            ('vertical_speed', (getattr, self.vessel.flight(self.body.reference_frame), 'vertical_speed')),

            ('mach', (getattr, self.vessel.flight(), 'mach')),
            ('angle_of_attack', (getattr, self.vessel.flight(), 'angle_of_attack')),
            ('mass', (getattr, self.vessel, 'mass')),
            ('specific_impulse', (getattr, self.vessel, 'specific_impulse')),
            ('available_thrust', (getattr, self.vessel, 'available_thrust')),
            ('thrust', (getattr, self.vessel, 'thrust')),

            ('time_to_apoapsis', (getattr, self.orbit, 'time_to_apoapsis')),
            ('apoapsis_altitude', (getattr, self.orbit, 'apoapsis_altitude')),
            ('periapsis_altitude', (getattr, self.orbit, 'periapsis_altitude')),
            ('apoapsis', (getattr, self.orbit, 'apoapsis')),
            ('periapsis', (getattr, self.orbit, 'periapsis')),

            ('surface_gravity', (getattr, self.body, 'surface_gravity')),
            ('equatorial_radius', (getattr, self.body, 'equatorial_radius')),
        )

        self._stream_locks = {}
        def get_stream_val(self, name, stream_args):
            if self._stream_dict.get(name, None) is None:
                with self._stream_locks[name]:
                    if self._stream_dict.get(name, None) is None:
                        print 'opening stream: {}'.format(name)
                        self._stream_dict[name] = self.connection.add_stream(*stream_args)
                        self._streams_open += 1
            return self._stream_dict[name]()

        self._stream_dict = {}
        for name, stream_args in stream_funcs:
            func = partial(get_stream_val, name=name, stream_args=stream_args)
            setattr(self.__class__, name, property(func))
            self._stream_locks[name] = Lock()

        # self._mach = self.connection.add_stream(getattr, self.vessel.flight(), 'mach')
        # self._angle_of_attack = self.connection.add_stream(getattr, self.vessel.flight(), 'angle_of_attack')
        # self._mass = self.connection.add_stream(getattr, self.vessel, 'mass')
        # self._specific_impulse = self.connection.add_stream(getattr, self.vessel, 'specific_impulse')
        # self._available_thrust = self.connection.add_stream(getattr, self.vessel, 'available_thrust')
        # self._thrust = self.connection.add_stream(getattr, self.vessel, 'thrust')
        #
        # self._time_to_apoapsis = self.connection.add_stream(getattr, self.orbit, 'time_to_apoapsis')
        # self._apoapsis_altitude = self.connection.add_stream(getattr, self.orbit, 'apoapsis_altitude')
        # self._periapsis_altitude = self.connection.add_stream(getattr, self.orbit, 'periapsis_altitude')
        # self._apoapsis = self.connection.add_stream(getattr, self.orbit, 'apoapsis')
        # self._periapsis = self.connection.add_stream(getattr, self.orbit, 'periapsis')
        #
        # self._surface_gravity = self.connection.add_stream(getattr, self.body, 'surface_gravity')
        # self._equatorial_radius = self.connection.add_stream(getattr, self.body, 'equatorial_radius')

        # self.current_stage = get_current_stage(self.vessel)
        # self.current_decouple_stage = get_current_decouple_stage(self.vessel)

        self._parts_in_stage = {
            'all': defaultdict(list),
            'engine': defaultdict(list),
            # 'liquidfuel': defaultdict(list),
        }
        self._parts_in_decouple_stage = {
            'all': defaultdict(list),
            'engine': defaultdict(list),
            # 'liquidfuel': defaultdict(list),
        }
        # self._part_streams = defaultdict(dict)
        self._engine_data = defaultdict(dict)

        # for i in range(max([part.stage for part in self.vessel.parts.all]), -2, -1):
        #     parts_in_stage = self.vessel.parts.in_stage(i)
        #     self._parts_in_stage['all'][i] = parts_in_stage

            # for part in parts_in_stage:
        for part in self.vessel.parts.all:
            stage = part.stage
            self._parts_in_stage['all'][stage].append(part)
            decouple_stage = part.decouple_stage
            self._parts_in_decouple_stage['all'][decouple_stage].append(part)
            if part.engine:
                self._parts_in_stage['engine'][stage].append(part)
                self._engine_data[part]['has_fuel'] = True
                self._parts_in_decouple_stage['engine'][decouple_stage].append(part)
                # self._part_streams[part]['available_thrust'] = self.connection.add_stream(getattr, part.engine, 'available_thrust')
                # self._part_streams[part]['thrust'] = self.connection.add_stream(getattr, part.engine, 'thrust')

        # for i in range(max([part.decouple_stage for part in self.vessel.parts.all]), -2, -1):
        #     parts_in_decouple_stage = self.vessel.parts.in_decouple_stage(i)
        #     self._parts_in_decouple_stage['all'][i] = parts_in_decouple_stage
        #
        #     for part in parts_in_decouple_stage:
        #         if part.engine:
        #             self._parts_in_decouple_stage['engine'][i].append(part)

    def run(self):
        self.staging_manager = StagingManager()
        self.pitch_manager = PitchManager()
        self.throttle_manager = ThrottleManager()
        self.burn_manager = BurnManager()

        self.state_condition = Condition()

        # print 'dcs: {}'.format(self.current_decouple_stage)
        # dv1 = self.burn_manager.get_dv_in_decouple_stage(self.current_decouple_stage)
        # print 'dv1: {}'.format(dv1)
        # print 'bt1: {}'.format(self.burn_manager.get_burn_times(dv1, self.current_decouple_stage))
        # dv2 = self.burn_manager.get_dv_in_decouple_stage(self.current_decouple_stage-1)
        # print 'dv2: {}'.format(dv2)
        # print 'bt2: {}'.format(self.burn_manager.get_burn_times(dv2, self.current_decouple_stage-1))
        # dv3 = 500
        # print 'dv3: {}'.format(dv3)
        # print 'bt3: {}'.format(self.burn_manager.get_burn_times(dv3, self.current_decouple_stage))
        # print self.mass
        # print
        # sleep(1)
        # asdfasdf

        # print self.connection.krpc.paused
        # self.connection.krpc.paused = True

        self._NextStateCls = PreLaunch
        # self.current_state = PreLaunch()
        # self.current_state = AscentState()
        # self.current_state = CoastToApoapsis()
        # self.current_state = TransitionToHover()
        # self._NextStateCls = InOrbit
        #
        # self.vessel.auto_pilot.stopping_time = (4.0, 4.0, 4.0)
        #
        # self.vessel.control.rcs = True
        # self.pitch_follow = PitchManager.ORBIT_PROGRADE
        # self.vessel.auto_pilot.engage()

        while True:
            # if not self.current_state._thread.isAlive():
            self.state_condition.acquire()

            NextStateCls = self.get_NextStateCls()
            if NextStateCls is None:
                break
            self.set_NextStateCls(None)
            print 'switch state to: ', NextStateCls
            self.current_state = NextStateCls()

            self.state_condition.wait()
            self.state_condition.release()



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

    # @property
    # def mach(self):
    #     return self._mach()
    #
    # @property
    # def angle_of_attack(self):
    #     return self._angle_of_attack()
    #
    # @property
    # def time_to_apoapsis(self):
    #     return self._time_to_apoapsis()
    #
    # @property
    # def apoapsis_altitude(self):
    #     return self._apoapsis_altitude()
    #
    # @property
    # def periapsis_altitude(self):
    #     return self._periapsis_altitude()
    #
    # @property
    # def apoapsis(self):
    #     return self._apoapsis()
    #
    # @property
    # def periapsis(self):
    #     return self._periapsis()
    #
    # @property
    # def mass(self):
    #     return self._mass()
    #
    # @property
    # def specific_impulse(self):
    #     return self._specific_impulse()
    #
    # @property
    # def surface_gravity(self):
    #     return self._surface_gravity()
    #
    # @property
    # def available_thrust(self):
    #     return self._available_thrust()
    #
    # @property
    # def thrust(self):
    #     return self._thrust()
    #
    # @property
    # def equatorial_radius(self):
    #     return self._equatorial_radius()

    def set_burn_dv(self, delta_v):
        self.burn_manager.set_burn_dv(delta_v)

    def set_burn_point(self, point_in_time):
        self.burn_manager.set_burn_point(point_in_time)

    def get_burn_dv(self):
        return self.burn_manager.get_burn_dv()

    def get_burn_start(self):
        return self.burn_manager.get_burn_start()

    def get_burn_time(self):
        return self.burn_manager.get_burn_time()

    def get_NextStateCls(self):
        return self._NextStateCls

    def set_NextStateCls(self, NextStateCls):
        self._NextStateCls = NextStateCls

    def get_parts_in_stage(self, stage_num, part_type='all'):
        # print 'get_parts_in_stage: {}'.format(part_type)
        # for key, val in self._parts_in_stage[part_type].items():
        #     print '{} | {}'.format(key, val)
        return self._parts_in_stage[part_type][stage_num]

    def all_parts_after_stage(self, stage_num):
        all_stage_parts = []
        for curr_stage in range(stage_num, -2, -1):
            all_stage_parts.extend(self.get_parts_in_stage(curr_stage))
        return all_stage_parts

    def get_parts_in_decouple_stage(self, stage_num, part_type='all'):
        # print 'get_parts_in_decouple_stage: {}'.format(part_type)
        # for key, val in self._parts_in_decouple_stage[part_type].items():
        #     print '{} | {}'.format(key, val)
        return self._parts_in_decouple_stage[part_type][stage_num]

    def all_parts_after_decouple_stage(self, stage_num):
        all_decouple_stage_parts = []
        for curr_decouple_stage in range(stage_num, -2, -1):
            all_decouple_stage_parts.extend(self.get_parts_in_decouple_stage(curr_decouple_stage))
        return all_decouple_stage_parts

    def get_available_thrust(self, part):
        return part.engine.available_thrust
        # return 0
        # print 'part: {} in: {}'.format(part, (part in self._part_streams))
        # stream = self._part_streams.get(part, {'available_thrust': lambda: None})['available_thrust']
        # print 'stream: {}'.format(stream)
        # output = stream()
        # print 'avail thrust: {}'.format(output)
        # return output

    def get_thrust(self, part):
        return part.engine.thrust
        # return self._part_streams.get(part, {'thrust': lambda: None})['thrust']()

    def get_thrust_for_engines(self, engines):
        return sum([part.engine.max_thrust for part in engines])

    def get_isp_for_engines(self, engines):
        # ASSUMPTION: parts are engines!
        total_thrust = self.get_thrust_for_engines(engines)
        isp = 0
        for part in engines:
            engine_contrib = part.engine.max_thrust / total_thrust
            isp += part.engine.vacuum_specific_impulse * engine_contrib
        return isp

    def get_resource_mass_for_decouple_stage(self, decouple_stage):
        resources_in_decouple_stage = self.vessel \
            .resources_in_decouple_stage(decouple_stage, cumulative=False)

        density_lf = resources_in_decouple_stage.density('LiquidFuel')
        liquid_fuel_in_stage = resources_in_decouple_stage.amount('LiquidFuel')
        density_ox = resources_in_decouple_stage.density('Oxidizer')
        oxidizer_in_stage = resources_in_decouple_stage.amount('Oxidizer')
        return density_lf * liquid_fuel_in_stage + density_ox * oxidizer_in_stage

    def get_throttle(self):
        return self.throttle_manager.get_throttle()

    def set_throttle(self, val):
        self.throttle_manager.set_throttle(val)

    @property
    def engine_data(self):
        return self._engine_data

    @property
    def current_stage(self):
        print
        print '&&current_stage'
        for part_type, stages in self._parts_in_stage.items():
            print part_type
            for stage, parts in stages.items():
                print '\t', stage, '|', parts
        return self.vessel.control.current_stage
        # return max(chain(*[stages.keys() for part_type, stages in self._parts_in_stage.items()]))

    @property
    def current_decouple_stage(self):
        print
        print '##current_decouple_stage'
        for part_type, stages in self._parts_in_decouple_stage.items():
            print part_type
            for stage, parts in stages.items():
                print '\t', stage, '|', parts
        return max(chain(*[stages.keys() for part_type, stages in self._parts_in_decouple_stage.items()]))

    def activate_next_stage(self):
        # stage_num = max(self.current_stage, self.current_decouple_stage)
        old_stage_num = self.vessel.control.current_stage
        # [stages.pop(stage_num, None) for part_type, stages in self._parts_in_stage.items()]
        # [stages.pop(stage_num, None) for part_type, stages in self._parts_in_decouple_stage.items()]
        self.vessel.control.activate_next_stage()
        # stage_num = self.ctrl.vessel.control.current_stage
        new_stage_num = old_stage_num-1

        # parts_in_stage = self.vessel.parts.in_stage(stage_num)
        # print '**PARTS: {}'.format(parts_in_stage)
        # for part in parts_in_stage:
        #     if part not in self._parts_in_stage['all'][stage_num]:
        #         self._parts_in_stage['all'][stage_num].append(part)
        #     if part.engine and part not in self._parts_in_stage['engine'][stage_num]:
        #         self._parts_in_stage['engine'][stage_num].append(part)

        # to_print = defaultdict(list)

        for part_type in self._parts_in_stage:
            for part in self._parts_in_stage[part_type][old_stage_num]:
                # if part not in to_print[part.stage]:
                #     to_print[part.stage].append(part)
                if part not in self._parts_in_stage[part_type][part.stage]:
                    self._parts_in_stage[part_type][part.stage].append(part)

        [stages.pop(old_stage_num, None) for part_type, stages in self._parts_in_stage.items()]
        [stages.pop(new_stage_num, None) for part_type, stages in self._parts_in_decouple_stage.items()]

        print '**STAGE {}'.format(old_stage_num)
        # for key, val in to_print.items():
        #     print key, '**', val

    def get_local_gravity(self):
        r = self.equatorial_radius
        h = self.altitude
        g = self.surface_gravity
        return g * (r / (r + h)) ** 2

    def get_avail_twr(self):
        return self.available_thrust / (self.mass * self.get_local_gravity())

    def get_twr(self):
        return self.thrust / (self.mass * self.get_local_gravity())

