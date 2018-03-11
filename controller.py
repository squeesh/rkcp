from time import sleep
from threading import Thread, Lock, Condition
import krpc
from datetime import datetime
from collections import defaultdict
from functools import partial
from itertools import chain
from copy import copy
import math
from sympy import Ellipse, Circle, Point

from util import SingletonMixin, get_current_stage, get_current_decouple_stage, get_vessel_pitch_heading, engine_is_active, get_pitch_heading
from state import PreLaunch, AscentState, CoastToApoapsis, InOrbit, CoastToInterceptBurn, SurfaceHover, TransitionToHover, \
    EnrouteToTarget, Descent, SuicideBurn
from manager import StagingManager, PitchManager, ThrottleManager, BurnManager, EventManager


class Controller(SingletonMixin, object):
    pitch_follow = PitchManager.FIXED_UP

    current_state = None
    connection = None
    space_center = None

    _NextStateCls = None  # Expected class object hence the camel case

    _parts_in_stage = None
    _target_body = None

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
            ('velocity_direction', (
                self.space_center.transform_direction,
                (0, 1, 0),
                self.vessel.surface_velocity_reference_frame,
                self.vessel.surface_reference_frame)
             ),
            ('mach', (getattr, self.vessel.flight(), 'mach')),
            ('pitch', (getattr, self.vessel.flight(), 'pitch')),
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
            ('radius', (getattr, self.orbit, 'radius')),
            ('semi_major_axis', (getattr, self.orbit, 'semi_major_axis')),
            ('semi_minor_axis', (getattr, self.orbit, 'semi_minor_axis')),
            ('true_anomaly', (getattr, self.orbit, 'true_anomaly')),

            ('surface_gravity', (getattr, self.body, 'surface_gravity')),
            ('equatorial_radius', (getattr, self.body, 'equatorial_radius')),
            ('body_mass', (getattr, self.body, 'mass')),
            ('rotational_period', (getattr, self.body, 'rotational_period')),
        )

        self.G = self.space_center.g

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

        # def val_changed_callback(val, attr_name, ctrl):
        #     setattr(ctrl, attr_name, val)
        #
        # change_callback_funcs = (
        # )
        #
        # for attr_name, callback_args in change_callback_funcs:
        #     callback_stream = self.connection.add_stream(*callback_args)
        #     callback_stream.add_callback(partial(val_changed_callback, attr_name=attr_name, ctrl=self))
        #     callback_stream.start()

        self._parts_in_stage = {
            'all': defaultdict(list),
            'engine': defaultdict(list),
            'leg': defaultdict(list),
            # 'liquidfuel': defaultdict(list),
        }
        self._parts_in_decouple_stage = {
            'all': defaultdict(list),
            'engine': defaultdict(list),
            'leg': defaultdict(list),
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
            elif part.leg:
                self._parts_in_stage['leg'][stage].append(part)
                self._parts_in_decouple_stage['leg'][decouple_stage].append(part)

        for stage in range(self.current_stage, -2, -1):
            # Remove any engines that activate the same stage they decouple (the small srbs used for seperation)
            stage_parts = copy(self._parts_in_stage['engine'][stage])
            for part in stage_parts:
                if part in self._parts_in_decouple_stage['engine'][stage]:
                    # print 'removing... {}'.format(part)
                    self._parts_in_decouple_stage['engine'][stage].remove(part)
                    self._parts_in_stage['engine'][stage].remove(part)


        # for i in range(max([part.decouple_stage for part in self.vessel.parts.all]), -2, -1):
        #     parts_in_decouple_stage = self.vessel.parts.in_decouple_stage(i)
        #     self._parts_in_decouple_stage['all'][i] = parts_in_decouple_stage
        #
        #     for part in parts_in_decouple_stage:
        #         if part.engine:
        #             self._parts_in_decouple_stage['engine'][i].append(part)

    def event_test(self):
        print "I'M AN EVENT! LOOK AT ME!"

    def run(self):
        self.event_manager = EventManager()

        self.event_manager.subscribe('StagingManager.pre_stage', self.event_test)

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

        # if self.vessel.situation == self.space_center.VesselSituation.pre_launch:
        #     self._NextStateCls = PreLaunch
        # elif self.vessel.situation == self.space_center.VesselSituation.landed:
        #     self._NextStateCls = PreLaunch
        # elif self.vessel.situation == self.space_center.VesselSituation.orbiting:
        #     self._NextStateCls = CoastToInterceptBurn
        # else:
        #     self._NextStateCls = AscentState

        self._NextStateCls = Descent
        # self._NextStateCls = SuicideBurn
        # self._NextStateCls = TransitionToHover
        # self._NextStateCls = EnrouteToTarget

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

    @property
    def velocity_pitch(self):
        velocity_pitch, _ = get_pitch_heading(self.velocity_direction)
        return velocity_pitch

    @property
    def velocity_heading(self):
        _, velocity_heading = get_pitch_heading(self.velocity_direction)
        return velocity_heading

    @property
    def mach(self):
        return self._mach()

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

    def set_burn_dv_func(self, func):
        self.burn_manager.set_burn_dv_func(func)

    def set_burn_point_func(self, func):
        self.burn_manager.set_burn_point_func(func)

    def set_burn_point(self, point_in_time):
        self.burn_manager.set_burn_point(point_in_time)

    @property
    def burn_dv(self):
        return self.burn_manager.burn_dv

    @property
    def burn_point(self):
        return self.burn_manager.burn_point

    @property
    def burn_start(self):
        return self.burn_manager.burn_start

    def get_burn_start(self):
        return self.burn_start

    @property
    def burn_time(self):
        return self.burn_manager.burn_time

    def get_burn_time(self):
        return self.burn_time

    @property
    def target_body(self):
        if self._target_body is None:
            return self.body
        return self._target_body

    @target_body.setter
    def target_body(self, value):
        if value == self.body:
            value = None
        self._target_body = value

    @property
    def sphere_impact_points(self):
        impact_points = getattr(self, '_impact_points', None)

        print round(self.semi_major_axis, 2)
        if impact_points:
            # TODO should include semi minor too
            """Invalid ellipse on change of semi major axis"""
            if round(self.semi_major_axis, 2) != round(self._impact_sma, 2):
                impact_points = None
                print 'invalid!'

        if impact_points is None:
            print 'create!'
            self._impact_sma = self.semi_major_axis

            r = self.equatorial_radius
            a = self.semi_major_axis
            b = self.semi_minor_axis
            c = math.sqrt(a ** 2 - b ** 2)  # linear eccentricity

            elp = Ellipse(Point(c, 0), a, b)
            crc = Circle(Point(0, 0), r)
            self._impact_points = sorted(list(set([(float(point.x), float(point.y)) for point in elp.intersection(crc)])))
        return self._impact_points

    def get_NextStateCls(self):
        return self._NextStateCls

    def set_NextStateCls(self, NextStateCls):
        self._NextStateCls = NextStateCls

    def get_parts_of_type(self, part_type, key=None):
        parts = list(chain(*self._parts_in_stage[part_type].values()))
        if key is not None:
            # parts = [part for part in parts if try_default(key, args=[part], default=False, exceptions=[krpc.error.RPCError])]
            parts = [part for part in parts if key(part)]
        return parts

    def get_parts_in_stage(self, stage_num, part_type='all', key=None):
        # print 'get_parts_in_decouple_stage: {}'.format(part_type)
        # for x, val in self._parts_in_decouple_stage[part_type].items():
        #     print '*{} | {}'.format(x, val)
        # print 'get_parts_in_stage: {}'.format(part_type)
        # for x, val in self._parts_in_stage[part_type].items():
        #     print '^{} | {}'.format(x, val)
        parts = self._parts_in_stage[part_type][stage_num]
        if key is not None:
            # parts = [part for part in parts if try_default(key, args=[part], default=False, exceptions=[krpc.error.RPCError])]
            parts = [part for part in parts if key(part)]
        return parts

    def all_parts_after_stage(self, stage_num, part_type='all', key=None):
        all_stage_parts = []
        for curr_stage in range(stage_num, -2, -1):
            all_stage_parts.extend(self.get_parts_in_stage(curr_stage, part_type=part_type, key=key))
        return all_stage_parts

    def get_parts_in_decouple_stage(self, stage_num, part_type='all', key=None):
        # print 'get_parts_in_decouple_stage: {}'.format(part_type)
        # for x, val in self._parts_in_decouple_stage[part_type].items():
        #     print '*{} | {}'.format(x, val)
        # print 'get_parts_in_stage: {}'.format(part_type)
        # for x, val in self._parts_in_stage[part_type].items():
        #     print '^{} | {}'.format(x, val)
        parts = self._parts_in_decouple_stage[part_type][stage_num]
        if key is not None:
            # parts = [part for part in parts if try_default(key, args=[part], default=False, exceptions=[krpc.error.RPCError])]
            parts = [part for part in parts if key(part)]
        return parts

    def all_parts_after_decouple_stage(self, stage_num, part_type='all', key=None):
        all_decouple_stage_parts = []
        for curr_decouple_stage in range(stage_num, -2, -1):
            all_decouple_stage_parts.extend(self.get_parts_in_decouple_stage(curr_decouple_stage, part_type=part_type, key=key))
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

    # def get_resource_mass_for_stage(self, stage, resources=('LiquidFuel', 'Oxidizer')):
    #     resources_in_stage = self.vessel.resources_in_stage(stage, cumulative=False)
    #     mass = 0
    #     for resource_name in resources:
    #         resource_density = resources_in_stage.density(resource_name)
    #         resource_amount = resources_in_stage.amount(resource_name)
    #         mass += resource_density * resource_amount
    #     return mass

    def get_resource_mass_for_decouple_stage(self, decouple_stage, resources=('LiquidFuel', 'Oxidizer')):
        resources_in_decouple_stage = self.vessel \
            .resources_in_decouple_stage(decouple_stage, cumulative=False)
        mass = 0
        for resource_name in resources:
            resource_density = resources_in_decouple_stage.density(resource_name)
            resource_amount = resources_in_decouple_stage.amount(resource_name)
            print resource_name, '|', resource_amount
            mass += resource_density * resource_amount
        return mass

    def get_throttle(self):
        return self.throttle_manager.get_throttle()

    def set_throttle(self, val):
        self.throttle_manager.set_throttle(val)

    @property
    def engine_data(self):
        return self._engine_data

    @property
    def current_stage(self):
        # print
        # print '&&current_stage'
        # for part_type, stages in self._parts_in_stage.items():
        #     print part_type
        #     for stage, parts in stages.items():
        #         print '\t', stage, '|', parts
        return self.vessel.control.current_stage
        # return max(chain(*[stages.keys() for part_type, stages in self._parts_in_stage.items()]))

    @property
    def current_decouple_stage(self):
        # print
        # print '##current_decouple_stage'
        # for part_type, stages in self._parts_in_decouple_stage.items():
        #     print part_type
        #     for stage, parts in stages.items():
        #         print '\t', stage, '|', parts
        return max([key for part_type, stages in self._parts_in_decouple_stage.items() for key, value in stages.items() if value])

    def activate_next_stage(self):
        # stage_num = max(self.current_stage, self.current_decouple_stage)
        old_stage_num = self.vessel.control.current_stage

        active_engines_in_stage = [part for part in self._parts_in_stage['engine'][old_stage_num] if part.engine.active]
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

        for part in active_engines_in_stage:
            for part_type in self._parts_in_stage:
                # if part not in to_print[part.stage]:
                #     to_print[part.stage].append(part)
                # print old_stage_num, '|', part, '|', self._parts_in_stage[part_type]
                if part not in self._parts_in_stage[part_type][part.stage]:
                    self._parts_in_stage[part_type][part.stage].append(part)

        # print '**STAGE {}'.format(old_stage_num)
        # for key, val in to_print.items():
        #     print key, '**', val

        [stages.pop(old_stage_num, None) for part_type, stages in self._parts_in_stage.items()]
        [stages.pop(new_stage_num, None) for part_type, stages in self._parts_in_decouple_stage.items()]

    def get_local_gravity(self):
        mi = self.mass
        r = self.radius
        M = self.body_mass
        G = self.G
        return (G * (M+mi) / r**2)

    def get_avail_twr(self):
        return self.available_thrust / (self.mass * self.get_local_gravity())

    def get_twr(self):
        return self.thrust / (self.mass * self.get_local_gravity())

