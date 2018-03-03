from threading import Thread, Lock, Condition
from collections import defaultdict
from datetime import datetime, timedelta
import math
from time import sleep
from functools import partial

from util import get_pitch_heading, unit_vector, engine_is_active


# class CallbackManager(object):
#     _callbacks = defaultdict(list)
#
#     def __init__(self):
#         from controller import Controller
#         from callback import has_fuel_callback
#
#         ctrl = Controller.get()
#
#         for part in ctrl.vessel.parts.all:
#             if part.engine:
#                 self.register_callback('fuel', ctrl.connection.add_stream(getattr, part.engine, 'has_fuel'), has_fuel_callback)
#                 # e_stream = self.connection.add_stream(getattr, part.engine, 'has_fuel')
#                 # e_stream.add_callback(has_fuel_callback)
#                 # e_stream.start()
#
#     def register_callback(self, name, stream, callback):
#         stream.add_callback(callback)
#         stream.start()
#         self._callbacks[name].append(stream)


class PitchManager(object):
    FIXED_UP = 0
    FIXED_POINT = 1
    SURFACE_PROGRADE = 2
    ORBIT_PROGRADE = 3
    CUSTOM_PITCH_HEADING = 4
    curr_follow = FIXED_UP

    custom_pitch_heading = (90, 90)
    fixed_point_pitch = 85

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
                vessel.auto_pilot.target_pitch_and_heading(self.fixed_point_pitch, 90)
            elif follow == self.SURFACE_PROGRADE:
                # if vessel.flight().pitch > 30:
                # vessel.auto_pilot.reference_frame = vessel.surface_velocity_reference_frame
                # vessel.auto_pilot.target_direction = (0, 1, 0)

                velocity_direction = self.ctrl.space_center.transform_direction(
                    (0, 1, 0), vessel.surface_velocity_reference_frame, vessel.surface_reference_frame)

                pitch, heading = get_pitch_heading(velocity_direction)
                heading_error = (90 - heading) / 2.0
                # print 'PITCH: {}'.format(pitch)
                vessel.auto_pilot.reference_frame = vessel.surface_reference_frame
                vessel.auto_pilot.target_pitch_and_heading(pitch, 90 + heading_error)
                # else:
                #     vessel.auto_pilot.reference_frame = vessel.surface_reference_frame
                #     vessel.auto_pilot.target_pitch_and_heading(30, 90)
                # if vessel.auto_pilot.target_roll == 180:
                #     vessel.auto_pilot.target_pitch_and_heading(flight.pitch + flight.angle_of_attack, 90)
                # else:
                #     vessel.auto_pilot.target_pitch_and_heading(flight.pitch - flight.angle_of_attack, 90)
            elif follow == self.ORBIT_PROGRADE:
                # orb_prograde, _ = get_pitch_heading(flight.prograde)
                # vessel.auto_pilot.target_pitch_and_heading(orb_prograde, 90)
                flight = vessel.flight()
                vessel.auto_pilot.reference_frame = vessel.surface_reference_frame
                vessel.auto_pilot.target_direction = flight.prograde
            elif follow == self.CUSTOM_PITCH_HEADING:
                vessel.auto_pilot.reference_frame = vessel.surface_reference_frame
                vessel.auto_pilot.target_pitch_and_heading(*self.custom_pitch_heading)


class ThrottleManager(object):
    _old_throttle = 0
    _throttle = 0
    _throttle_queue = None
    disabled = False

    set_calls = 0
    avg_input = False

    def __init__(self):
        self._throttle_queue = []

        from controller import Controller
        self.ctrl = Controller.get()
        self._thread = Thread(target=self.run)
        self._thread.daemon = True
        self._thread.start()

    def run(self):
        # throttle_update_interval = timedelta(milliseconds=10)
        # last_throttle_update = datetime.now()
        last_second = datetime.now().second
        i = 0
        while True:
            # i += 1
            # curr_second = datetime.now().second
            # if curr_second != last_second:
            #     print
            #     # print 'tps:', i
            #     print 'que: {}'.format(len(self._throttle_queue))
            #     print self._throttle_queue[:20]
            #     print
            #     # print 'set:', self.set_calls
            #
            #     i = 0
            #     last_second = curr_second

            if not self.disabled:
                if not self.avg_input:
                    if math.fabs(self._throttle - self._old_throttle) > 0.001:
                        self.ctrl.vessel.control.throttle = self._throttle
                        self._old_throttle = self._throttle
                elif self._throttle_queue:
                    avg = sum(self._throttle_queue) / float(len(self._throttle_queue))
                    self.ctrl.vessel.control.throttle = avg
                    self._throttle_queue = []
                    sleep(0.1)


    def get_throttle(self):
        return self._throttle

    def set_throttle(self, val):
        self._throttle = val
        self._throttle_queue.append(val)


class BurnManager(object):
    _burn_dv = None
    _burn_dv_func = None
    _burn_times = []
    _burn_point = None
    _burn_point_func = None

    burning = False
    condition = None

    burn_dv = None
    burn_start = None
    burn_point = None
    burn_time = None

    def __init__(self):
        from controller import Controller
        self.ctrl = Controller.get()
        self.condition = Condition()
        self._thread = Thread(target=self.run)
        self._thread.daemon = True
        self._thread.start()

    # dv_circ_burn = get_dv_needed_for_circularization(self.ctrl)
    # burn_time_for_circle = get_burn_time_for_dv(dv_circ_burn, self.ctrl)
    # burn_start_time = self.ctrl.ut + self.ctrl.time_to_apoapsis - (burn_time_for_circle / 2.0)

    def run(self):
        conn = self.ctrl.connection
        ut_call = conn.get_call(getattr, conn.space_center, 'ut')
        # Create an expression on the server

        while True:
            if self._burn_times:
                if self.ctrl.ut > self._get_burn_start():
                    if self.burning == False:
                        self.burning = True
                        self.condition.acquire()

                    self.ctrl.throttle_manager.disabled = True
                    self.ctrl.vessel.control.throttle = 1.0

                    burn_until = self.ctrl.ut + self._burn_times[0]
                    print 'pre bt: {}'.format(self._burn_times)
                    self._burn_times = self._burn_times[1:]
                    print 'pst bt: {}'.format(self._burn_times)

                    if len(self._burn_times):
                        print 'waiting for burn end'
                        self.ctrl.staging_manager.pre_stage.acquire()
                        self.ctrl.staging_manager.post_stage.acquire()
                        self.ctrl.staging_manager.pre_stage.wait()
                        self.ctrl.staging_manager.pre_stage.release()
                        self.ctrl.vessel.control.throttle = 0.0
                        self.ctrl.staging_manager.post_stage.wait()
                        self.ctrl.staging_manager.post_stage.release()
                        print 'burn end done'
                    else:
                        expr = conn.krpc.Expression.greater_than(
                            conn.krpc.Expression.call(ut_call),
                            conn.krpc.Expression.constant_double(burn_until))

                        # Create an event from the expression
                        event = conn.krpc.add_event(expr)

                        # Wait on the event
                        with event.condition:
                            event.wait()

                    self.ctrl.vessel.control.throttle = 0.0

                    if len(self._burn_times):
                        # self.ctrl.vessel.control.activate_next_stage()
                        if len(self._burn_times) > 1 and self._burn_dv_func is not None and self._burn_point_func is not None:
                            self._burn_times = self._get_burn_times(self._get_burn_dv())

                            # self.ctrl.set_burn_point(self.get_burn_point())
                            print
                            print 'new burn times: {}'.format(self._burn_times)
                            print 'dv: {}'.format(self.burn_dv)
                            print 'ut: {}'.format(self.ctrl.ut)
                            print 'bs: {}'.format(self.burn_start)
                            print

            elif self.burning:
                self.burning = False
                self._burn_dv = None
                self._burn_dv_func = None
                self._burn_times = []
                self._burn_point = None
                self._burn_point_func = None
                self.burn_dv = None
                self.burn_start = None
                self.burn_point = None
                self.burn_time = None
                self.condition.notify_all()
                self.condition.release()
                print "BURN DONE!!!"

    def set_burn_dv(self, val):
        self._burn_dv = val
        self._burn_times = self._get_burn_times(val)

    def set_burn_dv_func(self, func):
        self._burn_dv_func = func

    def set_burn_point_func(self, func):
        self._burn_point_func = func

    def set_burn_point(self, val):
        self._burn_point = val

    def _get_dv_in_stage(self, stage):
        g = 9.8

        print '== stg: {}'.format(stage)
        active_engines = self.ctrl.get_parts_in_stage(
            self.ctrl.current_stage,
            'engine',
            key=lambda part: part.engine.active and part.decouple_stage < stage
        )
        decouple_stage = max([part.decouple_stage for part in (active_engines + self.ctrl.all_parts_after_stage(stage, 'engine'))])
        print '== decoup: {}'.format(decouple_stage)
        engines = list(set(
            active_engines +
            self.ctrl.all_parts_after_decouple_stage(
                decouple_stage, 'engine', key=lambda part: part.engine.active or part.stage == stage)
        ))

        print '== engs: {}'.format(engines)

        isp = self.ctrl.get_isp_for_engines(engines)
        print '== isp: {}'.format(isp)

        fuel_mass_in_stage = self.ctrl.get_resource_mass_for_decouple_stage(decouple_stage)
        print '== fuel in stg: {}'.format(fuel_mass_in_stage)

        m_i = sum([part.mass for part in self.ctrl.all_parts_after_decouple_stage(decouple_stage)])
        print '== m_i: {}'.format(m_i)
        m_f = m_i - fuel_mass_in_stage
        print '== m_f: {}'.format(m_f)

        # # print 'thrust: ', total_thrust
        # print 'dcp: ', decouple_stage
        # print 'isp: ', isp
        # print 'fuel mass: ', fuel_mass_in_stage
        # print 'ship mass: ', m_i
        # print 'final mass: ', m_f

        output = math.log(m_i / m_f) * isp * g
        print '== out: {} | {}'.format(stage, output)
        return output

    def _get_burn_times(self, dv):
        stage = self.ctrl.current_stage

        def get_burn_time_for_stage(current_dv, current_stage):
            active_engines = self.ctrl.get_parts_in_stage(
                self.ctrl.current_stage,
                'engine',
                key=lambda part: part.engine.active and part.decouple_stage < current_stage
            )
            decouple_stage = max(
                [part.decouple_stage for part in (active_engines + self.ctrl.all_parts_after_stage(current_stage, 'engine'))])
            print '--decoup: {}'.format(decouple_stage)
            engines = list(set(
                active_engines +
                self.ctrl.all_parts_after_decouple_stage(
                    decouple_stage, 'engine', key=lambda part: part.engine.active or part.stage == current_stage)
            ))

            print '--engs: {}'.format(engines)

            isp = self.ctrl.get_isp_for_engines(engines)
            print '--isp: {}'.format(isp)
            m_i = sum([part.mass for part in self.ctrl.all_parts_after_decouple_stage(decouple_stage)])
            print '--mass: {} | {}'.format(self.ctrl.mass, m_i)
            f = self.ctrl.get_thrust_for_engines(engines)
            print '--f: {}'.format(f)
            g = 9.8  # This is a constant, not -> self.ctrl.surface_gravity

            if isp > 0.0:
                m_f = m_i / math.exp(current_dv / (isp * g))
                flow_rate = f / (isp * g)
                if flow_rate > 0.0:
                    return (m_i - m_f) / flow_rate

            return None

        burn_times = []
        c_dv = dv
        for c_ds in range(stage, -2, -1):
            d_ds = self._get_dv_in_stage(c_ds)
            print 'c_ds: {} c_dv: {} d_ds: {}'.format(c_ds, c_dv, d_ds)
            if c_dv > d_ds:
                burn_times.append(get_burn_time_for_stage(d_ds, c_ds))
                c_dv -= d_ds
            else:
                burn_times.append(get_burn_time_for_stage(c_dv, c_ds))
                break

        # hack ... bug is putting Nones into the array... remove them
        print 'burn_times: {}'.format(burn_times)
        burn_times = [time for time in burn_times if (time is not None and time != 0.0)]
        print 'burn_times: {}'.format(burn_times)
        return burn_times

    def _get_burn_dv(self):
        if self._burn_dv_func is not None:
            self.burn_dv = self._burn_dv_func()
        else:
            self.burn_dv = self._burn_dv
        return self.burn_dv

    def _get_burn_point(self):
        if self._burn_point_func is not None:
            self.burn_point = self._burn_point_func()
        else:
            self.burn_point = self._burn_point or self.ctrl.ut
        return self.burn_point

    def _get_burn_start(self):
        if not self._burn_times:
            self._burn_times = self._get_burn_times(self._get_burn_dv())
        self.burn_start = self._get_burn_point() - self._get_burn_time() * 0.4
        return self.burn_start

    def _get_burn_time(self):
        if not self._burn_times:
            self.burn_time = 0
        else:
            self.burn_time = sum(self._burn_times) + ((len(self._burn_times) - 1) * 0.5)
        return self.burn_time


class StagingManager(object):
    fuel_lock = None
    pre_stage = None
    post_stage = None

    def __init__(self):
        from controller import Controller
        self.ctrl = Controller.get()
        self.pre_stage = Condition()
        self.post_stage = Condition()
        self.fuel_lock = Lock()
        self.decouple_stage_activate_next = {}

        for part in self.ctrl.engine_data:
            stream = self.ctrl.connection.add_stream(getattr, part.engine, 'has_fuel')
            stream.add_callback(partial(self.has_fuel_callback, part=part))
            stream.start()

    def has_fuel_callback(self, has_fuel, part):
        self.ctrl.engine_data[part]['has_fuel'] = has_fuel

        if not has_fuel:
            with self.fuel_lock:
                engines_to_decouple = self.ctrl.get_parts_in_decouple_stage(
                    self.ctrl.current_decouple_stage, 'engine', key=engine_is_active)

                if engines_to_decouple:
                    all_engines_flameout = not any([self.ctrl.engine_data[part]['has_fuel'] for part in engines_to_decouple])

                    if all_engines_flameout:
                        already_activated = self.decouple_stage_activate_next.get(self.ctrl.current_decouple_stage, False)
                        if not already_activated:
                            self.pre_stage.acquire()
                            sleep(0.1)
                            self.pre_stage.notify_all()
                            self.pre_stage.release()
                            self.decouple_stage_activate_next[self.ctrl.current_decouple_stage] = True
                            self.ctrl.activate_next_stage()
                            self.post_stage.acquire()
                            sleep(0.1)
                            self.post_stage.notify_all()
                            self.post_stage.release()

