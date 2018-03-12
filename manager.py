from threading import Thread, Lock, Condition
from collections import defaultdict
from datetime import datetime, timedelta
import math
from time import sleep
from functools import partial
# from sympy import Ellipse, Circle, Point
import sympy

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


class EventManager(object):
    def __init__(self):
        self.events = {}
        self.threads = {}
        self.callbacks = defaultdict(list)

    def register(self, name, condition):
        assert name not in self.events

        self.events[name] = condition
        self.threads[name] = Thread(target=self.run, args=(name, condition,))
        self.threads[name].daemon = True
        self.threads[name].start()

    def subscribe(self, name, callback):
        self.callbacks[name].append(callback)

    def run(self, name, condition):
        while True:
            with condition:
                condition.wait()
                for callback in self.callbacks[name]:
                    thread = Thread(target=callback)
                    thread.start()


class Manager(object):
    _thread = None

    def __init__(self):
        from controller import Controller
        self.ctrl = Controller.get()
        self.init_state()
        self._thread = Thread(target=self.run, args=())
        self._thread.daemon = True
        self._thread.start()

    def init_state(self):
        pass


class PitchManager(Manager):
    FIXED_UP = 0
    FIXED_POINT = 1
    ASCENT_SURFACE_PROGRADE = 2
    SURFACE_RETROGRADE = 3
    ORBIT_PROGRADE = 4
    ORBIT_RETROGRADE = 5
    CUSTOM_PITCH_HEADING = 6
    curr_follow = FIXED_UP

    custom_pitch_heading = (90, 90)
    fixed_point_pitch = 85

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
            elif follow == self.ASCENT_SURFACE_PROGRADE:
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
            elif follow == self.SURFACE_RETROGRADE:
                velocity_direction = self.ctrl.space_center.transform_direction(
                    (0, -1, 0), vessel.surface_velocity_reference_frame, vessel.surface_reference_frame)

                pitch, heading = get_pitch_heading(velocity_direction)
                vessel.auto_pilot.reference_frame = vessel.surface_reference_frame
                vessel.auto_pilot.target_pitch_and_heading(pitch, heading)
                # vessel.auto_pilot.target_direction = velocity_direction
                # vessel.auto_pilot.target_pitch_and_heading(pitch, 90 + heading_error)


                # vessel.auto_pilot.reference_frame = vessel.surface_velocity_reference_frame
                # vessel.auto_pilot.target_direction = (0, -1, 0)

            elif follow == self.ORBIT_PROGRADE:
                # orb_prograde, _ = get_pitch_heading(flight.prograde)
                # vessel.auto_pilot.target_pitch_and_heading(orb_prograde, 90)
                flight = vessel.flight()
                vessel.auto_pilot.reference_frame = vessel.surface_reference_frame
                vessel.auto_pilot.target_direction = flight.prograde
            elif follow == self.ORBIT_RETROGRADE:
                # orb_prograde, _ = get_pitch_heading(flight.prograde)
                # vessel.auto_pilot.target_pitch_and_heading(orb_prograde, 90)
                flight = vessel.flight()
                vessel.auto_pilot.reference_frame = vessel.surface_reference_frame
                vessel.auto_pilot.target_direction = flight.retrograde
            elif follow == self.CUSTOM_PITCH_HEADING:
                vessel.auto_pilot.reference_frame = vessel.surface_reference_frame
                vessel.auto_pilot.target_pitch_and_heading(*self.custom_pitch_heading)


class ThrottleManager(Manager):
    _old_throttle = 0
    _throttle = 0
    _throttle_queue = None
    disabled = False

    set_calls = 0
    avg_input = False

    def init_state(self):
        self._throttle_queue = []

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
        if self.avg_input:
            self._throttle_queue.append(val)


class BurnManager(Manager):
    def __init__(self):
        super(BurnManager, self).__init__()
        self.burn_init = Condition()
        self.ctrl.event_manager.register('BurnManager.burn_init', self.burn_init)
        self.burn_begin = Condition()
        self.ctrl.event_manager.register('BurnManager.burn_begin', self.burn_begin)
        self.burn_end = Condition()
        self.ctrl.event_manager.register('BurnManager.burn_end', self.burn_end)

    def init_state(self):
        self._burn_dv_func = None
        self._burn_point_func = None
        self._burn_times = []
        self._burn_stop_event = None
        self._throttle_perms = defaultdict(lambda: True)
        self._throttle_disabled = False

        self.burn_dv = None
        self.burn_start = None
        self.burn_point = None
        self.burn_time = None

        self.burning = False
        self.recalc_burn_time = True

    def set_throttle(self, throttle, thread_name=None):
        if not self._throttle_disabled and self._throttle_perms[thread_name]:
            self.ctrl.vessel.control.throttle = throttle

    def run_burn(self):
        THREAD_NAME = 'run_burn'
        conn = self.ctrl.connection
        ut_call = conn.get_call(getattr, conn.space_center, 'ut')

        self.burn_end.acquire()
        while self._burn_times:
            self.set_throttle(1.0, THREAD_NAME)

            burn_until = self.ctrl.ut + self._burn_times[0]
            print 'pre bt: {}'.format(self._burn_times)
            self._burn_times = self._burn_times[1:]
            print 'pst bt: {}'.format(self._burn_times)

            if len(self._burn_times):
                # only the time of the last element of _burn_times is honored, the rest just mean "burn until stage"
                print 'waiting for stage'
                self.ctrl.staging_manager.pre_stage.acquire()
                self.ctrl.staging_manager.post_stage.acquire()
                self.ctrl.staging_manager.pre_stage.wait()
                self.ctrl.staging_manager.pre_stage.release()
                self.set_throttle(0.0, THREAD_NAME)
                self.ctrl.staging_manager.post_stage.wait()
                self.ctrl.staging_manager.post_stage.release()
                print 'stage done'

                print
                if self.recalc_burn_time and len(self._burn_times) > 1:
                    self._burn_times = self._get_burn_times(self._get_burn_dv())

                    # self.ctrl.set_burn_point(self.get_burn_point())

                    print 'new burn times: {}'.format(self._burn_times)

                # recalc burn start, to get more precise burns
                print 'os: {}'.format(self.ctrl.burn_start)
                burn_start = self._get_burn_start()
                print 'dv: {}'.format(self.burn_dv)
                print 'ut: {}'.format(self.ctrl.ut)
                print 'bs: {}'.format(burn_start)
                print

                burn_start_expr = conn.krpc.Expression.greater_than(
                    conn.krpc.Expression.call(ut_call),
                    conn.krpc.Expression.constant_double(burn_start))

                burn_start_event = conn.krpc.add_event(burn_start_expr)
                with burn_start_event.condition:
                    burn_start_event.wait()

            else:
                burn_expr = conn.krpc.Expression.greater_than(
                    conn.krpc.Expression.call(ut_call),
                    conn.krpc.Expression.constant_double(burn_until))
                burn_event = conn.krpc.add_event(burn_expr)

                with burn_event.condition:
                    burn_event.wait()

            self.set_throttle(0.0, THREAD_NAME)

        self.burn_end.notify_all()
        self.burn_end.release()

    def run(self):
        while True:
            if self._burn_dv_func is not None and self._burn_point_func is not None:
                print 'BURN INIT ACQ'
                self.burn_init.acquire()

                if not self._burn_times:
                    self._burn_times = self._get_burn_times(self._get_burn_dv())
                burn_start = self._get_burn_start()

                print 'BURN INIT NOTIFY'
                self.burn_init.notify_all()
                print 'BURN INIT REL'
                self.burn_init.release()
                print 'BURN INIT DONE'

                conn = self.ctrl.connection
                ut_call = conn.get_call(getattr, conn.space_center, 'ut')

                burn_start_expr = conn.krpc.Expression.greater_than(
                    conn.krpc.Expression.call(ut_call),
                    conn.krpc.Expression.constant_double(burn_start))

                # Create an event from the expression
                burn_start_event = conn.krpc.add_event(burn_start_expr)

                # Wait on the event
                print 'time: {} wait until: {}'.format(conn.space_center.ut, burn_start)
                with burn_start_event.condition:
                    burn_start_event.wait()
                    print 'wait done: {} | {}'.format(conn.space_center.ut, self.ctrl.ut)

                self.burn_begin.acquire()
                self.burning = True
                print 'BURN START...'
                self.burn_begin.notify_all()
                self.burn_begin.release()

                self.burn_end.acquire()
                burn_thread = Thread(target=self.run_burn)
                burn_thread.start()
                self.burn_end.wait()
                self.burn_end.release()

                self.init_state()
                print "BURN DONE!!!"

            # Burn manager needs to be set in advance, it only checks for a burn state once every 2 seconds
            sleep(2.0)
            #
            # if self._burn_times:
            #     if self.ctrl.ut > self._get_burn_start():
            #         if self.burning == False:
            #             self.burning = True
            #             print 'burn acquire ...'
            #             self.condition.acquire()
            #         elif self._burn_stop_event is not None:
            #             self._burn_stop_thread = Thread(target=self.burn_end_run)
            #             self._burn_stop_thread.daemon = True
            #             self._burn_stop_thread.start()
            #
            #         self.ctrl.throttle_manager.disabled = True
            #         self.ctrl.vessel.control.throttle = 1.0
            #
            #         burn_until = self.ctrl.ut + self._burn_times[0]
            #         print 'pre bt: {}'.format(self._burn_times)
            #         self._burn_times = self._burn_times[1:]
            #         print 'pst bt: {}'.format(self._burn_times)
            #
            #         if len(self._burn_times):
            #             print 'waiting for stage'
            #             self.ctrl.staging_manager.pre_stage.acquire()
            #             self.ctrl.staging_manager.post_stage.acquire()
            #             self.ctrl.staging_manager.pre_stage.wait()
            #             self.ctrl.staging_manager.pre_stage.release()
            #             self.ctrl.vessel.control.throttle = 0.0
            #             self.ctrl.staging_manager.post_stage.wait()
            #             self.ctrl.staging_manager.post_stage.release()
            #             print 'burn end done'
            #         else:
            #             expr = conn.krpc.Expression.greater_than(
            #                 conn.krpc.Expression.call(ut_call),
            #                 conn.krpc.Expression.constant_double(burn_until))
            #
            #             # Create an event from the expression
            #             event = conn.krpc.add_event(expr)
            #
            #             # Wait on the event
            #             with event.condition:
            #                 event.wait()
            #
            #         self.ctrl.vessel.control.throttle = 0.0
            #
            #         if len(self._burn_times):
            #             # self.ctrl.vessel.control.activate_next_stage()
            #             if len(self._burn_times) > 1 and self._burn_dv_func is not None and self._burn_point_func is not None:
            #                 self._burn_times = self._get_burn_times(self._get_burn_dv())
            #
            #                 # self.ctrl.set_burn_point(self.get_burn_point())
            #                 print
            #                 print 'new burn times: {}'.format(self._burn_times)
            #                 print 'dv: {}'.format(self.burn_dv)
            #                 print 'ut: {}'.format(self.ctrl.ut)
            #                 print 'bs: {}'.format(self.burn_start)
            #                 print
            #
            # elif self.burning:
            #     self.burning = False
            #     # self._burn_dv = None
            #     self._burn_dv_func = None
            #     self._burn_times = []
            #     # self._burn_point = None
            #     self._burn_point_func = None
            #     self.burn_dv = None
            #     self.burn_start = None
            #     self.burn_point = None
            #     self.burn_time = None
            #     self.force_stop = False
            #     print 'burn notify_all ...'
            #     self.condition.notify_all()
            #     print 'burn release ...'
            #     self.condition.release()
            #     print "BURN DONE!!!"

    # dv_circ_burn = get_dv_needed_for_circularization(self.ctrl)
    # burn_time_for_circle = get_burn_time_for_dv(dv_circ_burn, self.ctrl)
    # burn_start_time = self.ctrl.ut + self.ctrl.time_to_apoapsis - (burn_time_for_circle / 2.0)
    #
    # def burn_stop_run(self):
    #     with self._burn_stop_event.condition:
    #         self._burn_stop_event.wait(sum(self.burn_time or 1))
    #         self._burn_stop_event = None
    #         print 'FORCED BURN HALT'
    #         self.force_stop = True
    #
    # def run(self):
    #     conn = self.ctrl.connection
    #     ut_call = conn.get_call(getattr, conn.space_center, 'ut')
    #     # Create an expression on the server
    #
    #     while True:
    #         if not self.force_stop and self._burn_times:
    #             if self.ctrl.ut > self._get_burn_start():
    #                 if self.burning == False:
    #                     self.burning = True
    #                     print 'burn acquire ...'
    #                     self.condition.acquire()
    #                 elif self._burn_stop_event is not None:
    #                     self._burn_stop_thread = Thread(target=self.burn_end_run)
    #                     self._burn_stop_thread.daemon = True
    #                     self._burn_stop_thread.start()
    #
    #                 self.ctrl.throttle_manager.disabled = True
    #                 self.ctrl.vessel.control.throttle = 1.0
    #
    #                 burn_until = self.ctrl.ut + self._burn_times[0]
    #                 print 'pre bt: {}'.format(self._burn_times)
    #                 self._burn_times = self._burn_times[1:]
    #                 print 'pst bt: {}'.format(self._burn_times)
    #
    #                 if len(self._burn_times):
    #                     print 'waiting for stage'
    #                     self.ctrl.staging_manager.pre_stage.acquire()
    #                     self.ctrl.staging_manager.post_stage.acquire()
    #                     self.ctrl.staging_manager.pre_stage.wait()
    #                     self.ctrl.staging_manager.pre_stage.release()
    #                     self.ctrl.vessel.control.throttle = 0.0
    #                     self.ctrl.staging_manager.post_stage.wait()
    #                     self.ctrl.staging_manager.post_stage.release()
    #                     print 'burn end done'
    #                 else:
    #                     expr = conn.krpc.Expression.greater_than(
    #                         conn.krpc.Expression.call(ut_call),
    #                         conn.krpc.Expression.constant_double(burn_until))
    #
    #                     # Create an event from the expression
    #                     event = conn.krpc.add_event(expr)
    #
    #                     # Wait on the event
    #                     with event.condition:
    #                         event.wait()
    #
    #                 self.ctrl.vessel.control.throttle = 0.0
    #
    #                 if len(self._burn_times):
    #                     # self.ctrl.vessel.control.activate_next_stage()
    #                     if len(self._burn_times) > 1 and self._burn_dv_func is not None and self._burn_point_func is not None:
    #                         self._burn_times = self._get_burn_times(self._get_burn_dv())
    #
    #                         # self.ctrl.set_burn_point(self.get_burn_point())
    #                         print
    #                         print 'new burn times: {}'.format(self._burn_times)
    #                         print 'dv: {}'.format(self.burn_dv)
    #                         print 'ut: {}'.format(self.ctrl.ut)
    #                         print 'bs: {}'.format(self.burn_start)
    #                         print
    #
    #         elif self.burning:
    #             self.burning = False
    #             self._burn_dv = None
    #             self._burn_dv_func = None
    #             self._burn_times = []
    #             self._burn_point = None
    #             self._burn_point_func = None
    #             self.burn_dv = None
    #             self.burn_start = None
    #             self.burn_point = None
    #             self.burn_time = None
    #             self.force_stop = False
    #             print 'burn notify_all ...'
    #             self.condition.notify_all()
    #             print 'burn release ...'
    #             self.condition.release()
    #             print "BURN DONE!!!"

    def set_burn_dv(self, val):
        self.set_burn_dv_func(lambda: val)
        self.burn_dv = val

    def set_burn_dv_func(self, func):
        self._burn_dv_func = func

    def set_burn_point(self, val):
        self.set_burn_point_func(lambda: val)
        self.burn_point = val

    def set_burn_point_func(self, func):
        self._burn_point_func = func

    def burn_halt(self):
        self._throttle_disabled = True
        self.ctrl.vessel.control.throttle = 0.0

    def _get_dv_in_stage(self, stage):
        g = 9.8

        active_engines = self.ctrl.get_parts_in_stage(
            self.ctrl.current_stage,
            'engine',
            key=lambda part: part.engine.active and part.decouple_stage < stage
        )
        decouple_stages = [part.decouple_stage for part in (active_engines + self.ctrl.all_parts_after_stage(stage, 'engine'))]
        if decouple_stages:
            decouple_stage = max(decouple_stages)
        else:
            decouple_stage = stage

        engines = list(set(
            active_engines +
            self.ctrl.all_parts_after_decouple_stage(
                decouple_stage, 'engine', key=lambda part: part.engine.active or part.stage == stage)
        ))

        isp = self.ctrl.get_isp_for_engines(engines)
        fuel_mass_in_stage = self.ctrl.get_resource_mass_for_decouple_stage(decouple_stage)
        m_i = sum([part.mass for part in self.ctrl.all_parts_after_decouple_stage(decouple_stage)])
        m_f = m_i - fuel_mass_in_stage

        return math.log(m_i / m_f) * isp * g

    def _get_burn_times(self, dv):
        stage = self.ctrl.current_stage

        def get_burn_time_for_stage(current_dv, current_stage):
            active_engines = self.ctrl.get_parts_in_stage(
                self.ctrl.current_stage,
                'engine',
                key=lambda part: part.engine.active and part.decouple_stage < current_stage
            )
            decouple_stages = [part.decouple_stage for part in (active_engines + self.ctrl.all_parts_after_stage(current_stage, 'engine'))]
            if decouple_stages:
                decouple_stage = max(decouple_stages)
            else:
                decouple_stage = stage

            engines = list(set(
                active_engines +
                self.ctrl.all_parts_after_decouple_stage(
                    decouple_stage, 'engine', key=lambda part: part.engine.active or part.stage == current_stage)
            ))

            isp = self.ctrl.get_isp_for_engines(engines)
            m_i = sum([part.mass for part in self.ctrl.all_parts_after_decouple_stage(decouple_stage)])
            f = self.ctrl.get_thrust_for_engines(engines)
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
            if c_dv > d_ds:
                burn_times.append(get_burn_time_for_stage(d_ds, c_ds))
                c_dv -= d_ds
            else:
                burn_times.append(get_burn_time_for_stage(c_dv, c_ds))
                break

        # hack ... bug is putting Nones into the array... remove them # TODO: this might be fixed?
        return [time for time in burn_times if (time is not None and time != 0.0)]

    def _get_burn_dv(self):
        # if self._burn_dv_func is not None:
        self.burn_dv = self._burn_dv_func()
        # else:
        #     self.burn_dv = self._burn_dv
        return self.burn_dv

    def _get_burn_point(self):
        # if self._burn_point_func is not None:
        self.burn_point = self._burn_point_func()
        # else:
        #     self.burn_point = self._burn_point or self.ctrl.ut
        return self.burn_point

    def _get_burn_start(self):
        if not self._burn_times:
            self._burn_times = self._get_burn_times(self._get_burn_dv())
        self.burn_start = self._get_burn_point() - self._get_burn_time() * 0.4
        return self.burn_start

    def _get_burn_time(self):
        if not self._burn_times:
            self._burn_times = self._get_burn_times(self._get_burn_dv())
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
        self.ctrl.event_manager.register('StagingManager.pre_stage', self.pre_stage)

        self.post_stage = Condition()
        self.ctrl.event_manager.register('StagingManager.post_stage', self.post_stage)

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
                print 'in lock'
                engines_to_decouple = self.ctrl.get_parts_in_decouple_stage(
                    self.ctrl.current_decouple_stage, 'engine', key=engine_is_active)

                print 'eng_to_dc:', engines_to_decouple
                if engines_to_decouple:
                    all_engines_flameout = not any([self.ctrl.engine_data[part]['has_fuel'] for part in engines_to_decouple])

                    if all_engines_flameout:
                        already_activated = self.decouple_stage_activate_next.get(self.ctrl.current_decouple_stage, False)
                        if not already_activated:
                            print 'pre_stage acquired'
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


class ImpactManager(Manager):
    _semi_major_axis = 0.0
    _semi_minor_axis = 0.0

    _impact_pos = None
    _impact_true_anomaly = None

    def __init__(self):
        super(ImpactManager, self).__init__()

        I, D, sin, cos, sqrt = sympy.Integral, sympy.Derivative, sympy.sin, sympy.cos, sympy.sqrt
        self.integral_symbols = sympy.symbols('a b t tmin tmax')
        a, b, t, tmin, tmax = self.integral_symbols
        self.integral_func = I(sqrt(D(a * cos(t), t, evaluate=True) ** 2 + D(b * sin(t), t, evaluate=True) ** 2), (t, tmin, tmax))

    def run(self):

        def svg(e, stroke_width=1., color="#000", fill_color="transparent"):
            """Returns SVG ellipse element for the Ellipse.

            Parameters
            ==========

            scale_factor : float
                Multiplication factor for the SVG stroke-width.  Default is 1.
            fill_color : str, optional
                Hex string for fill color. Default is "#66cc99".
            """

            from sympy.core.evalf import N

            c = N(e.center)
            h, v = N(e.hradius), N(e.vradius)
            return (
                '<ellipse stroke="{}" fill="{}" stroke-width="{}"'
                ' opacity="1.0" cx="{}" cy="{}" rx="{}" ry="{}"/>'
            ).format(color, fill_color, stroke_width, c.x, c.y, h, v)

        def correct_angle(theta):
            return math.pi - theta

        while True:
            sleep(1.0)

            if all([self.ellipse, self.circle]):

                POINT_SIZE = self.circle.hradius*0.05
                STROKE_WIDTH = self.circle.hradius * 0.025
                TEXT_SCALE = 1000.0

                lines = []

                lines.append('<html><head><meta http-equiv="Refresh" content="2"></head><body>')
                x1 = int(-self.circle.hradius * 1.25)
                y1 = min(int(-self.ellipse.vradius * 1.25), int(-self.circle.vradius * 1.25))
                x2 = int(self.ellipse.hradius * 1.25 * 2)
                y2 = max(int(self.ellipse.vradius * 1.25), int(self.circle.vradius * 1.25)) + math.fabs(y1)
                lines.append('<svg viewBox="{} {} {} {}" style="width: 80%">'.format(x1, y1, x2, y2))
                lines.append(svg(self.ellipse, stroke_width=STROKE_WIDTH))
                lines.append(svg(self.circle, stroke_width=STROKE_WIDTH))
                lines.append(svg(sympy.Circle(self.focii, POINT_SIZE), color='#0000FF', fill_color='#0000FF'))
                lines.append(svg(sympy.Circle(self.ellipse.center, POINT_SIZE), color='#BBBB00', fill_color='#BBBB00'))
                if self._impact_pos is not None:
                    lines.append(svg(sympy.Circle(self._impact_pos, POINT_SIZE), color='#FF0000', fill_color='#FF0000'))

                if self._impact_true_anomaly is not None:
                    lines.append('<line x1="{}" y1="{}" x2="{}" y2="{}" stroke="{}" stroke-width="{}" transform="rotate({})" />'.format(
                        0, 0, self.circle.hradius, 0, '#FF0000', STROKE_WIDTH,
                        math.degrees(correct_angle(self._impact_true_anomaly)),
                    ))

                vessel_pos = self.ctrl.impact_manager.calc_pos(orbit=self.ctrl.orbit)
                lines.append(svg(sympy.Circle(vessel_pos, POINT_SIZE), color='#00FF00', fill_color='#00FF00'))
                lines.append('<line x1="{}" y1="{}" x2="{}" y2="{}" stroke="{}" stroke-width="{}" transform="rotate({})" />'.format(
                    0, 0, vessel_pos.radius, 0, '#00FF00', STROKE_WIDTH,
                    math.degrees(correct_angle(vessel_pos.true_anomaly)),
                ))
                lines.append('<text x="{}" y="{}" fill="{}" font-size="{}" transform="scale({})">{}</text>'.format(
                    -50000 / TEXT_SCALE, 50000 / TEXT_SCALE, '#00FF00', 50,
                    TEXT_SCALE, round(math.degrees(vessel_pos.true_anomaly), 3),
                ))

                vessel_center_theta = self.ctrl.impact_manager.calc_center_theta(pos=vessel_pos)
                lines.append('<line x1="{}" y1="{}" x2="{}" y2="{}" stroke="{}" stroke-width="{}" transform="rotate({} {} {})" />'.format(
                    float(self.ellipse.center.x), 0, float(self.ellipse.center.x) + self.ellipse.hradius, 0, '#00FFFF', STROKE_WIDTH,
                    math.degrees(correct_angle(vessel_center_theta)), float(self.ellipse.center.x), 0,
                ))
                lines.append('<text x="{}" y="{}" fill="{}" font-size="{}" transform="scale({})">{}</text>'.format(
                    (float(self.ellipse.center.x) - 50000) / TEXT_SCALE, 50000 / TEXT_SCALE, '#00FFFF', 50,
                    TEXT_SCALE, round(math.degrees(vessel_center_theta), 3),
                ))

                lines.append('</svg>')
                lines.append('</body></html>')

                f = open('orbit_svg.htm', 'w+')
                f.write(''.join(lines))
                f.close()
                print 'file created'
                sleep(5.0)

    def itegerate_distant(self,
        pos_1=None, orbit_1=None, theta_1=None,
        pos_2=None, orbit_2=None, theta_2=None,
        n=50,
    ):
        assert (pos_1 or orbit_1 or theta_1) and (pos_2 or orbit_2 or theta_2)

        # ASSUMPTION: theta_1 is ALWAYS < 90deg
        if theta_1 is None:
            kwargs_1 = {'pos': pos_1} if pos_1 is not None else {'orbit': orbit_1}
            theta_1 = self.calc_center_theta(**kwargs_1)

        if theta_2 is None:
            kwargs_2 = {'pos': pos_2} if pos_2 is not None else {'orbit': orbit_2}
            theta_2 = self.calc_center_theta(**kwargs_2)

        theta_1 = -theta_1
        theta_2 = -theta_2

        if theta_1 < 0 or theta_1 > theta_2 or theta_2 > math.pi:
            return -1.0

        print 'imp thet: ', math.degrees(theta_1), '|', theta_1
        print 'ves thet: ', math.degrees(theta_2), '|', theta_2

        half_pi = math.pi/2.0
        theta_extra = None
        if theta_2 > half_pi:
            theta_extra = half_pi - (theta_2 - half_pi)
            theta_2 = half_pi
            # dist_offset += self.quater_arc_length

        print 'theta_2: ', math.degrees(theta_2), '|', theta_2
        print 'theta_e: ', math.degrees(theta_extra or 0), '|', theta_extra

        def get_integral_dist(theta_a, theta_b):
            a, b, t, tmin, tmax = self.integral_symbols

            integral_kwargs = {
                a: self._semi_major_axis,
                b: self._semi_minor_axis,
                tmin: math.atan(self._semi_major_axis * math.tan(theta_a) / self._semi_minor_axis),
                tmax: math.atan(self._semi_major_axis * math.tan(theta_b) / self._semi_minor_axis),
            }

            integral_func = self.integral_func.subs(integral_kwargs)
            return math.fabs(integral_func.as_sum(n, 'trapezoid'))

        base_integral_dist = getattr(self, '_base_integral_dist', None)

        if base_integral_dist and (
            round(self.ctrl.semi_major_axis, 2) != round(self._semi_major_axis, 2) or
            round(self.ctrl.semi_minor_axis, 2) != round(self._semi_minor_axis, 2)
        ):
            base_integral_dist = None

        if theta_extra is None or base_integral_dist is None:
            base_integral_dist = get_integral_dist(theta_1, theta_2)
            print 'base_dist: ', base_integral_dist

        extra_dist = 0
        if theta_extra is not None:
            extra_dist = get_integral_dist(theta_extra, half_pi)
            print 'extr_dist: ', extra_dist

        self._base_integral_dist = base_integral_dist
        self._integral_dist = base_integral_dist + extra_dist
        return self._integral_dist

    def time_to_impact(self):
        time_dist_pos = getattr(self, '_time_dist_pos', None)

        if time_dist_pos and (
            round(self.ctrl.semi_major_axis, 2) != round(self._semi_major_axis, 2) or
            round(self.ctrl.semi_minor_axis, 2) != round(self._semi_minor_axis, 2)
        ):
            """Invalid ellipse on change of semi major / minor axis"""
            time_dist_pos = None
            print 'invalid time_dist_pos'

        if time_dist_pos is None:

            ReferenceFrame = self.ctrl.ReferenceFrame
            base_ut = self.ctrl.ut

            ref_frame = self.ctrl.body.reference_frame
            period = self.ctrl.rotational_period

            RESOLUTION_PARAMS = (
                1.0,
                0.5,
                0.25,
                0.125,
                0.0625,
            )
            resolution_param_index = 0

            time_resolution = RESOLUTION_PARAMS[resolution_param_index]

            base_time, base_pos = self.sphere_intersection_time_pos(cache=False)
            base_time += .1
            probe_time = base_time - base_ut
            prev_probe_time = probe_time

            i = 0
            while True:
                body_rot = math.pi * probe_time / period
                futr_ref_frame = ReferenceFrame.create_relative(ref_frame, rotation=(0, math.sin(body_rot), 0, math.cos(body_rot)))

                probe_pos = self.ctrl.vessel.orbit.position_at(base_ut + probe_time, ref_frame)
                probe_pos_alt = self.ctrl.body.altitude_at_position(probe_pos, ref_frame)

                probe_lat = self.ctrl.body.latitude_at_position(probe_pos, futr_ref_frame)
                probe_lng = self.ctrl.body.longitude_at_position(probe_pos, futr_ref_frame)

                surf_height = self.ctrl.body.surface_height(probe_lat, probe_lng)

                if surf_height - probe_pos_alt < 0:
                    probe_time = prev_probe_time
                    resolution_param_index += 1

                    if resolution_param_index >= len(RESOLUTION_PARAMS):
                        break

                    time_resolution = RESOLUTION_PARAMS[resolution_param_index]

                prev_probe_time = probe_time
                probe_time -= time_resolution
                i += 1

            # x1, y1, z1 = probe_pos
            # x2, y2, z2 = self.ctrl.vessel.position(ref_frame)
            self._time_dist_pos = (base_ut + probe_time, probe_pos)

            # vessel_theta = self.calc_center_theta(orbit=self.ctrl.orbit)
            # impact_theta = self.calc_center_theta(pos=self._impact_pos)
            #
            # print 'ves thet: ', math.degrees(vessel_theta), '|', vessel_theta
            # print 'imp thet: ', math.degrees(impact_theta), '|', impact_theta
            #
            # n = 50
            #
            # a, b, t, tmin, tmax = self.integral_symbols
            #
            # integral_kwargs = {
            #     a: self._semi_major_axis,
            #     b: self._semi_minor_axis,
            #     tmin: math.atan(self._semi_major_axis * math.tan(vessel_theta) / self._semi_minor_axis),
            #     tmax: math.atan(self._semi_major_axis * math.tan(impact_theta) / self._semi_minor_axis),
            # }
            #
            # integral_func = self.integral_func.subs(integral_kwargs)
            # self._integral_dist = math.fabs(integral_func.as_sum(n, 'trapezoid'))


        return self._time_dist_pos

    def calc_true_anomaly(self, x):
        x_to_p = float(x.distance(self.periapsis))
        x_to_f = float(x.distance(self.focii))
        p_to_f = self.a - self.c
        output = math.acos((x_to_f ** 2 + p_to_f ** 2 - x_to_p ** 2) / (2.0 * x_to_f * p_to_f))
        if x.y < 0:
            output = -output
        return output

    def calc_pos(self, orbit=None, radius=None, true_anomaly=None):
        assert orbit or (radius and true_anomaly)
        if orbit is not None:
            radius = orbit.radius
            true_anomaly = orbit.true_anomaly

        theta = true_anomaly + math.pi
        output = sympy.Point(
            radius * math.cos(theta),
            radius * -math.sin(theta),
        )

        # FOR DEBUG SVG
        output.radius = radius
        output.true_anomaly = true_anomaly

        return output

    def calc_center_theta(self, pos=None, orbit=None, radius=None, true_anomaly=None):
        assert pos or orbit or (radius and true_anomaly)
        if not pos:
            pos = self.calc_pos(orbit=orbit, radius=radius, true_anomaly=true_anomaly)

        pos_to_f = float(pos.distance(self.focii))
        pos_to_center = float(pos.distance(self.ellipse.center))

        # In our coordinate system, negative rotations are the "norm" so make this negative...
        return -math.acos((pos_to_center ** 2 + self.c ** 2 - pos_to_f ** 2) / (2.0 * pos_to_center * self.c))

    def sphere_intersection_time_pos(self, cache=True):
        impact_time_pos = getattr(self, '_impact_time_pos', None)

        if impact_time_pos and (
            round(self.ctrl.semi_major_axis, 2) != round(self._semi_major_axis, 2) or
            round(self.ctrl.semi_minor_axis, 2) != round(self._semi_minor_axis, 2)
        ):
            """Invalid ellipse on change of semi major / minor axis"""
            impact_time_pos = None
            print 'invalid intersection_time_pos'

        if impact_time_pos is None or not cache:
            print 'create intersection_time_pos!'
            inter_points = self.sphere_intersection_points(cache=False)
            theta_list = []

            for intersection in inter_points:
                # I think the math.pi part is incorrect, perhaps this doesn't handel oblique triangles
                impact_pos = sympy.Point(*intersection)
                theta = math.pi - self.calc_true_anomaly(impact_pos) - 2 * math.pi
                if theta >= self.true_anomaly:
                    theta_list.append((theta, impact_pos))

            if theta_list:
                impact_true_anomaly, self._impact_pos = sorted(theta_list)[0]
                self._impact_true_anomaly = impact_true_anomaly
                impact_time = self.ctrl.orbit.ut_at_true_anomaly(impact_true_anomaly)
                impact_pos = self.ctrl.orbit.position_at(impact_time, self.ctrl.body.non_rotating_reference_frame)
                self._impact_time_pos = (impact_time, impact_pos)
            else:
                self._impact_time_pos = (-1, None)

        return self._impact_time_pos

    def sphere_intersection_points(self, cache=True):
        intersection_points = getattr(self, '_intersection_points', None)

        if intersection_points and (
            round(self.ctrl.semi_major_axis, 2) != round(self._semi_major_axis, 2) or
            round(self.ctrl.semi_minor_axis, 2) != round(self._semi_minor_axis, 2)
        ):
            # TODO should include semi minor too
            """Invalid ellipse on change of semi major / minor axis"""
            intersection_points = None
            print 'invalid intersection points'

        if intersection_points is None or not cache:
            print 'create intersection points!'
            self.init_vars()
            self._intersection_points = \
                sorted(list(set([(float(point.x), float(point.y)) for point in self.ellipse.intersection(self.circle)])))
        return self._intersection_points

    def init_vars(self):
        self._semi_major_axis = self.ctrl.semi_major_axis
        self._semi_minor_axis = self.ctrl.semi_minor_axis

        self.r = self.ctrl.equatorial_radius
        self.a = self._semi_major_axis
        self.b = self._semi_minor_axis
        self.c = math.sqrt(self.a**2 - self.b**2)  # linear eccentricity

        self.periapsis = sympy.Point(self.a - self.c, 0)
        self.true_anomaly = self.ctrl.true_anomaly
        self.focii = sympy.Point(0, 0)
        self.ellipse = sympy.Ellipse(sympy.Point(self.c, 0), self.a, self.b)
        self.circle = sympy.Circle(self.focii, self.r)

        a, b, t, tmin, tmax = self.integral_symbols

        integral_kwargs = {
            a: self._semi_major_axis,
            b: self._semi_minor_axis,
            tmin: math.atan(self._semi_major_axis * math.tan(0.0) / self._semi_minor_axis),
            tmax: math.atan(self._semi_major_axis * math.tan(math.pi/2.0) / self._semi_minor_axis),
        }

        integral_func = self.integral_func.subs(integral_kwargs)
        self.quater_arc_length = math.fabs(integral_func.as_sum(50, 'trapezoid'))

