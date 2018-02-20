# from multiprocessing import Lock
from threading import Thread, Lock
import math


# class SingletonMetaclass(type):
#     # __singleton_lock = Lock()
#     # __singleton_instance = None
#
#     def __init__(self, *args, **kwargs):
#         print 'init...'
#         super(SingletonMetaclass, self).__init__(*args, **kwargs)
#         self.__singleton_instance = None
#         self.__singleton_lock = Lock()
#         print 'lock: {}'.format(self.__singleton_lock)
#
#     def __call__(self, *args, **kwargs):
#         print 'get in: {} | {} | {}'.format(self, self.__singleton_lock, self.__singleton_instance)
#         if self.__singleton_instance is None:
#             with self.__singleton_lock:
#                 if self.__singleton_instance is None:
#                     self.__singleton_instance = super(SingletonMetaclass, self).__call__(*args, **kwargs)
#         print 'get out: {} | {} | {}'.format(self, self.__singleton_lock, self.__singleton_instance)
#         return self.__singleton_instance


class SingletonMixin(object):
    __singleton_lock = Lock()
    __singleton_instance = None


    @classmethod
    def get(cls):
        print 'get in: {} | {} | {}'.format(cls, cls.__singleton_lock, cls.__singleton_instance)
        if cls.__singleton_instance is None:
            with cls.__singleton_lock:
                if cls.__singleton_instance is None:
                    print 'starting...'
                    cls.__singleton_instance = cls()
                    # t = Thread(target=lambda: cls.__singleton_instance.start())
                    # t.daemon = True
                    # t.start()

        print 'get out: {} | {} | {}'.format(cls, cls.__singleton_lock, cls.__singleton_instance)
        return cls.__singleton_instance


def get_current_stage(vessel):
    return max([part.stage for part in vessel.parts.all])


def get_current_decouple_stage(vessel):
    return max([part.decouple_stage for part in vessel.parts.all])


def get_local_gravity(ctrl=None):
    if not ctrl:
        from controller import Controller
        ctrl = Controller.get()
    r = ctrl.equatorial_radius
    h = ctrl.altitude
    g = ctrl.surface_gravity
    return g * (r / (r + h))**2


def get_avail_twr(ctrl=None):
    if not ctrl:
        from controller import Controller
        ctrl = Controller.get()
    # stage = ctrl.current_stage
    # stage_parts = ctrl.get_parts_in_stage(stage)
    # engines_in_stage = ctrl.get_parts_in_stage(stage, 'engine')
    # avail_thrust_for_stage = sum([ctrl.get_available_thrust(part) for part in engines_in_stage])
    # avail_thrust = ctrl.available_thrust
    return ctrl.available_thrust / (ctrl.mass * get_local_gravity(ctrl))


def get_twr():
    from controller import Controller
    ctrl = Controller.get()
    stage = ctrl.current_stage
    stage_parts = ctrl.vessel.parts.in_stage(stage)
    thrust_for_stage = sum([part.engine.thrust for part in stage_parts if part.engine])
    return thrust_for_stage / (ctrl.vessel.mass * get_local_gravity())


def get_pitch_heading(direction):
    x, y, z = direction
    hdg = math.atan2(z, y) * 180 / math.pi
    if hdg <= 0:
        hdg = (hdg + 360)
    ptch = (90 - math.atan2(math.sqrt(z * z + y * y), x) * 180 / math.pi)
    return ptch, hdg


def get_dv_needed_for_circularization(ctrl=None):
    if not ctrl:
        from controller import Controller
        ctrl = Controller.get()

    # r_ap = current apoapsis radius in meters (and the altitude you will circularize at)
    r_ap = ctrl.apoapsis
    # r_pe = current periapsis radius in meters
    r_pe = ctrl.periapsis
    # mu = gravitational parameter for the body you are orbiting (3.5316x1012 m3 / s2 for Kerbin)
    mu = ctrl.vessel.orbit.body.gravitational_parameter
    # dv_circ_burn = sqrt(mu / r_ap) - sqrt((r_pe * mu) / (r_ap * (r_pe + r_ap) / 2))
    return math.sqrt(mu / r_ap) - math.sqrt((r_pe * mu) / (r_ap * (r_pe + r_ap) / 2.0))


def get_burn_time_for_dv(dv, ctrl=None):
    if not ctrl:
        from controller import Controller
        ctrl = Controller.get()

    m_i = ctrl.mass
    isp = ctrl.specific_impulse
    f = ctrl.available_thrust
    g = ctrl.surface_gravity

    if isp > 0.0:
        m_f = m_i / math.exp(dv / (isp * g))
        flow_rate = f / (isp * g)
        if flow_rate > 0.0:
            return (m_i - m_f) / flow_rate

    return 999999

