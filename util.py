# from multiprocessing import Lock
from threading import Thread, Lock
import math
import numpy as np


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
        print 'getting: {}'.format(cls.__singleton_instance)
        if cls.__singleton_instance is None:
            with cls.__singleton_lock:
                if cls.__singleton_instance is None:
                    cls.__singleton_instance = cls()

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
    return ctrl.available_thrust / (ctrl.mass * get_local_gravity(ctrl))


def get_twr(ctrl=None):
    if not ctrl:
        from controller import Controller
        ctrl = Controller.get()
    return ctrl.thrust / (ctrl.vessel.mass * get_local_gravity())


def get_pitch_heading(direction):
    x, y, z = direction
    hdg = math.atan2(z, y) * 180 / math.pi
    if hdg <= 0:
        hdg = (hdg + 360)
    ptch = (90 - math.atan2(math.sqrt(z * z + y * y), x) * 180 / math.pi)
    return ptch, hdg


def cross_product(u, v):
    return (u[1]*v[2] - u[2]*v[1],
            u[2]*v[0] - u[0]*v[2],
            u[0]*v[1] - u[1]*v[0])


def dot_product(u, v):
    return u[0]*v[0] + u[1]*v[1] + u[2]*v[2]


def magnitude(v):
    return math.sqrt(dot_product(v, v))


def angle_between_vectors(u, v):
    """ Compute the angle between vector u and v """
    dp = dot_product(u, v)
    if dp == 0:
        return 0
    um = magnitude(u)
    vm = magnitude(v)
    return math.acos(dp / (um*vm)) * (180. / math.pi)


def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    print 'VECTOR: {}'.format(vector)
    return vector / np.linalg.norm(vector)


def get_vessel_pitch_heading(vessel):
    vessel_direction = vessel.direction(vessel.surface_reference_frame)
    return get_pitch_heading(vessel_direction)

    # # Get the direction of the vessel in the horizon plane
    # horizon_direction = (0, vessel_direction[1], vessel_direction[2])
    #
    # # Compute the pitch - the angle between the vessels direction and
    # # the direction in the horizon plane
    # pitch = angle_between_vectors(vessel_direction, horizon_direction)
    # if vessel_direction[0] < 0:
    #     pitch = -pitch
    #
    # # Compute the heading - the angle between north and
    # # the direction in the horizon plane
    # north = (0, 1, 0)
    # heading = angle_between_vectors(north, horizon_direction)
    # if horizon_direction[2] < 0:
    #     heading = 360 - heading
    #
    # return pitch, heading


def get_pitch_heading(unit_vector):
    # vessel_direction = vessel.direction(vessel.surface_reference_frame)

    # Get the direction of the vessel in the horizon plane
    horizon_direction = (0, unit_vector[1], unit_vector[2])

    # Compute the pitch - the angle between the vessels direction and
    # the direction in the horizon plane
    pitch = angle_between_vectors(unit_vector, horizon_direction)
    if unit_vector[0] < 0:
        pitch = -pitch

    # Compute the heading - the angle between north and
    # the direction in the horizon plane
    north = (0, 1, 0)
    heading = angle_between_vectors(north, horizon_direction)
    if horizon_direction[2] < 0:
        heading = 360 - heading

    return pitch, heading


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


# def try_default(func, args=(), kwargs=None, default=None, exceptions=(Exception,)):
#     kwargs = {} if kwargs is None else kwargs
#     try:
#         return func(*args, **kwargs)
#     except exceptions:
#         return default


def engine_is_active(part):
    # print '-==-'
    # print part
    engine = part.engine
    # print engine
    active = engine.active
    # print active
    return active

