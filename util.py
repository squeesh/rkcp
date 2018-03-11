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


def angle_between(v1, v2):
    # https://stackoverflow.com/questions/2827393/angles-between-two-n-dimensional-vectors-in-python
    # http://citadel.sjfc.edu/faculty/kgreen/vector/block1/vectors/node22.html
    """ Returns the angle in radians between vectors 'v1' and 'v2'::
            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    cross_prod_y = np.cross(v1, v2)[1]
    output = np.arccos(np.dot(np.array(v1_u), np.array(v2_u)))
    if cross_prod_y < 0:
        output = 2 * math.pi - output
    return output


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


# def get_dv_needed_for_circularization(ctrl=None):
#     if not ctrl:
#         from controller import Controller
#         ctrl = Controller.get()
#
#     # r_ap = current apoapsis radius in meters (and the altitude you will circularize at)
#     r_ap = ctrl.apoapsis
#     # r_pe = current periapsis radius in meters
#     r_pe = ctrl.periapsis
#     # mu = gravitational parameter for the body you are orbiting (3.5316x1012 m3 / s2 for Kerbin)
#     mu = ctrl.vessel.orbit.body.gravitational_parameter
#     # dv_circ_burn = sqrt(mu / r_ap) - sqrt((r_pe * mu) / (r_ap * (r_pe + r_ap) / 2))
#     return math.sqrt(mu / r_ap) - math.sqrt((r_pe * mu) / (r_ap * (r_pe + r_ap) / 2.0))


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


def get_seconds_to_impact(vi, dst, ctrl=None):
    if not ctrl:
        from controller import Controller
        ctrl = Controller.get()

    g = ctrl.surface_gravity
    v = math.sqrt(vi**2 + 2*g*dst)

    return (v-vi) / g


def get_speed_after_time(vi, t, ctrl=None):
    if not ctrl:
        from controller import Controller
        ctrl = Controller.get()
    g = ctrl.surface_gravity
    return vi + g * t


def get_speed_after_distance(vi, dst, ctrl=None):
    if not ctrl:
        from controller import Controller
        ctrl = Controller.get()
    g = ctrl.surface_gravity
    return math.sqrt(vi**2 + 2*g*dst)


def find_true_impact_time(vessel, ctrl=None):
    if not ctrl:
        from controller import Controller
        ctrl = Controller.get()

    ReferenceFrame = ctrl.space_center.ReferenceFrame

    ref_frame = ctrl.body.reference_frame
    flight = vessel.flight(ref_frame)

    base_time = get_seconds_to_impact(math.fabs(flight.speed) * 1.1, flight.surface_altitude, ctrl=ctrl)
    probe_time = base_time

    resolution_params = (
        (30.0, 15000),
        (10.0, 5000),
        (1.0,  500),
        (0.1,  50),
        (0.01, 5),
    )
    resolution_param_index = 0

    time_resolution, height_resolution = resolution_params[resolution_param_index]

    prev_probe_time = probe_time

    i = 0
    while True:
        probe_pos = vessel.orbit.position_at(ctrl.ut + probe_time, ref_frame)
        probe_pos_alt = ctrl.body.altitude_at_position(probe_pos, ref_frame)

        body_rot = math.pi * probe_time / ctrl.body.rotational_period
        futr_ref_frame = ReferenceFrame.create_relative(ref_frame, rotation=(0, math.sin(body_rot), 0, math.cos(body_rot)))

        probe_lat = ctrl.body.latitude_at_position(probe_pos, futr_ref_frame)
        probe_lng = ctrl.body.longitude_at_position(probe_pos, futr_ref_frame)

        surf_height = ctrl.body.surface_height(probe_lat, probe_lng)

        if probe_pos_alt - surf_height < height_resolution:
            probe_time = prev_probe_time
            resolution_param_index += 1

            if resolution_param_index >= len(resolution_params):
                break

            time_resolution, height_resolution = resolution_params[resolution_param_index]

        prev_probe_time = probe_time
        probe_time += time_resolution
        i += 1

    x1, y1, z1 = ctrl.body.surface_position(probe_lat, probe_lng, ref_frame)
    x2, y2, z2 = vessel.position(ref_frame)

    return probe_time, math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2), probe_pos, futr_ref_frame


def get_suicide_burn_time(vi, dist, use_local_g=False, ctrl=None):
    if not ctrl:
        from controller import Controller
        ctrl = Controller.get()

    mi = ctrl.vessel.mass
    isp = ctrl.vessel.specific_impulse

    if use_local_g:
        r = ctrl.vessel.orbit.radius
        M = ctrl.body.mass
        G = ctrl.space_center.g

        g = (G * (M+mi) / r**2)
    else:
        g = ctrl.body.surface_gravity
    thrust = ctrl.available_thrust

    bottom = (thrust/(g*isp))
    e_bit = (vi + math.sqrt(2*g*dist))/(g*isp)
    top = (mi - (mi/math.exp(e_bit)))
    return top / bottom


def get_suicide_burn_alt(vi, use_local_g=False, ctrl=None):
    if not ctrl:
        from controller import Controller
        ctrl = Controller.get()

    mi = ctrl.mass

    if use_local_g:
        r = ctrl.radius
        M = ctrl.body_mass
        G = ctrl.G

        g = (G * (M+mi) / r**2)
    else:
        g = ctrl.surface_gravity
    thrust = ctrl.available_thrust #* 0.7
    # print 't:', thrust, 'm:', vessel.mass, 't/m:', thrust/vessel.mass, 'm/t:', vessel.mass/thrust

    accel = thrust / mi - g
    # t = get_suicide_burn_time(vi, dist, use_local_g=use_local_g)

    # return (vi + (math.sqrt(2 * g * dist) / 2)) * t
    return (vi ** 2.0) / (2.0 * accel)


def to_min_sec_str(sec):
    minutes = int(sec / 60.0)
    seconds = int((sec / 60.0 - minutes) * 60.0)
    mili = int((((sec / 60.0 - minutes) * 60.0) - seconds) * 10)

    return '{}m {}.{}s'.format(minutes, seconds, mili)


def rotate_vector(vector, theta, axis):
    vector = np.asarray(vector)
    # axis = [axis[0], -axis[1], axis[2]]
    axis = np.asarray(axis)

    def rotation_matrix(axis, theta):
        """
        Return the rotation matrix associated with counterclockwise rotation about
        the given axis by theta radians.
        """
        axis = axis / math.sqrt(np.dot(axis, axis))
        a = math.cos(theta / 2.0)
        b, c, d = -axis * math.sin(theta / 2.0)
        aa, bb, cc, dd = a * a, b * b, c * c, d * d
        bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
        return np.array([
            [aa + bb - cc - dd, 2 * (bc + ad),      2 * (bd - ac)],
            [2 * (bc - ad),     aa + cc - bb - dd,  2 * (cd + ab)],
            [2 * (bd + ac),     2 * (cd - ab),      aa + dd - bb - cc],
        ])

    return np.dot(rotation_matrix(axis, theta), vector)
