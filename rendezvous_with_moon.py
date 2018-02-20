import time
import krpc
from datetime import datetime
from functools import partial
import math
import numpy as np
from time import sleep
from multiprocessing import Lock


def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)


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


def get_local_gravity(vessel):
    body = vessel.orbit.body
    r = body.equatorial_radius
    h = vessel.flight().mean_altitude
    g = body.surface_gravity
    return g * (r / (r + h))**2


def get_pitch_heading(direction):
    x, y, z = direction
    hdg = math.atan2(z, y) * 180 / math.pi
    if hdg <= 0:
        hdg = (hdg + 360)
    ptch = (90 - math.atan2(math.sqrt(z * z + y * y), x) * 180 / math.pi)
    return ptch, hdg


def get_dv_needed_for_circularization(vessel):
    # r_ap = current apoapsis radius in meters (and the altitude you will circularize at)
    r_ap = vessel.orbit.apoapsis
    # r_pe = current periapsis radius in meters
    r_pe = vessel.orbit.periapsis
    # mu = gravitational parameter for the body you are orbiting (3.5316x1012 m3 / s2 for Kerbin)
    mu = vessel.orbit.body.gravitational_parameter
    # dv_circ_burn = sqrt(mu / r_ap) - sqrt((r_pe * mu) / (r_ap * (r_pe + r_ap) / 2))
    return math.sqrt(mu / r_ap) - math.sqrt((r_pe * mu) / (r_ap * (r_pe + r_ap) / 2))


def get_burn_time_for_dv(dv, vessel):
    minit = vessel.mass
    isp = vessel.specific_impulse
    g0 = vessel.orbit.body.surface_gravity

    if isp > 0.0:
        mfinal = minit * math.exp(-dv / (isp * g0))
        mpropellent = minit - mfinal
        mdot = vessel.available_thrust / (isp * g0)
        if mdot > 0.0:
            return mpropellent / mdot

    return 999999


#STATES
# ASCENT = 1
# COAST_TO_APOAPSIS = 2
# CIRCULARIZE = 3
# IN_ORBIT = 4
# curr_state = ASCENT

ORBIT_PROGRADE = 0
ORBIT_RETROGRADE = 1
SURFACE_PROGRADE = 2
SURFACE_RETROGRADE = 3
FIXED_UP = 4
curr_follow = ORBIT_PROGRADE

def set_state(new_state):
    global curr_state
    curr_state = new_state
    print 'change state:', new_state


conn = krpc.connect(
    name='My Example Program',
    address='127.0.0.1',
    rpc_port=50000, stream_port=50001)
space_center = conn.space_center
vessel = space_center.active_vessel
active_body = vessel.orbit.body
body_radius = active_body.equatorial_radius

sun = space_center.bodies['Sun']
kerbin = space_center.bodies['Kerbin']
mun = space_center.bodies['Mun']

ReferenceFrame = sun.reference_frame.__class__

M = mun.mass
G = space_center.g

# States
COAST_TO_BURN = 0
MUN_BURN = 1
ENROUTE_TO_MUN = 2
MUN_DESCENT = 3
SUICIDE_BURN = 4
LAND = 5
FIN = 99
curr_state = MUN_DESCENT

ut = conn.space_center.ut

def ut_callback(x):
    global ut
    ut = x

    if curr_follow == FIXED_UP:
        vessel.auto_pilot.reference_frame = vessel.surface_reference_frame
        vessel.auto_pilot.target_pitch_and_heading(90, 90)

    elif curr_follow == ORBIT_PROGRADE:
        # orb_prograde, orb_heading = get_pitch_heading(flight.prograde)
        # vessel.auto_pilot.target_pitch_and_heading(orb_prograde, orb_heading)
        # vessel.auto_pilot.reference_frame = vessel.surface_velocity_reference_frame
        vessel.auto_pilot.reference_frame = vessel.surface_reference_frame
        vessel.auto_pilot.target_direction = vessel.flight().prograde

    elif curr_follow == ORBIT_RETROGRADE:
        # orb_retrograde, orb_heading = get_pitch_heading(flight.retrograde)
        # vessel.auto_pilot.target_pitch_and_heading(orb_retrograde, orb_heading)
        vessel.auto_pilot.reference_frame = vessel.surface_reference_frame
        vessel.auto_pilot.target_direction = vessel.flight().prograde

    elif curr_follow == SURFACE_PROGRADE:
        # vessel.auto_pilot.target_pitch_and_heading(flight.pitch - flight.angle_of_attack, flight.heading)
        vessel.auto_pilot.reference_frame = vessel.surface_velocity_reference_frame
        vessel.auto_pilot.target_direction = (0, 1, 0)

    elif curr_follow == SURFACE_RETROGRADE:
        vessel.auto_pilot.reference_frame = vessel.surface_velocity_reference_frame
        vessel.auto_pilot.target_direction = (0, -1, 0)

# Set up streams for telemetry
# ut = conn.add_stream(getattr, conn.space_center, 'ut')
ut_stream = conn.add_stream(getattr, conn.space_center, 'ut')
ut_stream.add_callback(ut_callback)
ut_stream.start()

# flight = vessel.flight()
altitude = 0
burn_until = None

vessel.control.sas = True
vessel.control.rcs = False
vessel.control.throttle = 0.0

# Activate the first stage
vessel.auto_pilot.engage()
vessel.auto_pilot.target_roll = 0

lines = []

def altitude_callback(x):
    global line
    global altitude
    altitude = x

    # sleep(0.5)
altitude_stream = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
altitude_stream.add_callback(altitude_callback)
altitude_stream.start()


def lat_lng_str(cur_lat, cur_lng):
    def to_hms(x):
        x = math.fabs(x)
        hours = int(x)
        minutes = int((x - hours) * 60)
        seconds = int(((((x - hours) * 60) - minutes) * 60) * 1000) / 1000.0
        return hours, minutes, seconds

    lat_str = '{}h {}m {}s'.format(*to_hms(cur_lat))
    if cur_lat < 0:
        lat_str = '-' + lat_str
    lng_str = '{}h {}m {}s'.format(*to_hms(cur_lng))
    if cur_lng < 0:
        lng_str = '-' + lng_str

    return lat_str, lng_str


def to_min_sec_str(sec):
    minutes = int(sec / 60.0)
    seconds = int((sec / 60.0 - minutes) * 60.0)
    mili = int((((sec / 60.0 - minutes) * 60.0) - seconds) * 10)

    return '{}m {}.{}s'.format(minutes, seconds, mili)


def norm_angle(ang):
    ang = 180.0 * ang / math.pi

    if ang > 180:
        ang -= 360 * math.ceil((ang - 180) / 360)

    if ang <= -180:
        ang -= 360 * math.floor((ang + 180) / 360)

    return math.pi * ang / 180.0


def time_to_periapsis(theta):
    e = vessel.orbit.eccentricity
    a = vessel.orbit.semi_major_axis
    rp = vessel.orbit.periapsis
    mu = vessel.orbit.body.gravitational_parameter

    print 'theta: ', theta

    if e == 1.0:
        D = math.tan(theta / 2)
        M = D + D * D * D / 3.0
        return math.sqrt(2.0 * rp * rp * rp / mu) * M

    if a > 0:
        cos_theta = math.cos(theta)
        cos_e = (e + cos_theta) / (1.0 + e * cos_theta)
        rad_e = math.acos(cos_e)
        M = rad_e - e * math.sin(rad_e)
        return math.sqrt(a * a * a / mu) * M

    if a < 0:
        cos_theta = math.cos(theta)
        cosh_f = (e + cos_theta) / (1.0 + e * cos_theta)
        rad_f = math.acosh(cosh_f)
        M = e * math.sinh(rad_f) - rad_f
        return math.sqrt(-a * a * a / mu) * M

    return 0


def radius_direction(theta):
    omega = vessel.orbit.argument_of_periapsis
    incl = vessel.orbit.inclination

    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)
    cos_omega = math.cos(omega)
    sin_omega = math.sin(omega)
    cos_incl = math.cos(incl)
    sin_incl = math.sin(incl)

    x = cos_omega * cos_theta - sin_omega * sin_theta
    y = cos_incl * (sin_omega * cos_theta + cos_omega * sin_theta)
    z = sin_incl * (sin_omega * cos_theta + cos_omega * sin_theta)
    mag = np.linalg.norm(np.array((x, y, z)))

    return (x, y, z), mag

#
# def calc_impact_time(vessel):
#     body_ref_frame = vessel.orbit.body.reference_frame
#     # do impact site calculations
#     impact_happening = True
#     impact_time = 0
#     # impactLongitude = 0
#     # impactLatitude = 0
#     impact_altitude = 0
#     e = vessel.orbit.eccentricity
#     # get current position direction vector
#     (current_pos_x, current_pos_y, current_pos_z), current_pos_mag = radius_direction(vessel.orbit.true_anomaly)
#     # calculate longitude in inertial reference frame from that
#     current_irf_long = math.atan2(current_pos_x, current_pos_y)
#
#     # experimentally determined; even for very flat trajectories, the errors go into the sub-millimeter area after 5 iterations or so
#     impact_iterations = 6
#
#     # do a few iterations of impact site calculations
#     for i in range(impact_iterations):
#         if vessel.orbit.periapsis_altitude >= impact_altitude:
#             # periapsis must be lower than impact alt
#             impact_happening = False
#
#         if vessel.orbit.eccentricity < 1 and vessel.orbit.apoapsis_altitude <= impact_altitude:
#             # apoapsis must be higher than impact alt
#             impact_happening = False
#
#         if vessel.orbit.eccentricity >= 1 and vessel.orbit.time_to_periapsis <= 0:
#             # if currently escaping, we still need to be before periapsis
#             impact_happening = False
#
#         if not impact_happening:
#             # this.impactTime = 0;
#             # this.impactLongitude = 0;
#             # this.impactLatitude = 0;
#             # this.impactAltitude = 0;
#             break
#
#         impact_theta = 0
#         if e > 0:
#             # in this step, we are using the calculated impact altitude of the last step, to refine the impact site position
#             cos_theta = (vessel.orbit.periapsis * (1 + e) / (vessel.orbit.body.equatorial_radius + impact_altitude) - 1) / e
#             if cos_theta < -1.0:
#                 cos_theta = -1.0
#             elif cos_theta > 1.0:
#                 cos_theta = 1.0
#             impact_theta = -math.acos(cos_theta)
#
#         # calculate time to impact
#         impact_time = vessel.orbit.time_to_periapsis - time_to_periapsis(impact_theta)
#
#         # calculate position vector of impact site
#         (impact_pos_x, impact_pos_y, impact_pos_z), impact_pos_mag = radius_direction(impact_theta)
#         # calculate longitude of impact site in inertial reference frame
#         impact_irf_long = math.atan2(impact_pos_x, impact_pos_y)
#         delta_irf_long = impact_irf_long - current_irf_long
#         # get body rotation until impact
#
#         body_rot = 2 * math.pi * impact_time / vessel.orbit.body.rotational_period
#         # body_rot = impact_time / vessel.orbit.body.rotational_period
#
#         print 'tme: ', impact_time, '|', vessel.orbit.body.rotational_period, '|', math.degrees(2 * math.pi * body_rot), '|', math.degrees(vessel.orbit.body.rotation_angle), '|', math.degrees(vessel.orbit.body.initial_rotation)
#
#         # print 'ROT: ', dir(body_ref_frame)
#
#         # # Here we calculate the sin( theta / 2) once for optimization
#         # factor = math.sin(body_rot / 2.0)
#         #
#         # # Calculate the x, y and z of the quaternion
#         # x = 0 * factor
#         # y = 1 * factor
#         # z = 0 * factor
#         #
#         # # Calcualte the w value by cos( theta / 2 )
#         # w = math.cos(body_rot / 2.0)
#
#         y_rotation = (
#             0,
#             math.sin(-body_rot),
#             0,
#             math.cos(-body_rot)
#         )
#
#         future_body_ref_frame = ReferenceFrame.create_relative(body_ref_frame, rotation=y_rotation)
#
#         # get current longitude in body coordinates
#         current_long = vessel.flight(future_body_ref_frame).longitude
#         # finally, calculate the impact longitude in body coordinates
#         impact_longitude = norm_angle(current_long - delta_irf_long - body_rot)
#         # calculate impact latitude from impact position
#         impact_latitude = math.asin(impact_pos_z / impact_pos_mag)
#
#         # future_impact_pos = vessel.orbit.body.surface_position(impact_latitude, impact_longitude, future_body_ref_frame)
#         # impact_pos = space_center.transform_position(future_impact_pos, future_body_ref_frame, body_ref_frame)
#         #
#         # print '----'
#         # print vessel.orbit.body.surface_position(impact_latitude, impact_longitude, future_body_ref_frame)
#         # print vessel.orbit.body.surface_position(impact_latitude, impact_longitude, body_ref_frame)
#         # print '****'
#         # print vessel.orbit.body.altitude_at_position(future_impact_pos, body_ref_frame)
#         # print vessel.orbit.body.altitude_at_position(impact_pos, body_ref_frame)
#
#         impact_altitude = vessel.orbit.body.altitude_at_position(impact_pos, future_body_ref_frame)
#
#         # impact_time = ut + vessel.orbit.time_to_periapsis - time_to_periapsis(impact_theta)
#         # vi = math.fabs(vessel.flight(body_ref_frame).vertical_speed)
#         # alt = vessel.flight(body_ref_frame).surface_altitude
#         # print 'grr: ', vi, '|', alt
#         # impact_time =get_seconds_to_impact(vi, alt)
#         # impact_pos = vessel.orbit.position_at(ut + impact_time, body_ref_frame)
#         # impact_altitude = vessel.orbit.body.altitude_at_position(impact_pos, body_ref_frame)
#         # cal_lat = vessel.orbit.body.latitude_at_position(impact_pos, ref_frame)
#         # cal_lng = vessel.orbit.body.longitude_at_position(impact_pos, ref_frame)
#
#         # print 'irf: ', current_irf_long, '|', impact_irf_long, 'x', current_long
#         # print impact_latitude, '--', impact_longitude
#         # print vessel.flight().latitude, '==', vessel.flight().longitude
#
#         print 'abc: ', impact_time, '|', impact_altitude, '=', current_long
#         # calculate the actual altitude of the impact site
#         # altitude for long/lat code stolen from some ISA MapSat forum post; who knows why this works, but it seems to.
#         # rad = QuaternionD.AngleAxis(this.impactLongitude, Vector3d.down) * QuaternionD.AngleAxis(this.impactLatitude, Vector3d.forward) * Vector3d.right;
#         # impact_altitude = FlightGlobals.ActiveVessel.mainBody.pqsController.GetSurfaceHeight(rad) - FlightGlobals.ActiveVessel.mainBody.pqsController.radius;
#
#         # if impact_altitude < 0 and FlightGlobals.ActiveVessel.mainBody.ocean:
#         #     impact_altitude = 0
#
#     return impact_time

    # // Set accessable properties.
    # if (this.impactHappening)
    # {
    #     ShowDetails = true;
    #     Time = this.impactTime;
    #     Longitude = this.impactLongitude;
    #     Latitude = this.impactLatitude;
    #     Altitude = this.impactAltitude;
    #     try
    #     {
    #         Biome = ScienceUtil.GetExperimentBiome(FlightGlobals.ActiveVessel.mainBody, this.impactLatitude, this.impactLongitude);
    #     }
    #     catch (Exception ex)
    #     {
    #         MyLogger.Log("GetExperimentBiome(" + FlightGlobals.ActiveVessel.mainBody.name + ", " + this.impactLatitude + ", " + this.impactLongitude + ") died");
    #         MyLogger.Exception(ex);
    #         Biome = "<failed>";
    #     }
    # }
    # else
    # {
    #     ShowDetails = false;
    # }


def get_current_stage(vessel):
    return max([part.stage for part in vessel.parts.all])


def get_avail_thrust(vessel):
    stage = get_current_stage(vessel)
    stage_parts = vessel.parts.in_stage(stage)
    return sum([part.engine.available_thrust for part in stage_parts if part.engine])


def get_avail_twr(vessel):
    stage = get_current_stage(vessel)
    stage_parts = vessel.parts.in_stage(stage)
    avail_thrust_for_stage = sum([part.engine.available_thrust for part in stage_parts if part.engine])
    return avail_thrust_for_stage / (vessel.mass * get_local_gravity(vessel))


def get_seconds_to_impact(vi, dst):
    m = vessel.mass
    r = vessel.orbit.radius
    g = active_body.surface_gravity

    # v = vi + a*t
    # d = ((v + vi)/2)*t
    # v**2 = vi**2 + 2*a*d
    # t = (-vi + math.sqrt(vi**2 - 2*g()*-dst)) / g()

    v = math.sqrt(vi**2 + 2*g*dst)

    # print ((v-vi) / g()), '==', dst / ((v+vi)/2.0), '==', (-vi + math.sqrt(vi**2 - 2*g()*-dst)) / g()
    return (v-vi) / g


def get_speed_after_time(vi, t):
    m = vessel.mass
    r = vessel.orbit.radius
    g = active_body.surface_gravity

    return vi + g * t


# def get_distance_after_time(vi, t):
#     vf = get_speed_after_time(vi, t)
#     return (vi + vf) / 2 * t


def get_time_for_distance(v, d):
    return d/v


def get_suicide_burn_time(vi, dist, use_local_g=False):
    r = vessel.orbit.radius
    mi = vessel.mass
    isp = vessel.specific_impulse
    if use_local_g:
        g = (G * (M+mi) / r**2)
    else:
        g = active_body.surface_gravity
    thrust = get_avail_thrust(vessel)

    bottom = (thrust/(g*isp))
    e_bit = (vi + math.sqrt(2*g*dist))/(g*isp)
    top = (mi - (mi/math.exp(e_bit)))
    return top / bottom




def get_suicide_burn_alt(vi, dist, use_local_g=False):
    r = vessel.orbit.radius
    mi = vessel.mass
    if use_local_g:
        g = (G * (M+mi) / r**2)
    else:
        g = active_body.surface_gravity

    thrust = get_avail_thrust(vessel) * 0.7
    # print 't:', thrust, 'm:', vessel.mass, 't/m:', thrust/vessel.mass, 'm/t:', vessel.mass/thrust

    accel = thrust/vessel.mass - g
    # t = get_suicide_burn_time(vi, dist, use_local_g=use_local_g)

    # return (vi + (math.sqrt(2 * g * dist) / 2)) * t
    return (vi ** 2.0) / (2.0 * accel)


def find_true_impact_time(vessel):

    active_body = vessel.orbit.body
    ref_frame = active_body.reference_frame
    flight = vessel.flight(ref_frame)

    base_time = get_seconds_to_impact(math.fabs(flight.speed) * 1.1, flight.surface_altitude)
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
        probe_pos = vessel.orbit.position_at(ut + probe_time, ref_frame)
        probe_pos_alt = active_body.altitude_at_position(probe_pos, ref_frame)

        body_rot = math.pi * probe_time / vessel.orbit.body.rotational_period
        futr_ref_frame = ReferenceFrame.create_relative(ref_frame, rotation=(0, math.sin(body_rot), 0, math.cos(body_rot)))

        probe_lat = active_body.latitude_at_position(probe_pos, futr_ref_frame)
        probe_lng = active_body.longitude_at_position(probe_pos, futr_ref_frame)

        surf_height = active_body.surface_height(probe_lat, probe_lng)

        # print probe_pos_alt, '|', surf_height, '|', resolution_param_index
        if probe_pos_alt - surf_height < height_resolution:
            probe_time = prev_probe_time
            resolution_param_index += 1

            if resolution_param_index >= len(resolution_params):
                break

            time_resolution, height_resolution = resolution_params[resolution_param_index]

        prev_probe_time = probe_time
        probe_time += time_resolution
        i += 1
    #
    # body_rot = math.pi * probe_time / vessel.orbit.body.rotational_period
    #
    # y_rotation = (
    #     0,
    #     math.sin(body_rot),
    #     0,
    #     math.cos(body_rot)
    # )
    #
    # neg_y_rotation = (
    #     0,
    #     math.sin(body_rot),
    #     0,
    #     -math.cos(body_rot)
    # )
    #
    # futr_ref_frame = ReferenceFrame.create_relative(ref_frame, rotation=y_rotation)
    # back_ref_frame = ReferenceFrame.create_relative(ref_frame, rotation=neg_y_rotation)

    # impact_pos = vessel.orbit.position_at(ut + probe_time, ref_frame)
    # impact_pos = space_center.transform_position(future_impact_pos, future_ref_frame, ref_frame)
    # print '-=-=-=-'
    # print math.degrees(body_rot)
    # print probe_time, to_min_sec_str(probe_time)
    #
    #
    #
    # fut_lat = active_body.latitude_at_position(probe_pos, futr_ref_frame)
    # fut_lng = active_body.longitude_at_position(probe_pos, futr_ref_frame)
    # print lat_lng_str(fut_lat, fut_lng)
    # print active_body.surface_height(fut_lat, fut_lng)
    # lat = active_body.latitude_at_position(probe_pos, ref_frame)
    # lng = active_body.longitude_at_position(probe_pos, ref_frame)
    # print lat_lng_str(lat, lng)
    # print active_body.surface_height(lat, lng)
    # bck_lat = active_body.latitude_at_position(probe_pos, back_ref_frame)
    # bck_lng = active_body.longitude_at_position(probe_pos, back_ref_frame)
    # print lat_lng_str(bck_lat, bck_lng)
    # print active_body.surface_height(bck_lat, bck_lng)
    #
    # print '&&&&', i
    # print to_min_sec_str(base_time), '|', to_min_sec_str(probe_time), '|', to_min_sec_str(probe_time - base_time)

    #
    # impact_lat =
    # impact_lng = vessel.flight(future_ref_frame).longitude
    #
    # future_impact_pos = vessel.orbit.body.surface_position(impact_latitude, impact_longitude, future_body_ref_frame)
    # impact_pos = space_center.transform_position(future_impact_pos, future_body_ref_frame, body_ref_frame)
    #
    # print '----'
    # print vessel.orbit.body.surface_position(impact_latitude, impact_longitude, future_body_ref_frame)
    # print vessel.orbit.body.surface_position(impact_latitude, impact_longitude, body_ref_frame)
    # print '****'
    # print vessel.orbit.body.altitude_at_position(future_impact_pos, body_ref_frame)
    # print vessel.orbit.body.altitude_at_position(impact_pos, body_ref_frame)

    # print lat_lng_str(probe_lat, probe_lng), i
    x1, y1, z1 = active_body.surface_position(probe_lat, probe_lng, ref_frame)
    x2, y2, z2 = vessel.position(ref_frame)

    # print speed, '@', surf_height

    return probe_time, math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2), probe_pos, futr_ref_frame

time_to_enter_mun_soi = None

print_twr = False
# warp = True
draw = True
ref_frame = vessel.orbital_reference_frame
ray_time_to_impact = 0
cal_time_to_impact = 0

while True:
    orb_flight = vessel.flight(active_body.non_rotating_reference_frame)
    surf_flight = vessel.flight(active_body.reference_frame)

    vi = math.fabs(orb_flight.speed)
    alt = orb_flight.surface_altitude

    if curr_state == COAST_TO_BURN:
        mun_pos = mun.position(kerbin.non_rotating_reference_frame)
        ves_pos = vessel.position(kerbin.non_rotating_reference_frame)
        phase_rads = angle_between(mun_pos, ves_pos)
        phase_angle = 360 - math.degrees(phase_rads)

        # ves_pos_5_behind = vessel.orbit.position_at(ut()-5, kerbin.reference_frame)
        # phase_angle_5_behind = 360 - math.degrees(angle_between(mun_pos, ves_pos_5_behind))
        # ves_pos_5_ahead = vessel.orbit.position_at(ut()+5, kerbin.reference_frame)
        # phase_angle_5_ahead = 360 - math.degrees(angle_between(mun_pos, ves_pos_5_ahead))

        relative_inclination = math.degrees(mun.orbit.relative_inclination(vessel))
        r1 = vessel.orbit.radius
        r2 = mun.orbit.radius

        phase_rad_needed = math.pi * (1 - math.sqrt((1/8.0)*(1.0+(r1/r2))**3))
        mu = vessel.orbit.body.gravitational_parameter

        dv_burn = math.sqrt(mu/r1) * (math.sqrt(2*r2/(r1 + r2)) - 1)
        burn_time = get_burn_time_for_dv(dv_burn, vessel)

        speed_r1 = math.sqrt(mu/(r1**3))
        speed_r2 = math.sqrt(mu/(r2**3))
        speed_diff = speed_r2 - speed_r1

        if phase_rad_needed < phase_rads:
            rad_to_burn = phase_rads - phase_rad_needed
        else:
            rad_to_burn = 2 * math.pi + phase_rads - phase_rad_needed

        seconds_until_burn = math.fabs(rad_to_burn / speed_diff)

        print 'bt: ', burn_time
        print 'ub: ', seconds_until_burn

        if seconds_until_burn > 25:
            space_center.rails_warp_factor = 3
        elif seconds_until_burn > 10:
            space_center.rails_warp_factor = 0
        else:

            burn_until = ut + burn_time
            vessel.control.throttle = 1.0
            set_state(MUN_BURN)
    elif curr_state == MUN_BURN:
        if ut > burn_until:
            vessel.control.throttle = 0.0
            time_to_enter_mun_soi = ut + vessel.orbit.time_to_soi_change
            set_state(ENROUTE_TO_MUN)
    elif curr_state == ENROUTE_TO_MUN:
        if ut >= time_to_enter_mun_soi:
            set_state(MUN_DESCENT)
    elif curr_state == MUN_DESCENT:
        curr_follow = SURFACE_RETROGRADE
        cal_time_to_impact, cal_dist, cal_impact_pos, cal_impact_ref_frame = find_true_impact_time(vessel)

        # we need to get the m/s at the time of the start of the suicide burn
        vf = get_speed_after_time(surf_flight.speed, cal_time_to_impact)
        # impact_plus_1km = get_time_for_distance(vf, 1000)
        # use that speed to make the suicide burn happen 100m higher

        thrust = get_avail_thrust(vessel)
        a = thrust - active_body.surface_gravity

        print 'spd: ', surf_flight.speed
        print 'vf:  ', vf
        print 'imp: ', cal_time_to_impact
        # print '1km: ', impact_plus_1km
        print 'dst: ', cal_dist
        print 'sbt: ', get_suicide_burn_time(surf_flight.speed, cal_dist)
        print 'alt: ', get_suicide_burn_alt(surf_flight.speed, cal_dist)

        # suicide_burn_time = get_suicide_burn_time(vf, cal_dist)
        # suicide_burn_start_time = ut + cal_time_to_impact - suicide_burn_time - impact_plus_1km
        # suicide_burn_start_alt = get_suicide_burn_alt(vf, cal_dist)

        set_state(SUICIDE_BURN)

    elif curr_state == SUICIDE_BURN:
        # curr_follow = SURFACE_RETROGRADE

        if ray_time_to_impact > 0:
            time_to_impact = ray_time_to_impact
        else:
            time_to_impact = cal_time_to_impact

        if ray_dist != float('inf') and ray_dist > 0:
            dist = ray_dist
        else:
            dist = cal_dist

        # vf = get_speed_after_time(surf_flight.speed, time_to_impact)
        # impact_plus_1km = get_time_for_distance(surf_flight.speed, 1000)
        # use that speed to make the suicide burn happen 100m higher

        thrust = get_avail_thrust(vessel)
        a = thrust - active_body.surface_gravity

        # suicide_burn_time = get_suicide_burn_time(surf_flight.speed, dist, use_local_g=True)
        # suicide_burn_start_time = ut + time_to_impact - suicide_burn_time #- impact_plus_1km
        suicide_burn_start_alt = get_suicide_burn_alt(surf_flight.speed, dist, use_local_g=True)

        print ''
        print 'sp:', surf_flight.speed
        # print 'bt:', suicide_burn_time
        # print 'ut:', ut, 'bs:', suicide_burn_start_time
        print 'ba:', suicide_burn_start_alt, 'dist:', dist

        if dist < suicide_burn_start_alt + 150:
            if dist < suicide_burn_start_alt:
                if surf_flight.vertical_speed < -5.0:
                    if suicide_burn_start_alt - dist > 0.5:
                        vessel.control.throttle = 1.0
                    else:
                        vessel.control.throttle = 0.705
                else:
                    avail_twr = get_avail_twr(vessel)
                    vessel.control.throttle = 0.85 / avail_twr
                    curr_follow = FIXED_UP
                    set_state(LAND)
        else:
            vessel.control.throttle = 0
    elif curr_state == LAND:
        if surf_flight.vertical_speed < -1.0:
            avail_twr = get_avail_twr(vessel)
            vessel.control.throttle = 0.9 / avail_twr
        else:
            vessel.control.throttle = 0
        # vessel.auto_pilot.disengage()



    # sqrt ( 2 * height / 9.8 )
    # vi = math.sqrt(2*g*d)
    # vi**2 = 2*g*d
    # vi**2 = (2*d)*(G*(M+m)/r**2)
    # d = (vi**2 * r**2) / (2*G*(M+m))
    # g in the above equations may be replaced by G(M+m)/r2 where G is the gravitational constant,
    # M is the mass of the astronomical body,
    # m is the mass of the falling body,
    # and r is the radius from the falling object to the center of the body.



    # t = math.sqrt((2*d)/g)
    # t = math.sqrt((2*d)/(G*(M+m)/r**2))
    # td = math.sqrt((2*d)/(G*(M+m)/r**2))
    # t1 = math.sqrt((2*alt)/(G*(M+m)/r**2))
    # dis = d + alt
    # t2 = math.sqrt((2*dis)/(G*(M+m)/r**2))
    # vessel.surface_altitude
    # pos_to_test = 'a'
    # ref_frame = vessel.reference_frame
    # ref_frame = sun.reference_frame

    m = vessel.mass
    r = vessel.orbit.radius


    # g = mun.surface_gravity
    # vi = d/t - g*t * 1/2
    # d/t = vi + (g*t * 1/2)
    # d = t * (vi + g*t * 1/2)
    # d = vi*t + g*t**2 * 1/2
    # 0 = vi*t + g*t**2 * 1/2 - d
    # 0 = 1/2*g*x^2 + vi*x - d
    # a = 1/2*g | b = vi | c = -d
    # t = (-vi +- math.sqrt(vi**2 - 4*1/2*g*-d)) / (2*1/2*g)
    # tp = (-vi + math.sqrt(vi**2 - 2*g*-d)) / g
    # tn = (-vi - math.sqrt(vi**2 - 2*g*-d)) / g

    # time_to_impact = None
    # # tn = None
    # g = lambda: (G*(M+m)/r**2)
    # if (vi**2 - 4*1/2*g()*-alt) >= 0:
    #     time_to_impact = (-vi + math.sqrt(vi**2 - 2*g()*-alt)) / g()
    #     # tn = (-vi - math.sqrt(vi**2 - 2*g()*-alt)) / g()

    # time_to_impact = get_seconds_to_impact(vi, alt)
    # impact_time = ut + time_to_impact
    # impact_pos = vessel.orbit.position_at(impact_time, ref_frame)

    # alt_lat = mun.latitude_at_position(impact_pos, ref_frame)
    # alt_lng = mun.longitude_at_position(impact_pos, ref_frame)
    # ax**2 + bx + c
    # -b +- math.sqrt(b**2 - 4ac) / 2a

    # vi/t = d/t**2 - g/2
    # g/2 = d/t**2 - vi/t
    # print vi, '|', math.sqrt(2*g*d), '|', (d/t)
    # print (-2*vi / g)



    ray_dist = 0
    ray_time_to_impact = 0
    ray_impact_time = 0
    ray_impact_pos = None
    ray_aai = None
    ray_lat = None
    ray_lng = None

    if lines:
        for line in lines:
            line.remove()
        lines = []

    if alt < 5000:
        ray_dist = space_center.raycast_distance((0, 5, 0), (0, 1, 0), vessel.surface_velocity_reference_frame)
        ray_time_to_impact = get_seconds_to_impact(surf_flight.speed, ray_dist)
        if ray_time_to_impact != float('inf'):
            ray_impact_time = ut + ray_time_to_impact

            body_rot = math.pi * ray_impact_time / vessel.orbit.body.rotational_period
            ray_ref_frame = ReferenceFrame.create_relative(
                active_body.reference_frame, rotation=(0, math.sin(body_rot), 0, math.cos(body_rot)))

            ray_impact_pos = vessel.orbit.position_at(ray_impact_time, ray_ref_frame)
            ray_aai = mun.altitude_at_position(ray_impact_pos, ray_ref_frame)
            ray_lat = mun.latitude_at_position(ray_impact_pos, ray_ref_frame)
            ray_lng = mun.longitude_at_position(ray_impact_pos, ray_ref_frame)

            # Shows which direction our ray cast is looking
            lines.append(conn.drawing.add_line((0, 5, 0), (0, 1, 0), vessel.surface_velocity_reference_frame))
            lines[-1].color = (0, 1.0, 0)

            rel_ray_impact_pos = space_center.transform_position(ray_impact_pos, ray_ref_frame, active_body.reference_frame)
            rel_ray_impact_ref_frame = ReferenceFrame.create_relative(active_body.reference_frame, position=rel_ray_impact_pos)
            rel_ray_vessel_pos = vessel.position(rel_ray_impact_ref_frame)

            lines.append(conn.drawing.add_line((0, 0, 0), rel_ray_vessel_pos, rel_ray_impact_ref_frame))
            lines[-1].color = (0, 0, 1)


    # # cal_dist = find_true_impact_time(vessel)
    # # cal_time_to_impact = get_seconds_to_impact(vi, cal_dist)
    #
    # # cal_impact_time = ut + cal_time_to_impact
    # # cal_impact_pos = vessel.orbit.position_at(cal_impact_time, ref_frame)
    # cal_aai = mun.altitude_at_position(cal_impact_pos, cal_impact_ref_frame)
    # cal_lat = mun.latitude_at_position(cal_impact_pos, cal_impact_ref_frame)
    # cal_lng = mun.longitude_at_position(cal_impact_pos, cal_impact_ref_frame)
    #
    # if lines:
    #     for line in lines:
    #         line.remove()
    #     lines = []
    #
    # vessel_pos = vessel.position(cal_impact_ref_frame)
    # # lines.append(conn.drawing.add_line(vessel_pos, impact_pos, ref_frame))
    # # lines[-1].color = (1.0, 0, 0)
    #
    # # rel_impact_pos = space_center.transform_position(impact_pos, ref_frame, mun.reference_frame)
    # # rel_impact_ref_frame = ReferenceFrame.create_relative(mun.reference_frame, position=rel_impact_pos)
    # # rel_vessel_pos = vessel.position(rel_impact_ref_frame)
    #
    # # lines.append(conn.drawing.add_line((0, 0, 0), rel_vessel_pos, rel_impact_ref_frame))
    # # lines[-1].color = (1, 0, 1)
    #
    # # cal_ray_impact_pos = space_center.transform_position(ray_impact_pos, ref_frame, mun.reference_frame)
    rel_cal_impact_pos = space_center.transform_position(cal_impact_pos, cal_impact_ref_frame, mun.reference_frame)
    rel_cal_impact_ref_frame = ReferenceFrame.create_relative(mun.reference_frame, position=rel_cal_impact_pos)
    rel_cal_vessel_pos = vessel.position(rel_cal_impact_ref_frame)
    #
    # Impact spot, to ship
    lines.append(conn.drawing.add_line((0, 0, 0), rel_cal_vessel_pos, rel_cal_impact_ref_frame))
    lines[-1].color = (0, 1, 1)
    #
    # Ship to impact spot
    # vessel_impact_pos = space_center.transform_position(cal_impact_pos, cal_impact_ref_frame, vessel.surface_velocity_reference_frame)
    # lines.append(conn.drawing.add_line((0, 0, 0), vessel_impact_pos, vessel.surface_velocity_reference_frame))
    # lines[-1].color = (1, 1, 0)

    # ves_imp_time = calc_impact_time(vessel)
    #
    # to_print = [
    #     # # '',
    #     # # 'vi: {} M: {} m: {} r: {} G: {}'.format(vi, M, m, r, G),
    #     # # 'ut: {}'.format(ut),
    #     # # 'vvl: {}'.format(vi),
    #     # 'spd: {}'.format(surf_flight.speed),
    #     # 'srf: {}'.format(surf_flight.surface_altitude),
    #     # # 'alt: {}'.format(alt),
    #     # 'cal: {}'.format(cal_dist),
    #     # 'ray: {}'.format(ray_dist),
    #     # # 'tst_tti: {} | {}'.format(ves_imp_time, to_min_sec_str(ves_imp_time)),
    #     # # 'alt_tti: {} | {}'.format(time_to_impact, to_min_sec_str(time_to_impact)),
    #     # 'cal_tti: {} | {}'.format(cal_time_to_impact, to_min_sec_str(cal_time_to_impact)),
    #     # 'ray_tti: {} | {}'.format(ray_time_to_impact, to_min_sec_str(ray_time_to_impact if ray_time_to_impact != float('inf') else 0)),
    #     # # 'alt_aai: {}'.format(mun.altitude_at_position(impact_pos, ref_frame)),
    #     # 'cal_aai: {}'.format(cal_aai),
    #     # 'ray_aai: {}'.format(ray_aai),
    #     # # 'alt_sfc: {}'.format(mun.surface_height(alt_lat, alt_lng)),
    #     # 'cal_sfc: {}'.format(mun.surface_height(cal_lat, cal_lng)),
    #     # 'ray_sfc: {}'.format(mun.surface_height(ray_lat, ray_lng) if ray_lat else None),
    #     # # 'alt_ll: {} | {}'.format(alt_lat, alt_lng),
    #     # # 'ray_ll: {} | {}'.format(ray_lat, ray_lng),
    #     #
    #     # # str(speed_r1) + ' | ' + str(speed_r2) + ' | ' + str(speed_diff),
    #     # # str(math.degrees(phase_rads)) + ' | ' + str(math.degrees(phase_rad_needed)) + ' | ' + str(dv_burn),
    #     # # str(math.degrees(rad_to_burn)) + ' | ' + str(seconds_until_burn) + ' | ' + str(seconds_until_burn / 60.0),
    #     # # str(burn_time) + ' | ' + str(ut()) + ' | ' + str(burn_until),
    #     # # str(space_center.raycast_distance((0.0, -5.0, 0.0), (0, -1, 0), vessel.reference_frame))
    #     '',
    # ]
    # print '\n'.join(to_print)

    # sleep(1)

