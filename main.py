import time
import krpc
from datetime import datetime
from functools import partial
import math
from time import sleep
from itertools import chain
from multiprocessing import Lock

# STARTING OPTIMIZATION:
# DV: 504 Aopap: 177406.1 Periap: 159465.1
# First Opt:
# DV: 826 Aopap: 175784.8 Periap: 157777.2
# More opt:
# DV: 765 Periap: 162114.8
# DV: 798 Periap: 163020 -- start_ptc = 78 / max bucket throttle = 0.5
# DV: 688 Periap: 163020 -- start_ptc = 80 / max bucket throttle = 0.5
# DV: 853ish Periap: 163020 -- start_ptc = 76.5 / max bucket throttle = 0.5

# turn_start_altitude = 250
# DV: 827 adjusted mach parameters -- start_ptc = 78 / max bucket throttle = 0.4
# DV: 714 start_ptc = 80 / max bucket throttle = 0.45
# DV: 780 start_ptc = 78 / max bucket throttle = 0.5
# DV: 836 start_ptc = 76.5 / max bucket throttle = 0.5
# DV: 873 start_ptc = 75.5 / max bucket throttle = 0.5

# turn_start_altitude = 1000
# DV: 639 start_ptc = 72.5 / max bucket throttle = 0.5

# turn_start_altitude = 750
# DV: 699 start_ptc = 72.5 / max bucket throttle = 0.5

# turn_start_altitude = 500
# DV: 789 start_ptc = 72.5 / max bucket throttle = 0.5


def get_current_stage(vessel):
    return max([part.stage for part in vessel.parts.all])


def get_local_gravity(vessel):
    body = vessel.orbit.body
    r = body.equatorial_radius
    h = vessel.flight().mean_altitude
    g = body.surface_gravity
    return g * (r / (r + h))**2


def get_avail_twr(vessel):
    stage = get_current_stage(vessel)
    stage_parts = vessel.parts.in_stage(stage)
    avail_thrust_for_stage = sum([part.engine.available_thrust for part in stage_parts if part.engine])
    return avail_thrust_for_stage / (vessel.mass * get_local_gravity(vessel))


def get_twr(vessel):
    stage = get_current_stage(vessel)
    stage_parts = vessel.parts.in_stage(stage)
    thrust_for_stage = sum([part.engine.thrust for part in stage_parts if part.engine])
    return thrust_for_stage / (vessel.mass * get_local_gravity(vessel))


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
ASCENT = 1
COAST_TO_APOAPSIS = 2
CIRCULARIZE = 3
CIRCULARIZE_BURN = 4
IN_ORBIT = 5
curr_state = ASCENT

FIXED_UP = 1
FIXED_POINT = 2
SURFACE_PROGRADE = 3
ORBIT_PROGRADE = 4
curr_follow = FIXED_UP

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

turn_start_altitude = 250
turn_end_altitude = 45000
target_altitude = 175000

# Set up streams for telemetry
ut = conn.add_stream(getattr, conn.space_center, 'ut')
flight = vessel.flight()
first_stage = get_current_stage(vessel)
altitude = 0
turn_angle = 0
time_to_apoapsis = 999999
burn_until = None

def flight_callback(x):
    global flight
    flight = x

flight_stream = conn.add_stream(vessel.flight)
flight_stream.add_callback(flight_callback)
flight_stream.start()

decouple_stage_activate_next = {}


fuel_lock = Lock()
def has_fuel_callback(has_fuel):
    global burn_until
    print has_fuel

    if not has_fuel:
        max_decouple_stage = max([part.decouple_stage for part in vessel.parts.all])
        engines_to_decouple = [part for part in vessel.parts.in_decouple_stage(max_decouple_stage) if part.engine]

        with fuel_lock:
            # stage = get_current_stage(vessel)
            # stage_parts = vessel.parts.in_stage(stage)
            # all_engines_flameout = not any([part.engine.has_fuel for part in engines_to_decouple])
            all_engines_flameout = not any([part.engine.thrust for part in engines_to_decouple])

            print 'mds: ', max_decouple_stage
            print 'rsc: ', [part.engine.thrust for part in engines_to_decouple]
            print 'exh: ', all_engines_flameout
            print decouple_stage_activate_next

            if engines_to_decouple and all_engines_flameout:
                already_activated = decouple_stage_activate_next.get(max_decouple_stage, False)
                if not already_activated:
                    decouple_stage_activate_next[max_decouple_stage] = True
                    vessel.control.activate_next_stage()

                    if curr_state == CIRCULARIZE_BURN:
                        dv_circ_burn = get_dv_needed_for_circularization(vessel)
                        burn_until = get_burn_time_for_dv(dv_circ_burn, vessel) + ut()

# srb_lock = Lock()
# def srb_fuel_callback(fuel):
#     global burn_until
#     print fuel
#
#     if fuel < 0.1:
#         with srb_lock:
#             # stage = get_current_stage(vessel)
#             # stage_parts = vessel.parts.in_stage(stage)
#             max_decouple_stage = max([part.decouple_stage for part in vessel.parts.all])
#             solid_fuel_resources = list(chain(*[
#                 part.resources.with_resource('SolidFuel') for part in vessel.parts.in_decouple_stage(max_decouple_stage)
#                 if part.resources.has_resource('SolidFuel')]))
#
#             all_solid_fuel_exhasted = not any([resource.amount > 0.1 for resource in solid_fuel_resources])
#             print 'mds: ', max_decouple_stage
#             print 'rsc: ', [resource.amount > 0.1 for resource in solid_fuel_resources]
#             print 'exh: ', all_solid_fuel_exhasted
#             print decouple_stage_activate_next
#             # all_engines_flameout = not any([part.engine.has_fuel for part in engines_to_decouple])
#
#             if all_solid_fuel_exhasted:
#                 already_activated = decouple_stage_activate_next.get(max_decouple_stage, False)
#                 if not already_activated:
#                     decouple_stage_activate_next[max_decouple_stage] = True
#                     vessel.control.activate_next_stage()
#
#                     # if curr_state == CIRCULARIZE_BURN:
#                     #     dv_circ_burn = get_dv_needed_for_circularization(vessel)
#                     #     burn_until = get_burn_time_for_dv(dv_circ_burn, vessel) + ut()

for part in vessel.parts.all:
    if part.engine:
        e_stream = conn.add_stream(getattr, part.engine, 'has_fuel')
        e_stream.add_callback(has_fuel_callback)
        e_stream.start()


# for part in vessel.parts.all:
#     # srb_fuel = conn.add_stream(stage_2_resources.amount, 'SolidFuel')
#     for resource in part.resources.with_resource('SolidFuel'):
#         srb_stream = conn.add_stream(getattr, resource, 'amount')
#         srb_stream.add_callback(has_fuel_callback)
#         srb_stream.start()


out_of_bucket = False
print_twr = True
burn_needed_for_circle = None


def altitude_callback(x):
    global altitude
    altitude = x

    if curr_follow == FIXED_UP:
        vessel.auto_pilot.reference_frame = vessel.surface_reference_frame
        vessel.auto_pilot.target_pitch_and_heading(90, 90)
    elif curr_follow == FIXED_POINT:
        vessel.auto_pilot.reference_frame = vessel.surface_reference_frame
        vessel.auto_pilot.target_pitch_and_heading(80, 90)
    elif curr_follow == SURFACE_PROGRADE:
        vessel.auto_pilot.reference_frame = vessel.surface_velocity_reference_frame
        vessel.auto_pilot.target_direction = (0, 1, 0)
        # if vessel.auto_pilot.target_roll == 180:
        #     vessel.auto_pilot.target_pitch_and_heading(flight.pitch + flight.angle_of_attack, 90)
        # else:
        #     vessel.auto_pilot.target_pitch_and_heading(flight.pitch - flight.angle_of_attack, 90)
    elif curr_follow == ORBIT_PROGRADE:
        # orb_prograde, _ = get_pitch_heading(flight.prograde)
        # vessel.auto_pilot.target_pitch_and_heading(orb_prograde, 90)
        vessel.auto_pilot.reference_frame = vessel.surface_reference_frame
        vessel.auto_pilot.target_direction = flight.prograde

altitude_stream = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
altitude_stream.add_callback(altitude_callback)
altitude_stream.start()


def time_to_apoapsis_callback(x):
    global time_to_apoapsis
    time_to_apoapsis = x

time_to_apoapsis_stream = conn.add_stream(getattr, vessel.orbit, 'time_to_apoapsis')
time_to_apoapsis_stream.add_callback(time_to_apoapsis_callback)
time_to_apoapsis_stream.start()

# Pre-launch setup
vessel.control.sas = True
vessel.control.rcs = False
vessel.control.throttle = 1.0

# Activate the first stage
vessel.control.activate_next_stage()
vessel.auto_pilot.engage()
vessel.auto_pilot.target_pitch_and_heading(90, 90)
vessel.auto_pilot.target_roll = 0

# Keep the program running...
while True:
    if datetime.now().second % 3 == 0:
        if print_twr:
            dv_circ_burn = get_dv_needed_for_circularization(vessel)

            rows_to_write = [
                '',
                'dv_circ_burn: ' + str(get_dv_needed_for_circularization(vessel)),
                'burn_time: ' + str(get_burn_time_for_dv(dv_circ_burn, vessel)),
                'burn_until: ' + str(burn_until),
                'burn_needed_for_circle: ' + str(burn_needed_for_circle),
                'time_to_apoapsis: ' + str(time_to_apoapsis),
                'curr_follow: ' + str(curr_follow),
            ]
            print '\n'.join(rows_to_write)

            print_twr = False
    else:
        print_twr = True

    if curr_state == ASCENT:
        if turn_end_altitude > altitude > turn_start_altitude:
            if not out_of_bucket or altitude < 30000:
                if flight.mach < 0.4:
                    curr_follow = FIXED_POINT
                elif flight.mach > 0.65:
                    curr_follow = SURFACE_PROGRADE
                elif curr_follow != SURFACE_PROGRADE and math.fabs(flight.angle_of_attack) > 1.0:
                    curr_follow = FIXED_POINT
                else:
                    curr_follow = SURFACE_PROGRADE

            else:
                curr_follow = ORBIT_PROGRADE

        if 1.85 > flight.mach > 0.85:
            diff = math.fabs(1.35 - flight.mach)  # gives num between 0 and 0.50
            mach_diff_pct = (0.50 - diff) / 0.50
            throttle_pct = 1.0 - (math.sin(math.radians(mach_diff_pct * 90)) * 0.50)
            avail_twr = get_avail_twr(vessel)
            min_twr = 1.55
            if throttle_pct * avail_twr > min_twr:
                vessel.control.throttle = throttle_pct
            else:
                if not avail_twr:
                    vessel.control.throttle = 1.0
                else:
                    throttle_pct = min_twr / avail_twr
                    vessel.control.throttle = throttle_pct

            if flight.mach > 1.5:
                out_of_bucket = True

        if vessel.orbit.apoapsis_altitude < target_altitude:
            if vessel.orbit.apoapsis_altitude >= target_altitude * 0.975:
                vessel.control.throttle = 0.25
        else:
            vessel.control.throttle = 0
            dv_circ_burn = get_dv_needed_for_circularization(vessel)
            burn_needed_for_circle = get_burn_time_for_dv(dv_circ_burn, vessel)
            set_state(COAST_TO_APOAPSIS)

    elif curr_state == COAST_TO_APOAPSIS:
        curr_follow = ORBIT_PROGRADE
        if time_to_apoapsis > (90 + burn_needed_for_circle / 2.0 - burn_needed_for_circle * 0.1):
            space_center.rails_warp_factor = 3
        elif time_to_apoapsis > (15 + burn_needed_for_circle/2.0 - burn_needed_for_circle*0.1):
            space_center.physics_warp_factor = 4
        elif time_to_apoapsis > (burn_needed_for_circle/2.0 - burn_needed_for_circle*0.1):
            vessel.control.rcs = True
            space_center.physics_warp_factor = 0
        else:
            vessel.control.rcs = False
            set_state(CIRCULARIZE)

    elif curr_state == CIRCULARIZE:
        curr_follow = ORBIT_PROGRADE

        dv_circ_burn = get_dv_needed_for_circularization(vessel)
        burn_until = get_burn_time_for_dv(dv_circ_burn, vessel) + ut()

        vessel.control.throttle = 1.0
        set_state(CIRCULARIZE_BURN)

    elif curr_state == CIRCULARIZE_BURN:
        if ut() > burn_until:
            vessel.control.throttle = 0
            set_state(IN_ORBIT)

    elif curr_state == IN_ORBIT:
        vessel.auto_pilot.disengage()

