from time import sleep
from threading import Thread
import math
from datetime import datetime

from util import get_avail_twr, get_burn_time_for_dv, get_vessel_pitch_heading, angle_between
from manager import PitchManager


class State(object):
    _thread = None

    def __init__(self):
        from controller import Controller
        self.ctrl = Controller.get()
        self.init_state()
        self._thread = Thread(target=self._run, args=())
        self._thread.daemon = True
        self._thread.start()

    def init_state(self):
        pass

    def _run(self):
        self.ctrl.state_condition.acquire()
        self.run()
        self.ctrl.state_condition.notify_all()
        self.ctrl.state_condition.release()

    def run(self):
        pass

    def join(self):
        return self._thread.join()


class PreLaunch(State):
    TARGET_ROLL = 0

    def run(self):
        self.ctrl.vessel.control.input_mode = self.ctrl.space_center.ControlInputMode.override

        # Do warning checks here
        # t_minus = 3
        # print 'Launch in:'
        # while t_minus > 0:
        #     print '{}'.format(t_minus)
        #     t_minus -= 1
        #     # sleep(1)
        #
        # print 'Lift off!'
        # print 'setting: {}'.format(AscentState)

        self.ctrl.vessel.control.throttle = 1.0
        self.ctrl.vessel.auto_pilot.engage()
        self.ctrl.vessel.auto_pilot.target_pitch_and_heading(90, 90)
        self.ctrl.vessel.auto_pilot.target_roll = self.TARGET_ROLL

        self.ctrl.activate_next_stage()

        self.ctrl.set_NextStateCls(AscentState)


class AscentState(State):
    ALTITUDE_TURN_START = 100
    ALTITUDE_TARGET = 175000
    # ALTITUDE_TARGET = 2868740  # geostationary
    # ALTITUDE_TARGET = 11400000  # Mun
    # ALTITUDE_TARGET = 46400000  # Minmus

    def run(self):
        self.ctrl.vessel.auto_pilot.engage()
        self.ctrl.vessel.auto_pilot.target_roll = PreLaunch.TARGET_ROLL

        twr = self.ctrl.get_avail_twr()
        if twr < 1.0:
            print 'ABORT! TWR TOO LOW! {}'.format(twr)

        low_twr_pitch = 85
        high_twr_pitch = 72.5

        twr_low_bound = 1.3
        twr_high_bound = 2.25

        if twr <= twr_low_bound:
            self.ctrl.pitch_manager.fixed_point_pitch = low_twr_pitch
        elif twr_low_bound < twr < twr_high_bound:
            pct = (twr - twr_low_bound) / (twr_low_bound - twr_high_bound)
            self.ctrl.pitch_manager.fixed_point_pitch = low_twr_pitch + pct * (low_twr_pitch - high_twr_pitch)
        else:
            self.ctrl.pitch_manager.fixed_point_pitch = high_twr_pitch

        print 'fixed pitch: {}'.format(self.ctrl.pitch_manager.fixed_point_pitch)

        last_second = datetime.now().second

        i = 0
        while True:
            i += 1
            curr_second = datetime.now().second
            if curr_second != last_second:
                print
                print 'tks:', i
                print 'stms:', self.ctrl._streams_open
                print 'alt:', self.ctrl.altitude
                print 'mach:', self.ctrl.mach
                print 'follow:', self.ctrl.pitch_follow
                print 'apoapsis:', self.ctrl.apoapsis_altitude
                print 'periapsis:', self.ctrl.periapsis_altitude

                i = 0
                last_second = curr_second
            if self.ctrl.altitude < self.ALTITUDE_TURN_START:
                self.ctrl.pitch_follow = PitchManager.FIXED_UP

            elif self.ctrl.altitude >= self.ALTITUDE_TURN_START:
                if self.ctrl.altitude < 30000:
                    if self.ctrl.mach < 0.4:
                        self.ctrl.pitch_follow = PitchManager.FIXED_POINT
                    elif self.ctrl.mach > 0.65:
                        self.ctrl.pitch_follow = PitchManager.SURFACE_PROGRADE
                    elif self.ctrl.pitch_follow != PitchManager.SURFACE_PROGRADE and math.fabs(self.ctrl.angle_of_attack) > 1.0:
                        self.ctrl.pitch_follow = PitchManager.FIXED_POINT
                    elif self.ctrl.vessel.flight().pitch < 30:
                        self.ctrl.pitch_manager.fixed_point_pitch = 30
                        self.ctrl.pitch_follow = PitchManager.FIXED_POINT
                    else:
                        self.ctrl.pitch_follow = PitchManager.SURFACE_PROGRADE
                elif self.ctrl.vessel.orbit.time_to_apoapsis < 30.0:
                    self.ctrl.pitch_manager.fixed_point_pitch = 10
                    self.ctrl.pitch_follow = PitchManager.FIXED_POINT
                else:
                    self.ctrl.pitch_follow = PitchManager.ORBIT_PROGRADE

            if 1.85 > self.ctrl.mach > 0.85:
                # print
                # print '-----'
                # print datetime.now().isoformat()
                diff = math.fabs(1.35 - self.ctrl.mach)  # gives num between 0 and 0.50
                mach_diff_pct = (0.50 - diff) / 0.50
                # print datetime.now().isoformat()
                throttle_pct = 1.0 - (math.sin(math.radians(mach_diff_pct * 90)) * 0.50)
                # print datetime.now().isoformat()
                avail_twr = get_avail_twr(self.ctrl)
                # print datetime.now().isoformat()
                min_twr = 1.75
                if throttle_pct * avail_twr > min_twr:
                    self.ctrl.set_throttle(throttle_pct)
                else:
                    if not avail_twr:
                        self.ctrl.set_throttle(1.0)
                    else:
                        throttle_pct = min_twr / avail_twr
                        self.ctrl.set_throttle(throttle_pct)
                # print datetime.now().isoformat()
                # print

                # if self.ctrl.mach > 1.5:
                #     out_of_bucket = True

            if self.ctrl.apoapsis_altitude < self.ALTITUDE_TARGET:
                if self.ctrl.apoapsis_altitude >= self.ALTITUDE_TARGET * 0.985:
                    self.ctrl.set_throttle(0.05)
            else:
                self.ctrl.set_throttle(0)

                if self.ctrl.altitude > 70000:
                    self.ctrl.pitch_follow = PitchManager.ORBIT_PROGRADE
                    # dv_circ_burn = self.get_dv_needed_for_circularization()
                    # burn_time_for_circle = get_burn_time_for_dv(dv_circ_burn, self.ctrl)
                    # burn_start_time = self.ctrl.ut + self.ctrl.time_to_apoapsis - (burn_time_for_circle / 2.0)

                    # self.ctrl.set_burn_dv(dv_circ_burn)
                    self.ctrl.set_burn_dv_func(self.get_dv_needed_for_circularization)
                    self.ctrl.set_burn_point_func(self.get_circularization_burn_point)
                    # self.ctrl.set_burn_point(self.get_circularization_burn_point())

                    # print 'dv needed: {}'.format(self.ctrl.burn_dv)
                    # print 'burn point: {}'.format(self.ctrl.burn_point)
                    # print 'ut:         {}'.format(self.ctrl.ut)
                    # print 'burn start: {}'.format(self.ctrl.burn_start)
                    # print 'burn time: {}'.format(self.ctrl.burn_time)
                    self.ctrl.vessel.control.rcs = True
                    self.ctrl.vessel.auto_pilot.stopping_time = (4.0, 4.0, 4.0)
                    self.ctrl.space_center.warp_to(self.ctrl.get_burn_start() - 20)

                    self.ctrl.set_NextStateCls(CoastToApoapsis)
                    break

    def get_dv_needed_for_circularization(self):
        # r_ap = current apoapsis radius in meters (and the altitude you will circularize at)
        r_ap = self.ctrl.apoapsis
        # r_pe = current periapsis radius in meters
        r_pe = self.ctrl.periapsis
        # mu = gravitational parameter for the body you are orbiting (3.5316x1012 m3 / s2 for Kerbin)
        mu = self.ctrl.vessel.orbit.body.gravitational_parameter
        # dv_circ_burn = sqrt(mu / r_ap) - sqrt((r_pe * mu) / (r_ap * (r_pe + r_ap) / 2))
        return math.sqrt(mu / r_ap) - math.sqrt((r_pe * mu) / (r_ap * (r_pe + r_ap) / 2.0))

    def get_circularization_burn_point(self):
        return self.ctrl.ut + self.ctrl.time_to_apoapsis



class CoastToApoapsis(State):
    # def init_state(self):
    #     self.dv_circ_burn = get_dv_needed_for_circularization(self.ctrl)
    #     self.burn_time_for_circle = get_burn_time_for_dv(self.dv_circ_burn, self.ctrl)
    #     self.burn_start_time = self.ctrl.ut + self.ctrl.time_to_apoapsis - (self.burn_time_for_circle / 2.0)
    #     self.burn_end_time = self.burn_start_time + self.burn_time_for_circle

    def run(self):
        burn_condition_aquired = False
        print 'COAST'
        self.ctrl.pitch_follow = PitchManager.ORBIT_PROGRADE
        self.ctrl.vessel.auto_pilot.engage()
        self.ctrl.vessel.auto_pilot.target_roll = PreLaunch.TARGET_ROLL

        last_second = datetime.now().second
        i = 0

        while True:
            i += 1
            curr_second = datetime.now().second
            if curr_second != last_second:
                print
                print 'tks:', i
                print 'alt:', self.ctrl.altitude
                print 'follow:', self.ctrl.pitch_follow
                print 'apoapsis:', self.ctrl.apoapsis_altitude
                print 'periapsis:', self.ctrl.periapsis_altitude
                print 'dv:', self.ctrl.burn_dv
                print 'bt:', self.ctrl.burn_time
                print 'bs:', self.ctrl.burn_start
                print 'ut:', self.ctrl.ut
                # print 'be:', self.burn_end_time

                i = 0
                last_second = curr_second

            # # if self.ctrl.time_to_apoapsis > (90 + self.burn_time_for_circle / 2.0 - self.burn_time_for_circle * 0.1):
            # if self.ctrl.ut < burn_start_time - 15:
            #     pass
            #     # self.ctrl.space_center.rails_warp_factor = 3
            # # elif self.ctrl.time_to_apoapsis > (15 + self.burn_time_for_circle / 2.0 - self.burn_time_for_circle * 0.1):
            # elif self.ctrl.ut < burn_start_time:
            #     pass
            #     # self.ctrl.space_center.physics_warp_factor = 4
            # # elif self.ctrl.time_to_apoapsis > (self.burn_time_for_circle / 2.0 - self.burn_time_for_circle * 0.1):
            # # elif self.ctrl.burn_manager.burning:
            #     # self.ctrl.space_center.physics_warp_factor = 0
            #     # self.ctrl.set_throttle(1.0)
            #     # pass
            # else:
            if self.ctrl.burn_start:
                if self.ctrl.ut < self.ctrl.burn_start - 4:
                    self.ctrl.vessel.auto_pilot.stopping_time = (2.0, 2.0, 2.0)
                elif self.ctrl.ut < self.ctrl.burn_start - 2:
                    if not burn_condition_aquired:
                        print 'CoastToApoapsis acquire...'
                        self.ctrl.burn_manager.condition.acquire()
                        burn_condition_aquired = True
                    self.ctrl.vessel.auto_pilot.stopping_time = (0.5, 0.5, 0.5)
                    self.ctrl.vessel.control.rcs = False
                elif self.ctrl.ut >= self.ctrl.burn_start:
                    print 'CoastToApoapsis waiting...'
                    self.ctrl.burn_manager.condition.wait()
                    self.ctrl.burn_manager.condition.release()
                    print 'CoastToApoapsis release...'
                    # Maybe some conditional logic here to choose where to go?
                    self.ctrl.set_NextStateCls(InOrbit)
                    break


class InOrbit(State):
    def init_state(self):
        pass

    def run(self):
        self.ctrl.set_NextStateCls(CoastToInterceptBurn)

        # while True:
        #     print 'in orbit!'
        #     sleep(1)
        # pass
        # while True:
        #     if self.burn_start_time < self.ctrl.ut < self.burn_end_time:
        #         self.ctrl.space_center.physics_warp_factor = 0
        #         self.ctrl.set_throttle(1.0)
        #     else:
        #         self.ctrl.set_throttle(0.0)
        #         self.ctrl.vessel.control.rcs = False
        #         self.ctrl.set_NextStateCls(Circularize)
        #         break


class CoastToInterceptBurn(State):
    def init_state(self):
        self.active_body = self.ctrl.space_center.bodies['Kerbin']
        self.target_body = self.ctrl.space_center.bodies['Mun']

    def run(self):
        self.ctrl.target_body = self.target_body

        self.ctrl.pitch_follow = PitchManager.ORBIT_PROGRADE
        self.ctrl.vessel.auto_pilot.engage()
        self.ctrl.vessel.auto_pilot.target_roll = 0

        vessel = self.ctrl.vessel
        # space_center = self.ctrl.space_center

        # orb_flight = vessel.flight(self.ctrl.body.non_rotating_reference_frame)
        # surf_flight = vessel.flight(self.ctrl.active_body.reference_frame)
        mun = self.target_body
        kerbin = self.active_body

        # vi = math.fabs(orb_flight.speed)
        # alt = orb_flight.surface_altitude

        mun_pos = mun.position(kerbin.non_rotating_reference_frame)
        ves_pos = vessel.position(kerbin.non_rotating_reference_frame)
        phase_rads = angle_between(mun_pos, ves_pos)

        # relative_inclination = math.degrees(mun.orbit.relative_inclination(vessel))
        r1 = vessel.orbit.radius
        r2 = mun.orbit.radius
        phase_rad_needed = math.pi * (1 - math.sqrt((1 / 8.0) * (1.0 + (r1 / r2)) ** 3))
        mu = vessel.orbit.body.gravitational_parameter

        # dv_burn = math.sqrt(mu / r1) * (math.sqrt(2 * r2 / (r1 + r2)) - 1)

        # burn_time = get_burn_time_for_dv(dv_burn, vessel)

        speed_r1 = math.sqrt(mu / (r1 ** 3))
        speed_r2 = math.sqrt(mu / (r2 ** 3))
        speed_diff = speed_r2 - speed_r1

        if phase_rad_needed < phase_rads:
            rad_to_burn = phase_rads - phase_rad_needed
        else:
            rad_to_burn = 2 * math.pi + phase_rads - phase_rad_needed

        seconds_until_burn = math.fabs(rad_to_burn / speed_diff)

        # print 'bt: ', burn_time
        # print 'ub: ', seconds_until_burn


        self.ctrl.set_burn_dv(math.sqrt(mu / r1) * (math.sqrt(2 * r2 / (r1 + r2)) - 1))
        self.ctrl.set_burn_point(self.ctrl.ut + seconds_until_burn)

        # while True:
            # if seconds_until_burn > 25:
            #     space_center.rails_warp_factor = 3
            # elif seconds_until_burn > 10:
            #     space_center.rails_warp_factor = 0
            # else:
            #
            #     burn_until = self.ctrl.ut + burn_time
            #     vessel.control.throttle = 1.0
            #     set_state(MUN_BURN)
            #
            # if self.ctrl.ut > burn_start_time - 4:
            #     self.ctrl.vessel.auto_pilot.stopping_time = (2.0, 2.0, 2.0)
            # elif self.ctrl.ut > burn_start_time - 2:
            #     self.ctrl.vessel.auto_pilot.stopping_time = (0.5, 0.5, 0.5)
            #     self.ctrl.vessel.control.rcs = False
            # elif self.ctrl.ut > burn_start_time:
            #     self.ctrl.burn_manager.condition.acquire()
            #     self.ctrl.burn_manager.condition.wait()
            #     self.ctrl.burn_manager.condition.release()

        self.ctrl.vessel.control.rcs = True
        self.ctrl.vessel.auto_pilot.stopping_time = (4.0, 4.0, 4.0)
        self.ctrl.space_center.warp_to(self.ctrl.get_burn_start() - 20)
        self.ctrl.set_NextStateCls(BurnToBody)
                # break


class BurnToBody(State):
    def run(self):
        print 'BurnToBody'
        self.ctrl.pitch_follow = PitchManager.ORBIT_PROGRADE
        self.ctrl.vessel.auto_pilot.engage()
        self.ctrl.vessel.auto_pilot.target_roll = 0

        last_second = datetime.now().second
        i = 0

        while True:
            i += 1
            curr_second = datetime.now().second
            if curr_second != last_second:
                print
                print 'tks:', i
                print 'alt:', self.ctrl.altitude
                print 'follow:', self.ctrl.pitch_follow
                print 'apoapsis:', self.ctrl.apoapsis_altitude
                print 'periapsis:', self.ctrl.periapsis_altitude
                print 'dv:', self.ctrl.burn_dv
                print 'bt:', self.ctrl.burn_time
                print 'bs:', self.ctrl.burn_start
                print 'ut:', self.ctrl.ut
                # print 'be:', self.burn_end_time

                i = 0
                last_second = curr_second

            if self.ctrl.ut < self.ctrl.burn_start - 4:
                self.ctrl.vessel.auto_pilot.stopping_time = (2.0, 2.0, 2.0)
            elif self.ctrl.ut < self.ctrl.burn_start - 2:
                self.ctrl.vessel.auto_pilot.stopping_time = (0.5, 0.5, 0.5)
                self.ctrl.vessel.control.rcs = False
            elif self.ctrl.ut >= self.ctrl.burn_start:
                self.ctrl.burn_manager.condition.acquire()
                self.ctrl.burn_manager.condition.wait()
                self.ctrl.burn_manager.condition.release()
                self.ctrl.set_NextStateCls(EnrouteToTarget)
                break


class EnrouteToTarget(State):
    ALTITUDE_TARGET = 15000

    def run(self):
        print 'EnrouteToTarget'

        self.ctrl.vessel.auto_pilot.engage()

        orbit = self.ctrl.vessel.orbit.next_orbit

        periapsis_alt = self.ctrl.connection.add_stream(getattr, orbit, 'periapsis_altitude')

        if periapsis_alt() > self.ALTITUDE_TARGET:
            if orbit.inclination < math.pi / 2.0:
                self.ctrl.pitch_follow = PitchManager.ORBIT_PROGRADE
            else:
                self.ctrl.pitch_follow = PitchManager.ORBIT_RETROGRADE
        else:
            # Need to face opposite direction since we are trying to INCREASE periapsis
            if orbit.inclination < math.pi / 2.0:
                self.ctrl.pitch_follow = PitchManager.ORBIT_RETROGRADE
            else:
                self.ctrl.pitch_follow = PitchManager.ORBIT_PROGRADE

        self.ctrl.target_body = self.ctrl.space_center.bodies['Mun']

        self.ctrl.vessel.auto_pilot.wait()
        sleep(10.0)
        self.ctrl.vessel.auto_pilot.wait()
        print 'facing direction'

        TEST_DIST = 100000.0

        last_second = datetime.now().second
        i = 0

        while True:
            i += 1
            curr_second = datetime.now().second
            if curr_second != last_second:
                # orbit = self.ctrl.target_body.orbit

                print
                print 'tks:', i
                # print 'body:', self.ctrl.target_body.name
                # print 'alt:', self.ctrl.altitude
                # print 'follow:', self.ctrl.pitch_follow
                # print 'radius:', self.ctrl.target_body.equatorial_radius
                # print 'closest:', orbit.distance_at_closest_approach(self.ctrl.vessel) - self.ctrl.target_body.equatorial_radius
                # print 'periapsis:', orbit.periapsis_altitude
                # print 'eccentricity:', orbit.eccentricity
                # print 'inclination:', orbit.inclination
                # print 'dv:', self.ctrl.burn_dv
                # print 'bt:', self.ctrl.burn_time
                # print 'bs:', self.ctrl.burn_start
                # print 'ut:', self.ctrl.ut
                # print 'be:', self.burn_end_time

                i = 0
                last_second = curr_second

            if periapsis_alt() > self.ALTITUDE_TARGET:
                dist_from_tgt_alt = math.fabs(periapsis_alt() - self.ALTITUDE_TARGET)

                throttle_pct = dist_from_tgt_alt / TEST_DIST if dist_from_tgt_alt < TEST_DIST else 1.0
                if throttle_pct < 0.02:
                    throttle_pct = 0.02

                self.ctrl.vessel.control.throttle = throttle_pct
            elif periapsis_alt() < self.ALTITUDE_TARGET * 0.75:
                dist_from_tgt_alt = math.fabs(periapsis_alt() - self.ALTITUDE_TARGET * 0.75)

                throttle_pct = dist_from_tgt_alt / TEST_DIST if dist_from_tgt_alt < TEST_DIST else 1.0
                if throttle_pct < 0.02:
                    throttle_pct = 0.02

                self.ctrl.vessel.control.throttle = throttle_pct
            else:
                self.ctrl.vessel.control.throttle = 0.0
                break


class TransitionToHover(State):
    def run(self):
        # self.ctrl.pitch_follow = PitchManager.FIXED_UP
        self.ctrl.pitch_follow = PitchManager.CUSTOM_PITCH_HEADING
        pitch, heading = get_vessel_pitch_heading(self.ctrl.vessel)
        self.ctrl.pitch_manager.custom_pitch_heading = (0, heading)

        vessel = self.ctrl.vessel

        vessel.auto_pilot.engage()
        vessel.auto_pilot.target_roll = 0

        last_second = datetime.now().second

        i = 0
        while True:
            i += 1
            curr_second = datetime.now().second
            if curr_second != last_second:
                print
                print 'tks:', i
                print 'stms:', self.ctrl._streams_open
                print 'alt:', self.ctrl.altitude
                print 'speed:', self.ctrl.vertical_speed
                print 'follow:', self.ctrl.pitch_follow
                print 'apoapsis:', self.ctrl.apoapsis_altitude
                print 'periapsis:', self.ctrl.periapsis_altitude

                i = 0
                last_second = curr_second

            if self.ctrl.apoapsis_altitude <= SurfaceHover.MIN_HEIGHT:
                # target_twr = 1.1
                # avail_twr = get_avail_twr(self.ctrl)
                # throttle_pct = target_twr / avail_twr
                self.ctrl.set_throttle(1.0)
            else:
                self.ctrl.set_throttle(0.0)

            if self.ctrl.surface_altitude > SurfaceHover.MIN_HEIGHT:
                self.ctrl.set_NextStateCls(SurfaceHover)
                break


class SurfaceHover(State):
    MIN_HEIGHT = 15.0
    MAX_HEIGHT = 20.0
    SPEED = 0.15

    def run(self):
        self.ctrl.pitch_follow = PitchManager.FIXED_UP
        self.ctrl.throttle_manager.avg_input = True

        vessel = self.ctrl.vessel

        vessel.auto_pilot.engage()
        # vessel.auto_pilot.target_roll = 0

        # conn = krpc.connect(name='User Interface Example')
        canvas = self.ctrl.connection.ui.stock_canvas

        # Get the size of the game window in pixels
        screen_size = canvas.rect_transform.size

        # Add a panel to contain the UI elements
        panel = canvas.add_panel()

        # Position the panel on the left of the screen
        rect = panel.rect_transform
        rect.size = (200, 100)
        rect.position = (110 - (screen_size[0] / 2), 0)

        # Add a button to set the throttle to maximum
        fwd_btn = panel.add_button("Forward")
        fwd_btn.rect_transform.position = (0, 30)
        hvr_btn = panel.add_button("Hover")
        hvr_btn.rect_transform.position = (0, 0)
        rvs_btn = panel.add_button("Reverse")
        rvs_btn.rect_transform.position = (0, -30)

        # Set up a stream to monitor the throttle button
        fwd_btn_clicked = self.ctrl.connection.add_stream(getattr, fwd_btn, 'clicked')
        hvr_btn_clicked = self.ctrl.connection.add_stream(getattr, hvr_btn, 'clicked')
        rvs_btn_clicked = self.ctrl.connection.add_stream(getattr, rvs_btn, 'clicked')

        last_second = datetime.now().second
        i = 0
        while True:
            i += 1
            curr_second = datetime.now().second

            pitch, heading = get_vessel_pitch_heading(self.ctrl.vessel)

            if curr_second != last_second:
                print
                print 'tks:', i
                print 'stms:', self.ctrl._streams_open
                print 'alt:', self.ctrl.altitude
                print 'speed:', self.ctrl.vertical_speed
                print 'follow:', self.ctrl.pitch_follow
                print 'ptch_head:', self.ctrl.pitch_manager.custom_pitch_heading
                print 'pitch:', pitch
                print 'heading:', heading
                print 'apoapsis:', self.ctrl.apoapsis_altitude
                print 'periapsis:', self.ctrl.periapsis_altitude
                print

                i = 0
                last_second = curr_second

            # Handle the throttle button being clicked
            # print fwd_btn_clicked(), '|', hvr_btn_clicked(), '|', rvs_btn_clicked()
            if fwd_btn_clicked():
                self.pitch_follow = PitchManager.CUSTOM_PITCH_HEADING
                # flight = self.vessel.flight(self.vessel.surface_velocity_reference_frame)
                pitch, heading = get_vessel_pitch_heading(self.ctrl.vessel)
                self.ctrl.pitch_manager.custom_pitch_heading = (-60, heading)
                fwd_btn.clicked = False
            elif hvr_btn_clicked():
                self.pitch_follow = PitchManager.CUSTOM_PITCH_HEADING
                pitch, heading = get_vessel_pitch_heading(self.ctrl.vessel)
                self.ctrl.pitch_manager.custom_pitch_heading = (0, heading)
                hvr_btn.clicked = False
            elif rvs_btn_clicked():
                self.pitch_follow = PitchManager.CUSTOM_PITCH_HEADING
                pitch, heading = get_vessel_pitch_heading(self.ctrl.vessel)
                self.ctrl.pitch_manager.custom_pitch_heading = (60, heading)
                rvs_btn.clicked = False

            avail_twr = get_avail_twr(self.ctrl)
            surf_apoapsis_alt = self.ctrl.surface_altitude + self.ctrl.apoapsis_altitude
            if surf_apoapsis_alt <= self.MIN_HEIGHT:
                if self.ctrl.vertical_speed < 0:
                    target_twr = avail_twr
                else:
                    target_twr = 1.25
            elif self.MIN_HEIGHT < surf_apoapsis_alt < self.MAX_HEIGHT:
                if self.ctrl.vertical_speed <= -self.SPEED:
                    target_twr = 1.75
                elif -self.SPEED < self.ctrl.vertical_speed < self.SPEED:
                    spread = (self.MAX_HEIGHT - self.MIN_HEIGHT) / 2.0
                    mid_alt = self.MIN_HEIGHT + spread
                    alt_diff = mid_alt - surf_apoapsis_alt

                    target_twr = 1.0 + (alt_diff / spread) * 0.5
                else:
                    target_twr = 0.25

            else:
                # if self.ctrl.vertical_speed < 0:
                #     target_twr = 1.25
                # else:
                target_twr = 0.5

            throttle_pct = target_twr / avail_twr
            self.ctrl.set_throttle(throttle_pct)
