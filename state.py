from time import sleep
from threading import Thread
import math
from datetime import datetime

from util import get_avail_twr, get_dv_needed_for_circularization, get_burn_time_for_dv
from manager import PitchManager


class State(object):
    _thread = None

    def __init__(self):
        from controller import Controller
        self.ctrl = Controller.get()
        self._thread = Thread(target=self.run, args=())
        self._thread.daemon = True
        self._thread.start()

    def run(self):

        # print self.__class__
        # sleep(2)
        pass

    def join(self):
        return self._thread.join()


class AscentState(State):
    def run(self):
        super(AscentState, self).run()

        vessel = self.ctrl.vessel

        vessel.control.throttle = 1.0
        vessel.auto_pilot.engage()
        vessel.auto_pilot.target_pitch_and_heading(90, 90)
        vessel.auto_pilot.target_roll = 0
        vessel.control.activate_next_stage()

        last_second = datetime.now().second

        i = 0
        while True:
            i += 1
            curr_second = datetime.now().second
            if curr_second != last_second:
                print
                print 'tks:', i
                print 'alt:', self.ctrl.altitude
                print 'mach:', self.ctrl.mach
                print 'follow:', self.ctrl.pitch_follow

                i = 0
                last_second = curr_second
            if self.ctrl.altitude < self.ctrl.ALTITUDE_TURN_START:
                self.ctrl.pitch_follow = PitchManager.FIXED_UP

            elif self.ctrl.altitude >= self.ctrl.ALTITUDE_TURN_START:
                if self.ctrl.altitude < 30000:
                    if self.ctrl.mach < 0.4:
                        self.ctrl.pitch_follow = PitchManager.FIXED_POINT
                    elif self.ctrl.mach > 0.65:
                        self.ctrl.pitch_follow = PitchManager.SURFACE_PROGRADE
                    elif self.ctrl.pitch_follow != PitchManager.SURFACE_PROGRADE and math.fabs(self.ctrl.angle_of_attack) > 1.0:
                        self.ctrl.pitch_follow = PitchManager.FIXED_POINT
                    else:
                        self.ctrl.pitch_follow = PitchManager.SURFACE_PROGRADE
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
                min_twr = 1.55
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

            if self.ctrl.apoapsis_altitude < self.ctrl.ALTITUDE_TARGET:
                if self.ctrl.apoapsis_altitude >= self.ctrl.ALTITUDE_TARGET * 0.975:
                    self.ctrl.set_throttle(0.25)
            else:
                self.ctrl.set_throttle(0)
                # dv_circ_burn = get_dv_needed_for_circularization(vessel)
                # burn_needed_for_circle = get_burn_time_for_dv(dv_circ_burn, vessel)
                self.ctrl.set_NextStateCls(CoastToApoapsis)
                break


class CoastToApoapsis(State):
    pass
