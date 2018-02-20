from threading import Lock


fuel_lock = Lock()

decouple_stage_activate_next = {}


def has_fuel_callback(has_fuel):
    # pass
    #
    print 'fuel: {}'.format(has_fuel)
    #
    # print 'fut: {}'.format(ctrl.ut)

    if not has_fuel:
        with fuel_lock:
            from controller import Controller
            from util import get_current_stage, get_current_decouple_stage
            ctrl = Controller.get()
            # vessel = ctrl.vessel
            # max_decouple_stage = max([part.decouple_stage for part in vessel.parts.all])
            # current_decouple_stage = ctrl.current_decouple_stage
            # engines_to_decouple = [part for part in vessel.parts.in_decouple_stage(max_decouple_stage) if part.engine]
            engines_to_decouple = ctrl.get_parts_in_decouple_stage(vessel)

            # stage = get_current_stage(vessel)
            # stage_parts = vessel.parts.in_stage(stage)
            # all_engines_flameout = not any([part.engine.has_fuel for part in engines_to_decouple])
            # all_engines_flameout = not any([part.engine.thrust for part in engines_to_decouple])
            all_engines_flameout = not any([ctrl.get_available_thrust(part) for part in engines_to_decouple])

            print 'cds: ', ctrl.current_decouple_stage
            # print 'rsc: ', [part.engine.thrust for part in engines_to_decouple]
            print 'exh: ', all_engines_flameout
            print decouple_stage_activate_next

            if engines_to_decouple and all_engines_flameout:
                already_activated = decouple_stage_activate_next.get(ctrl.current_decouple_stage, False)
                if not already_activated:
                    decouple_stage_activate_next[ctrl.current_decouple_stage] = True
                    ctrl.vessel.control.activate_next_stage()
                    ctrl.current_stage = get_current_stage(ctrl.vessel)
                    ctrl.current_decouple_stage = get_current_decouple_stage(ctrl.vessel)

                    # if curr_state == CIRCULARIZE_BURN:
                    #     dv_circ_burn = get_dv_needed_for_circularization(vessel)
                    #     burn_until = get_burn_time_for_dv(dv_circ_burn, vessel) + ut()




