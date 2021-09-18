import abc
import time
from math import cos, pi, sin

from rocket_log import RocketLog
from suicide_burn import SuicideBurn
from telemetry import Telemetry


class AutoPilotSystem:
    conn = None
    vessel = None
    space_center = None
    reference_frame = None
    mech_jeb = None
    suicide_burn = None
    rocket_log: RocketLog = None

    def __init__(self, conn, vessel):
        self.conn = conn
        self.space_center = self.conn.space_center
        self.vessel = vessel
        self.reference_frame = self.vessel.orbit.body.reference_frame
        self.mech_jeb = self.conn.mech_jeb

        self.suicide_burn = SuicideBurn(vessel=vessel, conn=conn, reference_frame=self.reference_frame)
        self.rocket_log = RocketLog(vessel=vessel, conn=conn)

        telemetry = Telemetry(self.conn, self.vessel)
        self.altitude = telemetry.get_altitude()

        self.__setup_abort_system()

    def set_sas_mode(self, sas=True, rcs=True):
        self.__set_sas_mode('off', sas=sas, rcs=rcs)

    def set_stability_assist(self, sas=True, rcs=True):
        self.__set_sas_mode('stability_assist', sas=sas, rcs=rcs)

    def set_retrograde(self, sas=True, rcs=True):
        self.__set_sas_mode('retrograde', sas=sas, rcs=rcs)

    def set_prograde(self, sas=True, rcs=True):
        self.__set_sas_mode('prograde', sas=sas, rcs=rcs)

    def set_radial(self, sas=True, rcs=True):
        self.__set_sas_mode('radial', sas=sas, rcs=rcs)

    def exec_auto_landing_using_mech_jeb(self, touchdown_speed=3, latitude=None, longitude=None):
        self.rocket_log.print_rocket_status('LANDING WITH MECHJEB: ON')
        try:
            self.vessel.control.rcs = True

            if latitude and longitude:
                self.mech_jeb.target_controller.set_position_target(
                    body=self.vessel.orbit.body,
                    latitude=latitude,
                    longitude=longitude
                )

            landing_autopilot = self.mech_jeb.landing_autopilot
            landing_autopilot.enabled = True

            landing_autopilot.touchdown_speed = touchdown_speed
            if latitude and longitude:
                landing_autopilot.land_at_position_target()
            else:
                landing_autopilot.land_untargeted()

            # cut throttle and enable SAS when landed
            while self.vessel.situation != self.space_center.VesselSituation.landed:
                time.sleep(0.01)
            self.vessel.control.throttle = 0.0

            landing_autopilot.enabled = False
            self.set_stability_assist()

        except Exception as error:
            self.__catastrophic_failure(method_name='exec_autopilot_using_mech_jeb_landing', error=error)

        self.rocket_log.print_rocket_status('LANDING WITH MECHJEB: OFF')

    def exec_auto_landing_at_position(self, touchdown_speed=3):
        self.rocket_log.print_rocket_status('LANDING WITH SCRIPT: ON')
        _alt_control = 1000  # altitude to change landing process

        # TODO: create feature to landing on defined position
        create_relative = self.conn.space_center.ReferenceFrame.create_relative
        # Coordinates of landing site
        landing_latitude = -(0 + (5.0 / 60) + (48.38 / 60 / 60))
        landing_longitude = -(74 + (37.0 / 60) + (12.2 / 60 / 60))
        landing_altitude = 111

        # Determine landing site reference frame
        # (orientation: x=zenith, y=north, z=east)
        landing_position = self.vessel.orbit.body.surface_position(
            landing_latitude, landing_longitude, self.reference_frame)
        q_long = (
            0,
            sin(-landing_longitude * 0.5 * pi / 180),
            0,
            cos(-landing_longitude * 0.5 * pi / 180)
        )
        q_lat = (
            0,
            0,
            sin(landing_latitude * 0.5 * pi / 180),
            cos(landing_latitude * 0.5 * pi / 180)
        )
        landing_reference_frame = \
            create_relative(
                create_relative(
                    create_relative(
                        self.reference_frame,
                        landing_position,
                        q_long),
                    (0, 0, 0),
                    q_lat),
                (landing_altitude, 0, 0))

        # self.draw_landing_site(landing_reference_frame)

        # Define autopilot reference frame
        self.auto_pilot.reference_frame = self.vessel.surface_reference_frame
        time.sleep(0.01)
        # self.auto_pilot.engage()
        # time.sleep(0.01)

        # time.sleep(0.01)
        self.auto_pilot.disengage()
        time.sleep(0.01)
        self.set_retrograde()

        while (self.suicide_burn.time_to_suicide_burn() > 0) and (self.altitude() > (_alt_control * 10)):
            print(self.time_to_suicide_burn())
            time.sleep(.5)
            continue

        # time.sleep(0.01)
        # self.auto_pilot.engage()
        # time.sleep(0.01)

        engine_on = False
        radial_on = False
        while True:
            throttle = self.suicide_burn.throttle_to_suicide_burn(touchdown_speed=touchdown_speed)

            if self.altitude() < 300 and not radial_on:
                self.set_radial()
                radial_on = True

            if (
                    throttle >= 1 or self.altitude() < _alt_control) or engine_on:  # begin when needs 100% throttle or low altitude
                engine_on = True
                self.vessel.control.throttle = throttle

            if self.vessel.situation == self.space_center.VesselSituation.landed:
                self.vessel.control.throttle = 0.0
                self.set_stability_assist()
                break

        self.rocket_log.print_rocket_status('LANDING WITH SCRIPT: OFF')

    def exec_auto_landing_anywhere(self, touchdown_speed=3):
        self.rocket_log.print_rocket_status('LANDING ANYWHERE: ON')
        _alt_control = 1000  # altitude to change landing process

        # Define autopilot reference frame
        self.auto_pilot.reference_frame = self.vessel.surface_reference_frame
        time.sleep(0.01)
        # self.auto_pilot.engage()
        # time.sleep(0.01)

        # time.sleep(0.01)
        self.auto_pilot.disengage()
        time.sleep(0.01)
        self.set_retrograde()

        while (self.suicide_burn.time_to_suicide_burn() > 0) and (self.altitude() > (_alt_control * 10)):
            print(self.time_to_suicide_burn())
            time.sleep(.5)
            continue

        # time.sleep(0.01)
        # self.auto_pilot.engage()
        # time.sleep(0.01)

        engine_on = False
        radial_on = False
        while True:
            throttle = self.suicide_burn.throttle_to_suicide_burn(touchdown_speed=touchdown_speed)

            if self.altitude() < 300 and not radial_on:
                self.set_radial()
                radial_on = True

            if (
                    throttle >= 1 or self.altitude() < _alt_control) or engine_on:  # begin when needs 100% throttle or low altitude
                engine_on = True
                self.vessel.control.throttle = throttle

            if self.vessel.situation == self.space_center.VesselSituation.landed:
                self.vessel.control.throttle = 0.0
                self.set_stability_assist()
                break

        self.rocket_log.print_rocket_status('LANDING ANYWHERE: OFF')

    @abc.abstractmethod
    def set_abort_control(self, status):
        raise NotImplementedError('Method "set_abort_control" not implemented')

    def __set_sas_mode(self, sas_mode_name, sas=True, rcs=True):
        self.rocket_log.print_rocket_status('SAS MODE: ' + sas_mode_name)

        self.auto_pilot.disengage()
        time.sleep(0.001)
        self.vessel.control.sas = sas
        self.vessel.control.rcs = rcs
        time.sleep(0.001)

        try:
            if sas_mode_name == 'prograde':
                self.vessel.control.sas_mode = self.conn.space_center.SASMode.prograde

            elif sas_mode_name == 'retrograde':
                self.vessel.control.sas_mode = self.conn.space_center.SASMode.retrograde

            elif sas_mode_name == 'stability_assist':
                self.vessel.control.sas_mode = self.conn.space_center.SASMode.stability_assist

            elif sas_mode_name == 'radial':
                self.vessel.control.sas_mode = self.conn.space_center.SASMode.radial

            elif sas_mode_name == 'off':
                pass

            else:
                print('\n' * 2)
                print('ALERTA: SAS_MODE "' + sas_mode_name + '" not found')
                print('\n' * 2)

        except Exception as error:
            self.__catastrophic_failure(method_name='__set_sas_mode ' + sas_mode_name, error=error)

    def __setup_abort_system(self):
        abort = self.conn.add_stream(getattr, self.vessel.control, 'abort')
        abort.add_callback(self.set_abort_control)
        abort.start()

    def __catastrophic_failure(self, error, method_name):
        print('\n' * 2)
        self.rocket_log.print_rocket_status('WARNING: CATASTROPHIC FAILURE "' + method_name + '"')
        print(error)
        print('\n' * 2)

        print('ABORTING!!  ' * 2)
        self.set_abort_control(status=True)

    # def draw_landing_site(self, landing_reference_frame):
    #     try:
    #         self.conn.drawing.add_line((0, 0, 0), (1, 0, 0), landing_reference_frame)
    #         self.conn.drawing.add_line((0, 0, 0), (0, 1, 0), landing_reference_frame)
    #         self.conn.drawing.add_line((0, 0, 0), (0, 0, 1), landing_reference_frame)
    #     except Exception as error:
    #         print(error)
