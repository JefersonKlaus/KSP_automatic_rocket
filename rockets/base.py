import abc
import os
import time
from math import cos, pi, sin
from threading import Thread

from numpy import arccos, clip, dot, inf, linalg, math
from telemetry import Telemetry


def cls():
    os.system('cls' if os.name == 'nt' else 'clear')


class BaseRocket(metaclass=abc.ABCMeta):
    vessel = None
    auto_pilot = None
    control = None
    space_center = None
    mech_jeb = None
    stage = 0
    reference_frame = None
    __background_threads = []

    def __init__(self, conn, vessel):
        self.conn = conn
        self.vessel = vessel
        self.mech_jeb = self.conn.mech_jeb
        self.space_center = self.conn.space_center
        self.reference_frame = self.vessel.orbit.body.reference_frame

        self.__setup_abort_system()

        self.auto_pilot = self.vessel.auto_pilot
        self.control = self.vessel.control

        self.__load_data_stream()

    def set_stability_assist(self, sas=True, rcs=True):
        self.__set_sas_mode('stability_assist', sas=sas, rcs=rcs)

    def set_retrograde(self, sas=True, rcs=True):
        self.__set_sas_mode('retrograde', sas=sas, rcs=rcs)

    def set_prograde(self, sas=True, rcs=True):
        self.__set_sas_mode('prograde', sas=sas, rcs=rcs)

    def set_radial(self, sas=True, rcs=True):
        self.__set_sas_mode('radial', sas=sas, rcs=rcs)

    def lift_off(self, altitude, throttle=1, freeze_commands=False, pitch=90, heading=90):
        """
        pitch (float) – Target pitch angle, in degrees between -90° and +90°.
        heading (float) – Target heading angle, in degrees between 0° and 360°.
        """
        self.__print_status('LIFT OFF: ON')

        try:
            self.auto_pilot.engage()
            self.auto_pilot.target_pitch_and_heading(pitch, heading)

            time.sleep(0.001)

            self.vessel.control.sas = False
            self.vessel.control.rcs = False

            self.vessel.control.throttle = throttle

            apoapsis_altitude = self.conn.get_call(getattr, self.vessel.orbit, 'apoapsis_altitude')
            expr = self.conn.krpc.Expression.greater_than(
                self.conn.krpc.Expression.call(apoapsis_altitude),
                self.conn.krpc.Expression.constant_double(altitude))
            event = self.conn.krpc.add_event(expr)
            with event.condition:
                event.wait()

            self.__print_status('LIFT OFF: OFF')
            self.vessel.control.throttle = 0

        except Exception as error:
            self.__catastrophic_failure(method_name='lift_off', error=error)

        if freeze_commands:
            self.__print_status('COMMANDS BLOCKED UNTIL GOAL ALTITUDE: ON')
            self.set_prograde(rcs=False)
            mean_apoapsis_altitude = self.conn.get_call(getattr, self.vessel.flight(), 'mean_altitude')
            expr = self.conn.krpc.Expression.greater_than(
                self.conn.krpc.Expression.call(mean_apoapsis_altitude),
                self.conn.krpc.Expression.constant_double(altitude * .9))
            event = self.conn.krpc.add_event(expr)
            with event.condition:
                event.wait()
            self.__print_status('COMMANDS BLOCKED UNTIL GOAL ALTITUDE: OFF')

        self.auto_pilot.disengage()

    def landing_with_mech_jeb(self, altitude_airbrake=None, touchdown_speed=3, latitude=-0.097, longitude=-74.5565):
        """
        Starts all threads needed to landing the rocket using mechJeb
        This method just work in active rocket
        """
        threads = []

        # TODO: create a method to start airbrake when the rocket has
        # t = Thread(target=self.__toggle_airbrake, args=(altitude_airbrake,))
        # t.start()
        # threads.append(t)

        t = Thread(target=self.__mech_jeb_landing, args=(touchdown_speed, latitude, longitude))
        t.start()
        threads.append(t)

        for t in threads:
            t.join()

        self.__print_status('ALL SYSTEM OF LANDING: OFF')

    def landing_anywhere(self):
        """
        Starts all threads needed to landing the rocket using just script
        This method just work with any rocket
        """

        threads = []

        # t = Thread(target=self.__toggle_airbrake, args=(altitude_airbrake,))
        # t.start()
        # threads.append(t)

        t = Thread(target=self.__open_legs, args=())
        t.start()
        threads.append(t)

        t = Thread(target=self.__script_landing_anywhere, args=())
        t.start()
        threads.append(t)

        for t in threads:
            t.join()

        self.__print_status('ALL SYSTEM OF LANDING: OFF')

    def time_to_suicide_burn(self, alt=0):
        '''
       Returns an estimate of how many seconds until you need to burn at 95% throttle to avoid crashing.
       This gives a 5% safety margin.
       I do not even PRETEND to understand all of the math in this function.  It's essentially a porting
       of the routine from the Mechjeb orbit extensions.
       '''
        rf = self.space_center.ReferenceFrame.create_hybrid(
            position=self.vessel.orbit.body.reference_frame,
            rotation=self.vessel.surface_reference_frame)

        if self.vessel.orbit.periapsis_altitude > 0:  ## We're not on a landing trajectory yet.
            return inf

        # calculate sin of angle from horizontal -
        v1 = self.vessel.velocity(rf)
        v2 = (0, 0, 1)
        angle_from_horizontal = self.__angle_between(v1, v2)
        sine = math.sin(angle_from_horizontal)

        # estimate deceleration time
        g = self.vessel.orbit.body.surface_gravity
        T = (self.vessel.max_thrust / self.vessel.mass)  # calculating with 5% safety margin!
        effective_decel = .5 * (-2 * g * sine + math.sqrt((2 * g * sine) * (2 * g * sine) + 4 * (T * T - g * g)))
        decel_time = self.vessel.flight(rf).speed / effective_decel

        # estimate time until burn
        radius = self.vessel.orbit.body.equatorial_radius + alt
        TA = self.vessel.orbit.true_anomaly_at_radius(radius)
        TA = -1 * TA  # look on the negative (descending) side of the orbit
        impact_time = self.vessel.orbit.ut_at_true_anomaly(TA)
        burn_time = impact_time - decel_time / 2
        ground_track = ((burn_time - self.space_center.ut) * self.vessel.flight(rf).speed) + (
                .5 * self.vessel.flight(rf).speed * decel_time)
        # print(ground_track)
        # TODO: study about ground_track
        return burn_time - self.space_center.ut

    def throttle_to_suicide_burn(self):
        v_speed = self.conn.add_stream(getattr, self.vessel.flight(self.reference_frame), 'vertical_speed')

        # Altitude PID values
        p = 0. - self.altitude()
        d = 0. - v_speed()
        T0 = self.available_thrust()
        m = self.mass()
        F0 = m * self.gravity()
        throttle = (F0 + 300 * p + 7000 * d) / T0

        if self.altitude() < 300:
            throttle = throttle * .95  # 95% of throttle

        return throttle

    def draw_landing_site(self, landing_reference_frame):
        try:
            self.conn.drawing.add_line((0, 0, 0), (1, 0, 0), landing_reference_frame)
            self.conn.drawing.add_line((0, 0, 0), (0, 1, 0), landing_reference_frame)
            self.conn.drawing.add_line((0, 0, 0), (0, 0, 1), landing_reference_frame)
        except Exception as error:
            print(error)

    def __unit_vector(self, vector):
        """ Returns the unit vector of the vector.  """
        return vector / linalg.norm(vector)

    def do_when_landing(self):
        """
        First thing that rocket will do when touch the floor
        called by __script_landing_anywhere
        """
        raise NotImplementedError()

    @abc.abstractmethod
    def set_abort_control(self, status):
        raise NotImplementedError('Methodo "set_abort_control" não implementado')

    @abc.abstractmethod
    def set_stages(self):
        """
        Use self.stage to control your rocket
        """
        raise NotImplementedError('Methodo "stages" não implementado')

    def next_stage(self):
        self.set_stages()
        self.stage += 1

    def __angle_between(self, v1, v2):
        """ Returns the angle in radians between vectors 'v1' and 'v2'"""
        v1_u = self.__unit_vector(v1)
        v2_u = self.__unit_vector(v2)
        return arccos(clip(dot(v1_u, v2_u), -1.0, 1.0))

    def __mech_jeb_landing(self, touchdown_speed, latitude, longitude):
        self.__print_status('LANDING WITH MECHJEB: ON')
        try:
            self.vessel.control.rcs = True

            self.mech_jeb.target_controller.set_position_target(
                body=self.vessel.orbit.body,
                latitude=latitude,
                longitude=longitude
            )

            landing_autopilot = self.mech_jeb.landing_autopilot
            landing_autopilot.enabled = True

            landing_autopilot.touchdown_speed = touchdown_speed
            landing_autopilot.land_at_position_target()

            # cut throttle and enable SAS when landed
            while self.vessel.situation != self.space_center.VesselSituation.landed:
                time.sleep(0.01)
            self.control.throttle = 0.0

            landing_autopilot.enabled = False
            self.set_stability_assist()

        except Exception as error:
            self.__catastrophic_failure(method_name='__mech_jeb_landing', error=error)

        self.__print_status('LANDING WITH MECHJEB: OFF')

    def __script_landing_anywhere(self):
        self.__print_status('LANDING WITH SCRIPT: ON')

        # TODO: create feature to landing on defined position
        create_relative = self.conn.space_center.ReferenceFrame.create_relative
        # Coordinates of landing site
        landing_latitude = -(0 + (5.0 / 60) + (48.38 / 60 / 60))
        landing_longitude = -(74 + (37.0 / 60) + (12.2 / 60 / 60))
        landing_altitude = 111

        # Determine landing site reference frame
        # (orientation: x=zenith, y=north, z=east)
        landing_position = self.vessel.orbit.body.surface_position(
            landing_latitude, landing_longitude, self.vessel.orbit.body.reference_frame)
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
                        self.vessel.orbit.body.reference_frame,
                        landing_position,
                        q_long),
                    (0, 0, 0),
                    q_lat),
                (landing_altitude, 0, 0))

        self.draw_landing_site(landing_reference_frame)

        self.__load_data_stream()
        # velocity = self.conn.add_stream(self.vessel.velocity, landing_reference_frame)
        # v_speed = self.conn.add_stream(getattr, self.vessel.flight(landing_reference_frame), 'vertical_speed')
        # h_speed = self.conn.add_stream(getattr, self.vessel.flight(landing_reference_frame), 'horizontal_speed')
        # position = self.conn.add_stream(self.vessel.position, landing_reference_frame)
        # rotation = self.conn.add_stream(self.vessel.rotation, landing_reference_frame)
        # direction = self.conn.add_stream(self.vessel.direction, landing_reference_frame)
        # angular_velocity = self.conn.add_stream(self.vessel.angular_velocity, landing_reference_frame)

        # Define autopilot reference frame
        self.auto_pilot.reference_frame = self.vessel.surface_reference_frame
        time.sleep(0.01)
        self.auto_pilot.engage()
        time.sleep(0.01)

        self.set_retrograde()

        while self.time_to_suicide_burn() > 0:
            time.sleep(0.01)
            self.auto_pilot.disengage()
            time.sleep(0.01)
            print(self.time_to_suicide_burn())
            time.sleep(.5)
            continue

        # time.sleep(0.01)
        # self.auto_pilot.engage()
        # time.sleep(0.01)

        engine_on = False
        while True:
            throttle = self.throttle_to_suicide_burn()

            if self.altitude() < 300:
                self.set_radial()

            if throttle >= 1 or engine_on:  # begin when needs 100% throttle
                engine_on = True
                self.control.throttle = throttle

            if self.vessel.situation == self.space_center.VesselSituation.landed:
                try:
                    self.do_when_landing()
                except NotImplementedError:
                    self.control.throttle = 0.0
                    self.set_stability_assist()
                break

        self.__print_status('LANDING WITH SCRIPT: OFF')

    def __load_data_stream(self):
        telemetry = Telemetry(self.conn, self.vessel)
        self.altitude = telemetry.get_altitude()
        self.mass = telemetry.get_mass()
        self.available_thrust = telemetry.get_available_thrust()
        self.gravity = telemetry.get_surface_gravity()

        self.latitude = telemetry.get_latitude()
        self.longitude = telemetry.get_longitude()

    def __toggle_airbrake(self, altitude_airbrake=10000):
        # # TODO: get altitude of airbrake automatically
        # # PEGAR ALTURA ALTOMATICA
        # srf_altitude = self.conn.get_call(getattr, self.vessel.flight(), 'surface_altitude')
        # expr = self.conn.krpc.Expression.less_than(
        #     self.conn.krpc.Expression.call(srf_altitude),
        #     self.conn.krpc.Expression.constant_double(altitude_airbrake))
        # event = self.conn.krpc.add_event(expr)
        event = self.__get_event_less_then(goal='surface_altitude', value=altitude_airbrake)
        with event.condition:
            event.wait()

        self.vessel.control.toggle_action_group(1)

    def __open_legs(self, altitude_to_open=500):
        self.__print_status('SYSTEM OF LEGS: ON')
        # srf_altitude = self.conn.get_call(getattr, self.vessel.flight(), 'surface_altitude')
        # expr = self.conn.krpc.Expression.less_than(
        #     self.conn.krpc.Expression.call(srf_altitude),
        #     self.conn.krpc.Expression.constant_double(altitude_airbrake))
        # event = self.conn.krpc.add_event(expr)
        event = self.__get_event_less_then(goal='surface_altitude', value=altitude_to_open)
        with event.condition:
            event.wait()

        self.vessel.control.gear = True
        self.__print_status('SYSTEM OF LEGS: OFF')

    def __get_event_less_then(self, goal: str, value: int):
        """
        Create time warp until get this goal
        """
        srf_altitude = self.conn.get_call(getattr, self.vessel.flight(), goal)
        expr = self.conn.krpc.Expression.less_than(
            self.conn.krpc.Expression.call(srf_altitude),
            self.conn.krpc.Expression.constant_double(value))
        return self.conn.krpc.add_event(expr)

    def __set_sas_mode(self, sas_mode_name, sas=True, rcs=True):
        self.__print_status('SAS MODE: ' + sas_mode_name)

        self.auto_pilot.disengage()
        time.sleep(0.001)
        self.control.sas = sas
        self.control.rcs = rcs
        time.sleep(0.001)

        try:
            if sas_mode_name == 'prograde':
                self.control.sas_mode = self.conn.space_center.SASMode.prograde

            elif sas_mode_name == 'retrograde':
                self.control.sas_mode = self.conn.space_center.SASMode.retrograde

            elif sas_mode_name == 'stability_assist':
                self.control.sas_mode = self.conn.space_center.SASMode.stability_assist

            elif sas_mode_name == 'radial':
                self.control.sas_mode = self.conn.space_center.SASMode.radial

            else:
                print('\n' * 2)
                print('ALERTA: SAS_MODE "' + sas_mode_name + '" nao encontrado')
                print('\n' * 2)

        except Exception as error:
            self.__catastrophic_failure(method_name='__set_sas_mode ' + sas_mode_name, error=error)

    def __setup_abort_system(self):
        abort = self.conn.add_stream(getattr, self.vessel.control, 'abort')
        abort.add_callback(self.set_abort_control)
        abort.start()

    def __catastrophic_failure(self, error, method_name):
        print('\n' * 2)
        self.__print_status('WARNING: CATASTROPHIC FAILURE "' + method_name + '"')
        print(error)
        print('\n' * 2)

        print('ABORTING!!  ' * 2)
        self.set_abort_control(status=True)

    def __print_status(self, status):
        try:
            print(
                f'COMMAND: {status}\n'
                f'SITUATION: {self.vessel.situation}\n'
                f'SPEED: {self.vessel.flight(self.reference_frame).speed}\n'
                f'LATITUDE: {self.latitude()}\n'
                f'LONGITUDE: {self.longitude()}\n'
            )
        except Exception as error:
            "Telemetry FAIL"
            print(error)
