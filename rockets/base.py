import abc
import os
import time
from threading import Thread

from autopilot_system import AutoPilotSystem


def cls():
    os.system('cls' if os.name == 'nt' else 'clear')


class BaseRocket(AutoPilotSystem, metaclass=abc.ABCMeta):
    auto_pilot = None
    __stage = 0

    def __init__(self, conn, vessel):
        super().__init__(conn=conn, vessel=vessel)

        self.auto_pilot = self.vessel.auto_pilot

    def lift_off(self, altitude, throttle=1, freeze_commands=False, pitch=90, heading=90):
        """
        pitch (float) – Target pitch angle, in degrees between -90° and +90°.
        heading (float) – Target heading angle, in degrees between 0° and 360°.
        """
        self.rocket_log.print_rocket_status('LIFT OFF: ON')

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

            self.rocket_log.print_rocket_status('LIFT OFF: OFF')
            self.vessel.control.throttle = 0

        except Exception as error:
            self.__catastrophic_failure(method_name='lift_off', error=error)

        if freeze_commands:
            self.rocket_log.print_rocket_status('COMMANDS BLOCKED UNTIL GOAL ALTITUDE: ON')
            self.set_prograde(rcs=False)
            mean_apoapsis_altitude = self.conn.get_call(getattr, self.vessel.flight(), 'mean_altitude')
            expr = self.conn.krpc.Expression.greater_than(
                self.conn.krpc.Expression.call(mean_apoapsis_altitude),
                self.conn.krpc.Expression.constant_double(altitude * .9))
            event = self.conn.krpc.add_event(expr)
            with event.condition:
                event.wait()
            self.rocket_log.print_rocket_status('COMMANDS BLOCKED UNTIL GOAL ALTITUDE: OFF')

        self.auto_pilot.disengage()

    def landing_with_mech_jeb(self, touchdown_speed=3, latitude=-0.09726583022256104, longitude=-74.55767038089346):
        """
        Starts all threads needed to landing the rocket using mechJeb
        This method just work in active rocket
        latitude=-0.09726583022256104, longitude=-74.55767038089346 (Launch Pad)
        """
        threads = []

        t = Thread(target=self.exec_auto_landing_using_mech_jeb, args=(touchdown_speed, latitude, longitude))
        t.start()
        threads.append(t)

        for t in threads:
            t.join()

        self.rocket_log.print_rocket_status('ALL SYSTEM OF LANDING: OFF')

    def landing_anywhere(self, touchdown_speed=3):
        """
        Starts all threads needed to landing the rocket using just script
        This method just work with any rocket
        """

        threads = self.get_process_to_landing()

        t = Thread(target=self.__open_legs, args=())
        t.start()
        threads.append(t)

        t = Thread(target=self.exec_auto_landing_anywhere, args=(touchdown_speed,))
        t.start()
        threads.append(t)

        for t in threads:
            t.join()

        self.rocket_log.print_rocket_status('ALL SYSTEM OF LANDING: OFF')

    @abc.abstractmethod
    def set_stages(self, stage: int):
        """
        Use self.stage to control your rocket
        """
        raise NotImplementedError('Method "stages" not implemented')

    @abc.abstractmethod
    def get_process_to_landing(self):
        """
        Implement to return a list(Thread) with each process that needs to be executed in the landing process
        """
        raise NotImplementedError('Method "stages" not implemented, if not need to return []')

    def next_stage(self):
        self.set_stages(self.__stage)
        self.__stage += 1

    def exec_toggle_action_group(self, action_group: int):
        self.vessel.control.toggle_action_group(action_group)

    def __open_legs(self, altitude_to_open=500):
        self.rocket_log.print_rocket_status('SYSTEM OF LEGS: ON')
        event = self._get_event_less_then(goal='surface_altitude', value=altitude_to_open)
        with event.condition:
            event.wait()

        self.vessel.control.gear = True
        self.rocket_log.print_rocket_status('SYSTEM OF LEGS: OFF')

    def _get_event_less_then(self, goal: str, value: int):
        """
        Create time warp until get this goal
        """
        srf_altitude = self.conn.get_call(getattr, self.vessel.flight(), goal)
        expr = self.conn.krpc.Expression.less_than(
            self.conn.krpc.Expression.call(srf_altitude),
            self.conn.krpc.Expression.constant_double(value))
        return self.conn.krpc.add_event(expr)
