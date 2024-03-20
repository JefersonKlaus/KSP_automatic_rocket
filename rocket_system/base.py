import abc
import os
import time
from threading import Thread

from .autopilot_system import AutoPilotSystem, RocketSystemRunning


def cls():
    os.system("cls" if os.name == "nt" else "clear")


class BaseRocket(AutoPilotSystem, metaclass=abc.ABCMeta):
    auto_pilot = None
    current_stage = None

    def __init__(self, conn, vessel):
        super().__init__(conn=conn, vessel=vessel)

        self.space_center = conn.space_center
        self.vessel = vessel

        self.control = self.vessel.control
        self.orbit = self.vessel.orbit
        self.reference_frame = self.vessel.orbit.body.reference_frame

    def lift_off_in_the_direction(
        self, altitude, throttle=1, pitch=90, heading=90, stage_limit=None
    ):
        """

        Args:
            altitude (int): altitude to turn off the engines
            throttle: (int): from 0 to 1
            pitch (float): Target pitch angle, in degrees between -90° and +90°.
            heading (float): Target heading angle, in degrees between 0° and 360°.
            stage_limit (int): 
            
        Returns:
            None
        """
        self.exec_lift_off_in_direction(
            altitude=altitude,
            throttle=throttle,
            pitch=pitch,
            heading=heading,
            stage_limit=stage_limit,
            func_nex_stage=self.next_stage,
        )

    def landing_with_mech_jeb(
        self,
        touchdown_speed=3,
        latitude=-0.09726583022256104,
        longitude=-74.55767038089346,
    ):
        """
        Starts all threads needed to landing the rocket using mechJeb
        This method just work in active rocket
        latitude=-0.09726583022256104, longitude=-74.55767038089346 (Launch Pad)
        """
        threads = []

        t = Thread(
            target=self.exec_auto_landing_using_mech_jeb,
            args=(touchdown_speed, latitude, longitude),
        )
        t.start()
        threads.append(t)

        for t in threads:
            t.join()

    def landing_anywhere(self, touchdown_speed=3):
        # TODO: to implement
        """
        Starts all threads needed to landing the rocket using just script
        This method just work with any rocket
        """
        threads = []

        t = Thread(target=self.exec_auto_landing_anywhere, args=(touchdown_speed,))
        t.start()
        threads.append(t)

        t = Thread(target=self.__open_legs, args=())
        t.start()
        threads.append(t)

        threads = threads + self.get_process_to_landing()

        for t in threads:
            t.join()

        self.rocket_log.print_rocket_status("ALL SYSTEM OF LANDING: OFF")

    @abc.abstractmethod
    def set_stages(self, stage: int):
        """
        Use self.stage to control your rocket
        """
        raise NotImplementedError('Method "stages" not implemented')

    @abc.abstractmethod
    def get_process_to_landing(self):
        """
        Implement to return a list(StoppableThread) with each process that needs to be executed in the landing process

        threads = []

        altitude_to_start = int(self.vessel.orbit.body.atmosphere_depth / 2)
        t = Thread(target=self._parachutes_system, args=(altitude_to_start,))
        t.start()
        threads.append(t)

        t = Thread(target=self._shield_system, args=())
        t.start()
        threads.append(t)

        t = Thread(target=self._shield_decouple_system, args=())
        t.start()
        threads.append(t)

        t = Thread(target=self._delivery_system, args=())
        t.start()
        threads.append(t)

        return threads
        """
        raise NotImplementedError(
            'Method "stages" not implemented, if not need to return []'
        )

    def next_stage(self):
        """
        the stage count is descending 5, 4, 3, 2
        """
        self.set_stages(stage=self.vessel.control.current_stage - 1)

    def exec_toggle_action_group(self, action_group: int):
        self.vessel.control.toggle_action_group(action_group)

    def __open_legs(self, altitude_to_open=500):
        self.rocket_log.print_rocket_status("SYSTEM OF LEGS: ON")
        event = self._get_event_less_then(
            goal="surface_altitude", value=altitude_to_open
        )
        with event.condition:
            if self.system_running != RocketSystemRunning.LANDING:
                self.rocket_log.print_rocket_status("SYSTEM OF LEGS: CANCELED")
                return None
            event.wait()

        self.vessel.control.gear = True
        self.rocket_log.print_rocket_status("SYSTEM OF LEGS: OFF")

    def _get_event_less_then(self, goal: str, value: int):
        """
        Create time warp until get this goal
        """
        srf_altitude = self.conn.get_call(getattr, self.vessel.flight(), goal)
        expr = self.conn.krpc.Expression.less_than(
            self.conn.krpc.Expression.call(srf_altitude),
            self.conn.krpc.Expression.constant_double(value),
        )
        return self.conn.krpc.add_event(expr)
