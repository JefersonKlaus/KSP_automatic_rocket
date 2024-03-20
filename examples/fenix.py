import time

from rockets.base import BaseRocket
from threading import Thread
import os

cls = lambda: os.system("cls")


class Fenix4(BaseRocket):
    ROVER_MODULE_NAME = "rover_v1"

    def set_stages(self):
        if self.__stage == 0:
            for part in self.vessel.parts.with_tag("engine-stage-0"):
                part.engine.active = True

        elif self.__stage == 1:
            for part in self.vessel.parts.with_tag("decoupler-stage-1"):
                part.decoupler.decouple()

            for part in self.vessel.parts.with_tag("sepratron-stage-1"):
                part.engine.active = True

        else:
            print("SEM ESTAGIO CONFIGURADO")
            pass

        super().next_stage()

    def set_abort_control(self, status: bool):
        if status:
            self.auto_pilot.disengage()
            self.vessel.control.throttle = 0
            _count = 0
            while _count < 10:
                self.vessel.control.activate_next_stage()
                _count += 1

    def get_process_to_landing(self):
        threads = []

        # self._delivery_system()

        t = Thread(target=self._delivery_system, args=())
        t.start()
        threads.append(t)

        return threads

    def _delivery_system(self, altitude_to_start=150):
        self.rocket_log.print_rocket_status("SYSTEM OF DELIVERY: ON")
        event = self._get_event_less_then(
            goal="surface_altitude", value=altitude_to_start
        )
        with event.condition:
            event.wait()

        # extend cable
        self.exec_toggle_action_group(1)
        time.sleep(10)
        # stop cable
        self.exec_toggle_action_group(2)

        all_vessels = self.space_center.vessels
        lander_vessel = all_vessels[
            [
                i
                for i, item in enumerate(all_vessels)
                if item.name == self.ROVER_MODULE_NAME
            ][0]
        ]

        with self.conn.stream(getattr, lander_vessel, "situation") as situation:
            situation.condition.acquire()
            while situation() != self.space_center.VesselSituation.landed:
                situation.wait()

            lander_vessel.parts.decouplers[0].decouple()
            self.exec_toggle_action_group(3)

        # TODO: improve this code
        self.auto_pilot.reference_frame = self.vessel.surface_reference_frame
        self.auto_pilot.target_pitch_and_heading(45, 90)
        self.auto_pilot.target_roll = float("nan")
        self.set_sas_mode(sas=False)
        self.auto_pilot.engage()
        self.auto_pilot.wait()

        self.rocket_log.print_rocket_status("SYSTEM OF DELIVERY: OFF")
