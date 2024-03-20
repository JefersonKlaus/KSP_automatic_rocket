import abc
import time
import math
from enum import Enum

from tools.telemetry import Telemetry

from .suicide_burn import SuicideBurn


class RocketSystemRunning(Enum):
    NONE = 0
    LANDING = 1
    TAKING_OFF = 2
    FLYING = 3
    ABORTING = 4


class AutoPilotSystem:
    conn = None
    vessel = None
    space_center = None
    reference_frame = None
    mech_jeb = None
    suicide_burn = None
    telemetry = None

    system_running = RocketSystemRunning.NONE

    def __init__(self, conn, vessel):
        self.conn = conn
        self.space_center = self.conn.space_center
        self.vessel = vessel
        self.auto_pilot = self.vessel.auto_pilot
        self.reference_frame = self.vessel.orbit.body.reference_frame
        self.mech_jeb = self.conn.mech_jeb
        self.telemetry = Telemetry(self.conn, self.vessel)

        # TODO: update suicideBurn code
        self.suicide_burn = SuicideBurn(
            vessel=vessel, conn=conn, reference_frame=self.reference_frame
        )

        # TODO: remove it
        self.altitude = self.telemetry.get_latitude_stream()

        self.__setup_abort_system()

    def set_sas_mode(self, sas=True, rcs=True):
        self.__set_sas_mode("off", sas=sas, rcs=rcs)

    def set_stability_assist(self, sas=True, rcs=True):
        self.__set_sas_mode("stability_assist", sas=sas, rcs=rcs)

    def set_retrograde(self, sas=True, rcs=True):
        self.__set_sas_mode("retrograde", sas=sas, rcs=rcs)

    def set_prograde(self, sas=True, rcs=True):
        self.__set_sas_mode("prograde", sas=sas, rcs=rcs)

    def set_radial(self, sas=True, rcs=True):
        self.__set_sas_mode("radial", sas=sas, rcs=rcs)

    def exec_lift_off_in_direction(
        self,
        altitude,
        throttle=1,
        pitch=90,
        heading=90,
        stage_limit=None,
        func_nex_stage=None,
    ):
        """
        Args:
            altitude (int): altitude to turn off the engines
            throttle: (int): from 0 to 1
            pitch (float): Target pitch angle, in degrees between -90° and +90°.
            heading (float): Target heading angle, in degrees between 0° and 360°.
            stage_limit (int):
            func_nex_stage (function): if you needs control stages of you plane/rocket send a function in attribue func_next_stage

        Returns:
            None
        """
        try:
            self.auto_pilot.engage()
            time.sleep(0.001)

            self.vessel.control.sas = False
            self.vessel.control.rcs = False
            time.sleep(0.001)

            self.vessel.auto_pilot.reference_frame = self.reference_frame
            self.auto_pilot.target_pitch_and_heading(pitch, heading)

            # Define a altitude desejada
            self.orbit.target_altitude = altitude

            # Ativa o motor e faz o lançamento
            if throttle:
                self.vessel.control.throttle = throttle
            else:
                self.vessel.control.throttle = self.get_engine_efficiency(
                    altitude=self.telemetry.get_altitude(),
                    speed=self.telemetry.get_velocity(),
                    mass=self.telemetry.get_mass(),
                    drag=self.telemetry.get_drag(),
                )

            if func_nex_stage:
                func_nex_stage()

            while self.vessel.available_thrust == 0:
                pass

            # Espera até que o foguete esteja fora do solo
            while self.vessel.flight().mean_altitude < 300:
                pass

            # Loop principal do lançamento
            while True:
                print(
                    self.vessel.resources_in_decouple_stage(
                        self.vessel.control.current_stage - 1
                    ).amount("LiquidFuel")
                )
                # Verifica se o combustível do estágio atual acabou
                # FIXME: there is a bug (KRPC) that was need puth current stage -1 to get the right fuel
                if (
                    self.vessel.resources_in_decouple_stage(
                        self.vessel.control.current_stage - 1
                    ).amount("LiquidFuel")
                    < 0.1
                    and self.vessel.resources_in_decouple_stage(
                        self.vessel.control.current_stage - 1
                    ).amount("SolidFuel")
                    < 0.1
                ):
                    # Troca para o próximo estágio
                    if stage_limit >= self.vessel.control.current_stage:
                        break
                    else:
                        if func_nex_stage:
                            func_nex_stage()

                # Verifica se o apoastro atingiu a altitude desejada
                if self.orbit.apoapsis_altitude >= altitude:
                    # Desliga o motor
                    self.control.throttle = 0.0
                    break

        except Exception as error:
            self.__catastrophic_failure(method_name="lift_off", error=error)

        self.auto_pilot.disengage()

    def exec_auto_landing_using_mech_jeb(
        self, touchdown_speed=3, latitude=None, longitude=None
    ):
        self.system_running = RocketSystemRunning.LANDING

        try:
            self.vessel.control.rcs = True

            if latitude and longitude:
                self.mech_jeb.target_controller.set_position_target(
                    body=self.vessel.orbit.body, latitude=latitude, longitude=longitude
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
                if self.system_running != RocketSystemRunning.LANDING:
                    return None
                time.sleep(0.01)
            self.vessel.control.throttle = 0.0

            landing_autopilot.enabled = False
            self.set_stability_assist()

        except Exception as error:
            self.__catastrophic_failure(
                method_name="exec_autopilot_using_mech_jeb_landing", error=error
            )

        self.system_running = RocketSystemRunning.NONE

    def exec_auto_landing_at_position(self, touchdown_speed=3):
        raise NotImplementedError(
            'Method "exec_auto_landing_at_position" not implemented'
        )

    def exec_auto_landing_anywhere(self, touchdown_speed=3):
        """
        This method always needed to be the first method when it is called on a thread array.

        """
        self.system_running = RocketSystemRunning.LANDING
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

        while (self.suicide_burn.time_to_suicide_burn() > 0) and (
            self.altitude() > _alt_control
        ):
            if self.system_running != RocketSystemRunning.LANDING:
                return None
            time.sleep(0.5)
            continue

        # time.sleep(0.01)
        # self.auto_pilot.engage()
        # time.sleep(0.01)

        engine_on = False
        radial_on = False
        while True:
            if self.system_running != RocketSystemRunning.LANDING:
                return None

            throttle = self.suicide_burn.throttle_to_suicide_burn(
                touchdown_speed=touchdown_speed
            )

            if self.altitude() < 300 and not radial_on:
                self.set_radial()
                radial_on = True

            if (
                throttle >= 1 or self.altitude() < _alt_control
            ) or engine_on:  # begin when needs 100% throttle or low altitude
                engine_on = True
                self.vessel.control.throttle = throttle

            if self.vessel.situation == self.space_center.VesselSituation.landed:
                self.vessel.control.throttle = 0.0
                self.set_stability_assist()
                break

        self.system_running = RocketSystemRunning.NONE

    def get_engine_efficiency(self, altitude, speed, mass, drag):
        """
        Calcula a eficiência do motor do foguete baseado na altitude, velocidade, massa e arrasto.

        Args:
            altitude (float): Altitude atual do foguete, em metros.
            speed (float): Velocidade atual do foguete, em m/s.
            mass (float): Massa total do foguete, em kg.
            drag (tuple): Força de arrasto em X, Y e Z, em Newtons.

        Returns:
            float: Porcentagem de potência do motor necessária para garantir melhor eficiência.
        """

        # Define os coeficientes para o cálculo da eficiência do motor.
        c1 = 0.5
        c2 = 0.0001
        c3 = 0.00001

        # Calcula a eficiência do motor baseada na altitude, velocidade, massa e arrasto.
        x = altitude / 1000
        y = speed / 1000
        z = mass / 1000
        drag_magnitude = (drag[0] ** 2 + drag[1] ** 2 + drag[2] ** 2) ** 0.5
        efficiency = (
            c1 + c2 * (1 - z) * (1 - (x / 100)) * (1 + (y / 100)) - c3 * drag_magnitude
        )

        # Limita a eficiência do motor entre 0 e 100%.
        efficiency = max(0, efficiency)
        efficiency = min(1, efficiency)

        # Retorna a porcentagem de potência do motor necessária para garantir melhor eficiência.
        return efficiency * 100

    @abc.abstractmethod
    def set_abort_control(self, status: bool):
        raise NotImplementedError('Method "set_abort_control" not implemented')

    def __set_sas_mode(self, sas_mode_name, sas=True, rcs=True):
        self.auto_pilot.disengage()
        time.sleep(0.001)
        self.vessel.control.sas = sas
        self.vessel.control.rcs = rcs
        time.sleep(0.001)

        try:
            if sas_mode_name == "prograde":
                self.vessel.control.sas_mode = self.conn.space_center.SASMode.prograde

            elif sas_mode_name == "retrograde":
                self.vessel.control.sas_mode = self.conn.space_center.SASMode.retrograde

            elif sas_mode_name == "stability_assist":
                self.vessel.control.sas_mode = (
                    self.conn.space_center.SASMode.stability_assist
                )

            elif sas_mode_name == "radial":
                self.vessel.control.sas_mode = self.conn.space_center.SASMode.radial

            elif sas_mode_name == "off":
                pass

            else:
                print("\n" * 2)
                print('ALERTA: SAS_MODE "' + sas_mode_name + '" not found')
                print("\n" * 2)

        except Exception as error:
            pass
            # self.__catastrophic_failure(method_name='__set_sas_mode ' + sas_mode_name, error=error)

    def __setup_abort_system(self):
        abort = self.conn.add_stream(getattr, self.vessel.control, "abort")
        abort.add_callback(self.set_abort_control)
        abort.start()

    def __catastrophic_failure(self, error, method_name):
        print("\n" * 2)
        # self.rocket_log.print_rocket_status(
        #     'WARNING: CATASTROPHIC FAILURE "' + method_name + '"'
        # )
        print(error)
        print("\n" * 2)

        print("ABORTING!!  " * 2)
        self.set_abort_control(status=True)
