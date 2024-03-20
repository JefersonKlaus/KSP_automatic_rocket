from numpy import math

from tools.telemetry import Telemetry


class SuicideBurn:
    conn = None
    vessel = None
    reference_frame = None
    space_center = None

    telemetry = None

    def __init__(self, conn, vessel, reference_frame):
        self.conn = conn
        self.vessel = vessel
        self.reference_frame = reference_frame
        self.space_center = self.conn.space_center

        self.telemetry = Telemetry(self.conn, self.vessel)

    def suicide_burn_time(self):
        # stream
        vesel_mass = self.telemetry.get_mass_stream()
        surface_gravity = self.telemetry.get_surface_gravity_stream()
        vessel_thrust = self.telemetry.get_thrust_stream()
        drag_coeff = self.telemetry.get_drag_coefficient_stream()
        vessel_altitude = self.telemetry.get_altitude_stream()
        vessel_v_velocity = self.telemetry.get_vertical_speed_stream()

        return self._suicide_burn_time(
            mass=vesel_mass(),
            g=surface_gravity(),
            thrust=vessel_thrust(),
            drag_coeff=drag_coeff(),
            altitude=vessel_altitude(),
            velocity=vessel_v_velocity(),
            fuel_flow_rate=self.telemetry.get_fuel_flow_rate(),
        )

    def _suicide_burn_time(
        mass, g, thrust, fuel_flow_rate, drag_coeff, altitude, velocity
    ):
        """
        mass: mass of the rocket in kg
        g: gravitational acceleration in m/s^2
        thrust: engine thrust in N
        fuel_flow_rate: engine fuel burn rate in kg/s
        drag_coeff: atmospheric drag coefficient
        altitude: initial height of the rocket in m
        velocity: initial velocity of the rocket in m/s
        """
        burn_time = (mass * velocity) / (thrust - (mass * g))
        time_to_fuel_depletion = mass / fuel_flow_rate
        landing_velocity = math.sqrt(
            (2 * mass * g * altitude)
            + (velocity**2)
            - (2 * drag_coeff * altitude * mass * g)
        )
        total_time = burn_time + time_to_fuel_depletion
        time_at_70_percent_burn = 0.7 * total_time
        time_to_start_burn = time_at_70_percent_burn - burn_time
        return time_to_start_burn

    def _suicide_burn_control(
        thrust, mass, g, drag_coeff, height, desired_velocity, kp
    ):
        """
        thrust: maximum engine power in N
        mass: mass of the rocket in kg
        g: gravitational interference in m/s^2
        drag_coeff: atmospheric drag coefficient
        height: initial height of the rocket in m
        desired_velocity: desired descent speed in m/s
        kp: controller proportional gain (also known as tuning constant)
        """
        # TODO: change this metodo for only return the % that the engine need to be
        time_step = 1  # Passo de tempo em segundos
        time_to_impact = math.sqrt(2 * height / g)
        velocity = 0
        position = height
        velocity_error = desired_velocity - velocity

        while position > 0:
            drag = 0.5 * drag_coeff * velocity**2 * mass
            acceleration = (thrust - drag) / mass
            velocity += acceleration * time_step
            position -= velocity * time_step
            velocity_error = desired_velocity - velocity
            throttle = kp * velocity_error
            throttle = max(
                min(throttle, 1), 0
            )  # Limita a potência do motor entre 0 e 1
            thrust = throttle * mass * g

            # Imprime os resultados a cada iteração para fins de análise
            print("Time:", time_to_impact - position / velocity, "s")
            print("Velocity:", velocity, "m/s")
            print("Altitude:", position, "m")
            print("Throttle:", throttle)
            print("")

        return throttle
