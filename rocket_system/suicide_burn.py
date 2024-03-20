from numpy import arccos, clip, dot, inf, linalg, math
from telemetry import Telemetry


class SuicideBurn:
    conn = None
    vessel = None
    reference_frame = None
    space_center = None

    def __init__(self, conn, vessel, reference_frame):
        self.conn = conn
        self.vessel = vessel
        self.reference_frame = reference_frame
        self.space_center = self.conn.space_center

        telemetry = Telemetry(self.conn, self.vessel)
        self.altitude = telemetry.get_altitude()
        self.mass = telemetry.get_mass()
        self.available_thrust = telemetry.get_available_thrust()
        self.gravity = telemetry.get_surface_gravity()
        self.v_speed = telemetry.get_vertical_speed()

    def time_to_suicide_burn(self, floor=0):
        rf = self.space_center.ReferenceFrame.create_hybrid(
            position=self.reference_frame,
            rotation=self.vessel.surface_reference_frame)

        if self.vessel.orbit.periapsis_altitude > 0:  ## We're not on a landing trajectory yet.
            return inf

        # calculate sin of angle from horizontal -
        v1 = self.vessel.velocity(rf)
        v2 = (0, 0, 1)
        angle_from_horizontal = self.__angle_between(v1, v2)
        sine = math.sin(angle_from_horizontal)

        # estimate deceleration time
        g = self.gravity()
        # g = self.vessel.orbit.body.surface_gravity
        T = (self.vessel.max_thrust / self.mass())
        effective_decel = .5 * (-2 * g * sine + math.sqrt((2 * g * sine) * (2 * g * sine) + 4 * (T * T - g * g)))
        decel_time = self.vessel.flight(rf).speed / effective_decel

        # estimate time until burn
        radius = self.vessel.orbit.body.equatorial_radius + floor
        TA = self.vessel.orbit.true_anomaly_at_radius(radius)
        TA = -1 * TA  # look on the negative (descending) side of the orbit
        impact_time = self.vessel.orbit.ut_at_true_anomaly(TA)
        burn_time = impact_time - decel_time / 2
        ground_track = ((burn_time - self.space_center.ut) * self.vessel.flight(rf).speed) + (
                .5 * self.vessel.flight(rf).speed * decel_time)
        # print(ground_track)
        # TODO: study about ground_track
        return burn_time - self.space_center.ut

    def throttle_to_suicide_burn(self, touchdown_speed=3):
        # Altitude PID values
        p = 0. - self.altitude()
        d = (-1 * float(touchdown_speed)) - self.v_speed()
        to = self.available_thrust()
        m = self.mass()
        f0 = m * self.gravity()
        throttle = (f0 + 300 * p + 7000 * d) / to

        return throttle

    def __angle_between(self, v1, v2):
        """ Returns the angle in radians between vectors 'v1' and 'v2'"""
        v1_u = self.__unit_vector(v1)
        v2_u = self.__unit_vector(v2)
        return arccos(clip(dot(v1_u, v2_u), -1.0, 1.0))

    def __unit_vector(self, vector):
        """ Returns the unit vector of the vector.  """
        return vector / linalg.norm(vector)
