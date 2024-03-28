class Telemetry:
    conn = None
    vessel = None

    def __init__(self, conn, vessel, reference_frame=None):
        self.conn = conn
        self.vessel = vessel

        if reference_frame:
            self.reference_frame = reference_frame
        else:
            self.reference_frame = self.vessel.orbit.body.reference_frame

        # velocity = self.conn.add_stream(self.vessel.velocity, landing_reference_frame)
        # v_speed = self.conn.add_stream(getattr, self.vessel.flight(landing_reference_frame), 'vertical_speed')
        # h_speed = self.conn.add_stream(getattr, self.vessel.flight(landing_reference_frame), 'horizontal_speed')
        # position = self.conn.add_stream(self.vessel.position, landing_reference_frame)
        # rotation = self.conn.add_stream(self.vessel.rotation, landing_reference_frame)
        # direction = self.conn.add_stream(self.vessel.direction, landing_reference_frame)
        # angular_velocity = self.conn.add_stream(self.vessel.angular_velocity, landing_reference_frame)

    def get_altitude(self):
        return self.vessel.flight().surface_altitude

    def get_ut(self):
        return self.conn.space_center.ut

    def get_aerodynamic_force(self):
        return self.vessel.flight().aerodynamic_force

    def get_g_force(self):
        return self.vessel.flight().g_force

    def get_mass(self):
        return self.vessel.mass

    def get_moment_of_inertia(self):
        return self.vessel.moment_of_inertia

    def get_inertia_tensor(self):
        return self.vessel.inertia_tensor

    def get_center_of_mass(self):
        return self.vessel.flight().center_of_mass

    def get_available_torque(self):
        return self.vessel.available_torque

    def get_thrust(self):
        return self.vessel.thrust

    def get_available_thrust(self):
        return self.vessel.available_thrust

    def get_surface_gravity(self):
        return self.vessel.orbit.body.surface_gravity

    def get_vertical_speed(self):
        return self.vessel.flight(self.reference_frame).vertical_speed

    def get_speed(self):
        return self.vessel.flight(self.reference_frame).speed

    def get_velocity(self):
        return self.vessel.flight(self.reference_frame).velocity[0]

    def get_latitude(self):
        return self.vessel.flight().latitude

    def get_longitude(self):
        return self.vessel.flight().longitude

    def get_atmosphere_density(self):
        return self.vessel.flight().atmosphere_density

    def get_dynamic_pressure(self):
        return self.vessel.flight().dynamic_pressure

    def get_static_pressure(self):
        return self.vessel.flight().static_pressure

    def get_lift(self):
        return self.vessel.flight().lift

    def get_drag(self):
        return self.vessel.flight().drag
        # return self.vessel.flight(vessel.orbit.body.reference_frame).drag

    def get_drag_coefficient(self):
        return self.vessel.flight().drag_coefficient

    def get_pitch(self):
        return self.vessel.flight().pitch

    def get_heading(self):
        return self.vessel.flight().heading

    def get_roll(self):
        return self.vessel.flight().roll

    def get_throttle_control(self):
        return self.vessel.control.throttle

    def get_right_control(self):
        return self.vessel.control.right

    def get_up_control(self):
        return self.vessel.control.up

    def get_pitch_control(self):
        return self.vessel.control.pitch

    def get_roll_control(self):
        return self.vessel.control.roll

    def get_yaw_control(self):
        return self.vessel.control.yaw

    def get_roll_pid_gains(self):
        return self.auto_pilot.roll_pid_gains

    def get_yaw_pid_gains(self):
        return self.auto_pilot.yaw_pid_gains

    def get_pitch_pid_gains(self):
        return self.auto_pilot.pitch_pid_gains

    def get_fuel_flow_rate(self):
        engines = self.vessel.parts.engines
        total_fuel_flow_rate = 0
        for engine in engines:
            total_fuel_flow_rate += engine.fuel_flow_rate

        return total_fuel_flow_rate

    # Stream
    def get_altitude_stream(self):
        return self.conn.add_stream(getattr, self.vessel.flight(), "surface_altitude")

    def get_ut_stream(self):
        return self.conn.add_stream(getattr, self.conn.space_center, "ut")

    def get_aerodynamic_force_stream(self):
        return self.conn.add_stream(getattr, self.vessel.flight(), "aerodynamic_force")

    def get_mass_stream(self):
        return self.conn.add_stream(getattr, self.vessel, "mass")

    def get_moment_of_inertia_stream(self):
        return self.conn.add_stream(getattr, self.vessel, "moment_of_inertia")

    def get_inertia_tensor_stream(self):
        return self.conn.add_stream(getattr, self.vessel, "inertia_tensor")

    def get_center_of_mass_stream(self):
        return self.conn.add_stream(getattr, self.vessel.flight(), "center_of_mass")

    def get_available_torque_stream(self):
        return self.conn.add_stream(getattr, self.vessel, "available_torque")

    def get_thrust_stream(self):
        return self.conn.add_stream(getattr, self.vessel, "thrust")

    def get_available_thrust_stream(self):
        return self.conn.add_stream(getattr, self.vessel, "available_thrust")

    def get_surface_gravity_stream(self):
        return self.conn.add_stream(getattr, self.vessel.orbit.body, "surface_gravity")

    def get_vertical_speed_stream(self):
        return self.conn.add_stream(
            getattr, self.vessel.flight(self.reference_frame), "vertical_speed"
        )

    def get_speed_stream(self):
        return self.conn.add_stream(
            getattr, self.vessel.flight(self.reference_frame), "vertical"
        )

    def get_latitude_stream(self):
        return self.conn.add_stream(getattr, self.vessel.flight(), "latitude")

    def get_longitude_stream(self):
        return self.conn.add_stream(getattr, self.vessel.flight(), "longitude")

    def get_atmosphere_density_stream(self):
        return self.conn.add_stream(getattr, self.vessel.flight(), "atmosphere_density")

    def get_dynamic_pressure_stream(self):
        return self.conn.add_stream(getattr, self.vessel.flight(), "dynamic_pressure")

    def get_static_pressure_stream(self):
        return self.conn.add_stream(getattr, self.vessel.flight(), "static_pressure")

    def get_lift_stream(self):
        return self.conn.add_stream(getattr, self.vessel.flight(), "lift")

    def get_drag_stream(self):
        return self.conn.add_stream(getattr, self.vessel.flight(), "drag")

    def get_drag_coefficient_stream(self):
        return self.conn.add_stream(getattr, self.vessel.flight(), "drag_coefficient")

    def get_pitch_stream(self):
        return self.conn.add_stream(getattr, self.vessel.flight(), "pitch")

    def get_heading_stream(self):
        return self.conn.add_stream(getattr, self.vessel.flight(), "heading")

    def get_roll_stream(self):
        return self.conn.add_stream(getattr, self.vessel.flight(), "roll")

    def get_throttle_control_stream(self):
        return self.conn.add_stream(getattr, self.vessel.control, "throttle")

    def get_right_control_stream(self):
        return self.conn.add_stream(getattr, self.vessel.control, "right")

    def get_up_control_stream(self):
        return self.conn.add_stream(getattr, self.vessel.control, "up")

    def get_pitch_control_stream(self):
        return self.conn.add_stream(getattr, self.vessel.control, "pitch")

    def get_roll_control_stream(self):
        return self.conn.add_stream(getattr, self.vessel.control, "roll")

    def get_yaw_control_stream(self):
        return self.conn.add_stream(getattr, self.vessel.control, "yaw")

    def get_roll_pid_gains_stream(self):
        return self.conn.add_stream(getattr, self.auto_pilot, "roll_pid_gains")

    def get_yaw_pid_gains_stream(self):
        return self.conn.add_stream(getattr, self.auto_pilot, "yaw_pid_gains")

    def get_pitch_pid_gains_stream(self):
        return self.conn.add_stream(getattr, self.auto_pilot, "pitch_pid_gains")
