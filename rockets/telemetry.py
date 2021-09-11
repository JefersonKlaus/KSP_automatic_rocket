class Telemetry:
    conn = None
    vessel = None

    def __init__(self, conn, vessel):
        self.conn = conn
        self.vessel = vessel

    def get_altitude(self):
        return self.conn.add_stream(getattr, self.vessel.flight(), 'surface_altitude')

    def get_ut(self):
        return self.conn.add_stream(getattr, self.conn.space_center, 'ut')

    def get_aerodynamic_force(self):
        return self.conn.add_stream(getattr, self.vessel.flight(), 'aerodynamic_force')

    def get_mass(self):
        return self.conn.add_stream(getattr, self.vessel, 'mass')

    def get_moment_of_inertia(self):
        return self.conn.add_stream(getattr, self.vessel, 'moment_of_inertia')

    def get_inertia_tensor(self):
        return self.conn.add_stream(getattr, self.vessel, 'inertia_tensor')

    def get_center_of_mass(self):
        return self.conn.add_stream(getattr, self.vessel.flight(), 'center_of_mass')

    def get_available_torque(self):
        return self.conn.add_stream(getattr, self.vessel, 'available_torque')

    def get_available_thrust(self):
        return self.conn.add_stream(getattr, self.vessel, 'available_thrust')

    def get_surface_gravity(self):
        return self.conn.add_stream(getattr, self.vessel.orbit.body, 'surface_gravity')

    def get_latitude(self):
        return self.conn.add_stream(getattr, self.vessel.flight(), 'latitude')

    def get_longitude(self):
        return self.conn.add_stream(getattr, self.vessel.flight(), 'longitude')

    def get_atmosphere_density(self):
        return self.conn.add_stream(getattr, self.vessel.flight(), 'atmosphere_density')

    def get_dynamic_pressure(self):
        return self.conn.add_stream(getattr, self.vessel.flight(), 'dynamic_pressure')

    def get_static_pressure(self):
        return self.conn.add_stream(getattr, self.vessel.flight(), 'static_pressure')

    def get_lift(self):
        return self.conn.add_stream(getattr, self.vessel.flight(), 'lift')

    def get_drag(self):
        return self.conn.add_stream(getattr, self.vessel.flight(), 'drag')

    def get_pitch(self):
        return self.conn.add_stream(getattr, self.vessel.flight(), 'pitch')

    def get_heading(self):
        return self.conn.add_stream(getattr, self.vessel.flight(), 'heading')

    def get_roll(self):
        return self.conn.add_stream(getattr, self.vessel.flight(), 'roll')

    def get_throttle_control(self):
        return self.conn.add_stream(getattr, self.vessel.control, 'throttle')

    def get_right_control(self):
        return self.conn.add_stream(getattr, self.vessel.control, 'right')

    def get_up_control(self):
        return self.conn.add_stream(getattr, self.vessel.control, 'up')

    def get_pitch_control(self):
        return self.conn.add_stream(getattr, self.vessel.control, 'pitch')

    def get_roll_control(self):
        return self.conn.add_stream(getattr, self.vessel.control, 'roll')

    def get_yaw_control(self):
        return self.conn.add_stream(getattr, self.vessel.control, 'yaw')

    def get_roll_pid_gains(self):
        return self.conn.add_stream(getattr, self.auto_pilot, 'roll_pid_gains')

    def get_yaw_pid_gains(self):
        return self.conn.add_stream(getattr, self.auto_pilot, 'yaw_pid_gains')

    def get_pitch_pid_gains(self):
        return self.conn.add_stream(getattr, self.auto_pilot, 'pitch_pid_gains')
