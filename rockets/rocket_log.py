from telemetry import Telemetry


class RocketLog:
    vessel = None
    conn = None

    def __init__(self, conn, vessel):
        self.vessel = vessel
        self.conn = conn

    def print_rocket_status(self, status: str):
        telemetry = Telemetry(vessel=self.vessel, conn=self.conn)
        latitude = telemetry.get_latitude()
        longitude = telemetry.get_longitude()
        vert_speed = telemetry.get_vertical_speed()
        try:
            print(
                f'COMMAND: {status}\n'
                f'SITUATION: {self.vessel.situation}\n'
                f'SPEED: {self.vessel.flight(self.vessel.orbit.body.reference_frame).speed}\n'
                f'V SPEED: {vert_speed()}\n'
                f'LATITUDE: {latitude()}\n'
                f'LONGITUDE: {longitude()}\n\n'
            )
        except Exception as error:
            "Telemetry FAIL"
            print(error)
