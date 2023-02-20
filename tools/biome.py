import krpc


class Biome:
    def __init__(self, conn, vessel):
        self.conn = conn
        self.vessel = vessel
        self.latitude = None
        self.longitude = None
        self.altitude = None
        self.name = None

    def get_name(self):
        self._set_biome_attributes()
        return self.name

    def _set_biome_attributes(self):
        self.name = self.vessel.biome

        vessel_flight = self.vessel.flight()
        self.latitude = vessel_flight.latitude
        self.longitude = vessel_flight.longitude
        self.altitude = vessel_flight.surface_altitude
