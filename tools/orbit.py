import math
from math import e
import time
import numpy as np

class Orbit:

    def __init__(self, conn, vessel):
        self.conn = conn
        self.space_center = self.conn.space_center
        
        self.vessel = vessel
        self.auto_pilot = self.vessel.auto_pilot

        self.orbit = self.vessel.orbit
        self.body = self.space_center.bodies[self.orbit.body.name]

    def get_target(self):
        target = self.space_center.target_vessel
        if target:
            return target
        target = self.space_center.target_body
        if target:
            return target

        return None
    
    def create_circularization_node(self):
        ut = self.conn.add_stream(getattr, self.space_center, "ut")
        mu = self.body.gravitational_parameter
        r = self.orbit.apoapsis
        a1 = self.orbit.semi_major_axis
        a2 = r
        v1 = math.sqrt(mu * ((2.0 / r) - (1.0 / a1)))
        v2 = math.sqrt(mu * ((2.0 / r) - (1.0 / a2)))
        delta_v = v2 - v1
        return self.vessel.control.add_node(
            ut() + self.orbit.time_to_apoapsis, prograde=delta_v
        )

    def execute_next_node_v2(self, node, buffer_time=20.0, slow_burn_time=0.5):
        nd = node
        reference_frame = self.vessel.surface_reference_frame
        burn_direction = nd.direction(reference_frame)
        self.auto_pilot.target_direction = burn_direction

        dt = 1/20.0

        burn_time = self.get_burn_time(nd)
        burn_throttle = 1.0

        small_burn_ratio = 5.0
        if burn_time < 2.0:
            burn_throttle /= small_burn_ratio
            burn_time *= small_burn_ratio

        # Orient ship
        burn_direction = nd.direction(reference_frame)
        ship_direction = self.vessel.direction(reference_frame)

        self.vessel.control.sas = False
        self.auto_pilot.engage()
        while not np.dot(burn_direction, ship_direction) >= .98:
            self.auto_pilot.target_direction = burn_direction
            time.sleep(dt)
            reference_frame = self.vessel.surface_reference_frame
            burn_direction = nd.direction(reference_frame)
            ship_direction = self.vessel.direction(reference_frame)

        self.auto_pilot.disengage()

        print("Expecting {}s burn.".format(burn_time))
        warp_time = nd.ut - (burn_time / 2.0) - buffer_time
        self.space_center.warp_to(warp_time)

        self.auto_pilot.engage()
        reference_frame = self.vessel.surface_reference_frame
        burn_direction = nd.remaining_burn_vector(reference_frame)
        self.vessel.auto_pilot.target_direction = burn_direction

        while self.space_center.ut < nd.ut - (burn_time/2.0):
            time.sleep(dt)

        self.vessel.control.throttle = burn_throttle
        # Perform most of the burn, stopping when half a second is remaining.
        while self.get_burn_time(nd.remaining_delta_v) > slow_burn_time:
            reference_frame = self.vessel.surface_reference_frame
            burn_direction = nd.remaining_burn_vector(reference_frame)
            self.auto_pilot.target_direction = burn_direction
            if self.vessel.thrust < 1e-6:
                pass
                from IPython import embed; embed()
                # self.stage_if_needed()

        # Start slow burn
        self.vessel.control.throttle = self.slow_burn_throttle
        # Home in on remainder of burn.
        while nd.remaining_delta_v > 0.5:
            burn_direction = nd.remaining_burn_vector(reference_frame)
            self.vessel.auto_pilot.target_direction = burn_direction
        # Throttle Off
        self.vessel.control.throttle = 0.0
        self.auto_pilot.disengage()
        nd.remove()


    def exec_node(self, node, rcs=False):
        self.auto_pilot.engage()
        time.sleep(0.001)

        self.vessel.control.rcs = rcs
        time.sleep(0.001)

        # set the position
        self.auto_pilot.reference_frame = node.reference_frame
        self.auto_pilot.target_direction = (0, 1, 0)
        self.auto_pilot.wait()

        # get burn time
        F = self.vessel.available_thrust
        Isp = self.vessel.specific_impulse * 9.82
        m0 = self.vessel.mass
        m1 = m0 / math.exp(node.remaining_delta_v / Isp)
        flow_rate = F / Isp
        burn_time = (m0 - m1) / flow_rate

        # get current time and move until burn_time/2 less a security margin
        remaining_time = node.time_to
        current_ut = self.conn.space_center.ut
        target_ut = current_ut + remaining_time - (burn_time / 2) - 10
        self.conn.space_center.warp_to(target_ut)

        # waite to get the target time
        while self.conn.space_center.ut < target_ut:
            time.sleep(0.1)

        # turn on the engine
        self.vessel.control.throttle = 1
        while node.remaining_delta_v > 15:
            time.sleep(0.1)

        # reduce to 10% and wait to reduce to 0.5%
        self.vessel.control.throttle = 0.1
        las_remaining_delta_v = node.remaining_delta_v
        while node.remaining_delta_v > 1:
            time.sleep(0.1)
            if node.remaining_delta_v > las_remaining_delta_v:
                break
            else:
                las_remaining_delta_v = node.remaining_delta_v

        # reduce to 0.5% and wait to turn off
        self.vessel.control.throttle = 0.005
        las_remaining_delta_v = node.remaining_delta_v
        while node.remaining_delta_v > 0:
            time.sleep(0.1)
            if node.remaining_delta_v > las_remaining_delta_v:
                break
            else:
                las_remaining_delta_v = node.remaining_delta_v

        # turn off
        self.vessel.control.throttle = 0
        self.auto_pilot.disengage()
        self.vessel.control.rcs = False
        node.remove()


   
    def get_burn_time(self, node):
        remaining_delta_v = node.delta_v
        body = self.vessel.orbit.body
        g = body.gravitational_parameter / self.vessel.orbit.radius

        isp = self.vessel.specific_impulse
        m_init = self.vessel.mass
        m_final = m_init * e ** (-remaining_delta_v / (isp * g))
        m_prop = m_init - m_final
        m_dot = self.vessel.max_thrust / (isp * g)
        burn_time = m_prop / m_dot
        return burn_time