from rockets.base import BaseRocket
from threading import Thread
import os

cls = lambda: os.system('cls')


class Blue1(BaseRocket):

    def next_stage(self):
        if self.stage == 0:
            for part in self.vessel.parts.with_tag('engine-stage-0'):
                part.engine.active = True

        elif self.stage == 1:
            self.vessel.control.activate_next_stage()
            self.vessel.control.activate_next_stage()
            # parachute - stage - 1
            # for part in self.vessel.parts.with_tag('decoupler-stage-1'):
            #     part.decoupler.decouple()
            #
            # for part in self.vessel.parts.with_tag('sepratron-stage-1'):
            #     part.engine.active = True

        else:
            print('not configured')
            pass

        super().next_stage()

    def set_abort_control(self, status):
        if status:
            self.vessel = self.conn.space_center.active_vessel
            self.vessel.auto_pilot.disengage()
            self.vessel.control.throttle = 0
            _count = 0
            while _count < 10:
                self.vessel.control.activate_next_stage()
                _count += 1
