from rockets.base import BaseRocket
from threading import Thread
import os

cls = lambda: os.system('cls')


class Fenix4(BaseRocket):

    def set_stages(self):
        if self.stage == 0:
            for part in self.vessel.parts.with_tag('engine-stage-0'):
                part.engine.active = True

        elif self.stage == 1:
            for part in self.vessel.parts.with_tag('decoupler-stage-1'):
                part.decoupler.decouple()

            for part in self.vessel.parts.with_tag('sepratron-stage-1'):
                part.engine.active = True

        else:
            print('SEM ESTAGIO CONFIGURADO')
            pass

        super().next_stage()

    def set_abort_control(self, status):
        if status:
            self.auto_pilot.disengage()
            self.control.throttle = 0
            _count = 0
            while _count < 10:
                self.control.activate_next_stage()
                _count += 1
