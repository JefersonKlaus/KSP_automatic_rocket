from rockets.base import BaseRocket


class PythonX1(BaseRocket):
    def set_stages(self, stage):
        if stage == 4:
            self.vessel.control.activate_next_stage()

        if stage == 3:
            self.vessel.control.activate_next_stage()

        elif stage == 2:
            self.vessel.control.activate_next_stage()

        else:
            print("SEM STAGIO")

    def set_abort_control(self, status):
        if status:
            self.auto_pilot.disengage()
            self.vessel.control.throttle = 0
            _count = 0
            while _count < 10:
                self.vessel.control.activate_next_stage()
                _count += 1

    def get_process_to_landing(self):
        return []
