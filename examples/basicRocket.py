from rocket_system.base import BaseRocket


class BasicRocket(BaseRocket):
    def set_stages(self, stage):
        if stage == 4:
            self.vessel.control.activate_next_stage()
            return
        if stage == 3:
            self.vessel.control.activate_next_stage()
            return
        elif stage == 2:
            self.vessel.control.activate_next_stage()
            return
        else:
            print("SEM STAGIO")
            return

    def set_abort_control(self, status: bool):
        if status:
            self.auto_pilot.disengage()
            self.vessel.control.throttle = 0
            _count = 0
            while _count < 10:
                self.vessel.control.activate_next_stage()
                _count += 1

    def get_process_to_landing(self):
        return []
