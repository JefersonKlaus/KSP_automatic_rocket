from rocket_system.base import BaseRocket


class BasicRocket(BaseRocket):
    def set_stages(self, stage):
        if stage >= 1:
            self.active_stage()
            return
        else:
            # print(stage)
            # print("SEM STAGIO")
            return

    def set_abort_control(self, status: bool):
        if status:
            self.auto_pilot.disengage()
            self.vessel.control.throttle = 0

            for _ in range(10, 0,):
                self.active_stage()


    def get_process_to_landing(self):
        return []
