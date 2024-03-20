import curses

from tools.biome import Biome
from tools.telemetry import Telemetry


class RocketLog:
    vessel = None
    conn = None

    def __init__(self, conn, vessel):
        self.vessel = vessel
        self.conn = conn

    def keep_printing_rocket_status(self, status: str = ""):
        telemetry = Telemetry(vessel=self.vessel, conn=self.conn)
        biome = Biome(vessel=self.vessel, conn=self.conn)

        import curses
        import time

        # Inicializa a biblioteca curses
        stdscr = curses.initscr()
        curses.noecho()
        curses.cbreak()

        try:
            # Executa vários prints
            while True:
                # Move o cursor para a última linha
                stdscr.move(curses.LINES - 1, 0)
                # Limpa a linha
                stdscr.clrtoeol()

                # Escreve o texto desejado
                stdscr.addstr(0, 0, f"----- ROCKET -----")
                stdscr.addstr(1, 0, f"STAGE: {self.vessel.control.current_stage}")
                stdscr.addstr(2, 0, f"SITUATION: {self.vessel.situation}")
                stdscr.addstr(3, 0, f"VELOCITY: {telemetry.get_velocity()}")
                stdscr.addstr(4, 0, f"V SPEED: {telemetry.get_vertical_speed()}\n")
                stdscr.addstr(5, 0, f"LATITUDE: {telemetry.get_latitude()}\n")
                stdscr.addstr(6, 0, f"LONGITUDE: {telemetry.get_longitude()}")
                stdscr.addstr(7, 0, f"PITCH: {telemetry.get_pitch()}")
                stdscr.addstr(8, 0, f"HEAD: {telemetry.get_heading()}")

                stdscr.addstr(9, 0, f"----- BIOME -----")
                stdscr.addstr(1, 0, f"NAME: {biome.get_name()}")
                stdscr.addstr(1, 0, f"SCIENCE: {biome.get_science()}")

                # Atualiza o terminal
                stdscr.refresh()
                # Aguarda um tempo para exibir o próximo print
                time.sleep(1)

        finally:
            # Restaura as configurações originais do terminal
            curses.echo()
            curses.nocbreak()
            curses.endwin()
