import time

import krpc

from examples.pythonx1 import PythonX1

if __name__ == "__main__":
    print("... conectando ...")
    conn = krpc.connect(name="Rocket command")
    print("!!! conectado !!!")

    vessel = conn.space_center.active_vessel

    for i in range(5, 0, -1):
        print(str(i) + " ...")
        time.sleep(1)

    print(vessel)

    python_x_1 = PythonX1(conn=conn, vessel=vessel)
    python_x_1.lift_off_in_the_direction(
        altitude=100000, pitch=90, heading=270, stage_limit=2
    )
