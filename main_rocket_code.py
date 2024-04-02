import time

import krpc

from examples.basicRocket import BasicRocket

if __name__ == "__main__":
    print("... conectando ...")
    # Nome da nave
    conn = krpc.connect(name="Python01")
    print("!!! conectado !!!")

    vessel = conn.space_center.active_vessel

    for i in range(3, 0, -1):
        print(str(i) + " ...")
        time.sleep(1)


    # Rocket example one
    example_rocket_1 = BasicRocket(conn=conn, vessel=vessel)
    example_rocket_1.lift_off_in_the_direction(
        altitude=100000, pitch=85, heading=90, stage_limit=0
    )

    example_rocket_1.set_prograde(sas=True, rcs=False)
    node = example_rocket_1.create_circularization_node()
    example_rocket_1.exec_node(node=node)