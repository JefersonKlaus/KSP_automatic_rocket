import time

import krpc

from tools.biome import Biome

if __name__ == "__main__":
    print("... conectando ...")
    conn = krpc.connect(name="Test")
    print("!!! conectado !!!")

    vessel = conn.space_center.active_vessel
    biome = Biome(vessel=vessel, conn=conn)
    auto_pilot = vessel.auto_pilot
    print(biome.get_name())
    # node = vessel.control.nodes[0]

    from IPython import embed
    embed()
