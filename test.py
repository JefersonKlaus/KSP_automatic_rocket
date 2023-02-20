import time

import krpc

from tools.biome import Biome

if __name__ == "__main__":
    print("... conectando ...")
    conn = krpc.connect(name="Test")
    print("!!! conectado !!!")
    vessel = conn.space_center.active_vessel

    biome = Biome(vessel=vessel, conn=conn)
    print(biome.get_name())

    from IPython import embed

    embed()
