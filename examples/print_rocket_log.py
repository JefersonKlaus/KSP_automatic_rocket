import krpc

from rockets.rocket_log import RocketLog

if __name__ == "__main__":
    print("... conectando ...")
    conn = krpc.connect(name="Command StarHopper")
    print("!!! conectado !!!")

    vessel = conn.space_center.active_vessel

    rocket_log = RocketLog(conn=conn, vessel=vessel)
    rocket_log.keep_printing_rocket_status()

    # ##################### COMO USAR DISTOMETER
    # x = vessel.parts.with_name("distometer100x")
    # [y.name for y in x[0].modules]
    # lasers = vessel.parts.modules_with_name("LaserDistModule")
    # lasers[0].fields

    # from IPython import embed

    # embed()

    # for part in vessel.parts.with_name("distometer100x"):
    #     part.engine.active = True
    # ############################
