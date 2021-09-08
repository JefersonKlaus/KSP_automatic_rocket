# KSP_automatic_rocket

Library with basic rocket commands.


## Features

* Landing using MechJeb;
* Suicide burn withou MechJeb (landing anywhere);
* Set retrograde, prograde, stability assist and radial;
* Take off and stop at a given apoapsi;
* Calculate time to start suicide burn (secons);
* Abstract methods to stage control and abort system.


## Example of use

```python
    conn = krpc.connect(name='Command Fenix')
    vessel = conn.space_center.active_vessel

    star_fenix = Fenix4(conn=conn, vessel=vessel)
    star_fenix.landing_with_mech_jeb(
        altitude_airbrake=2
    )
```


Feel free to talk about this project.
jefersonklaus@gmail.com
