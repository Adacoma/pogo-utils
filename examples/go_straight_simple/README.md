`go_straight_simple` runs the same kinematics, heading PID and magnetometer wall
planner as [go_straight](../go_straight/README.md). Its [main.c](main.c) now has
detailed comments explaining the control flow and each configuration step.
The executable logic is unchanged from the original 127-line version; the
source-length limit has been removed to make room for the explanations.

The robot loads `magnetometer_calibration` from PFFS and uses the stored steering
sign, with +1 as the fallback when sign metadata is unavailable. Run the
[calibration example](../magnetometer_calibration/README.md) first. Persistent
motor calibration must also be valid. This example only reads flash.

```sh
make -C examples/go_straight_simple sim
./examples/go_straight_simple/go_straight_simple -c conf/magnetometer.yaml
make -C examples/go_straight_simple bin
```

The second command launches Pogosim. The last builds firmware; SDK paths and
upload instructions are in [getting started](../../docs/getting_started.md#hardware).
The Makefile compiles local control/heading/flash-reader objects for simulation
and reuses go_straight's firmware build rules. It changes no existing example.

Control runs at 20 Hz with half-power forward motion. Library defaults match
go_straight's default PID gains (.60/.10/.04) and wall settings, including the
120-degree minimum-turn policy and protected forward commit. YAML controller
parameters are ignored here; change the configuration in main.c to tune it.
There are no UART diagnostics or simulator CSV callbacks.

Amber means warming or recovering, green means normal heading hold, and blue
means avoidance/forward commit. Violet is reserved for startup failure. Startup
must obtain a full heading window within three seconds. A runtime fault resets
the maneuver and PID, refills the heading window, and retries after at least
500 ms. Missing sensor readings still pass invalid snapshots to kinematics.
A permanently broken sensor or obstruction can prevent safe motion; the code
does not guarantee a physical escape.

See the [motion guide](../../docs/systems/motion_and_avoidance.md) and
[controller tutorial](../../docs/tutorials/controllers.md) for the API contracts.
