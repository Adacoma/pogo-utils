# Vicsek with collective turn events

This example keeps `examples/vicsek`'s alignment, PID, flash-loaded heading,
wall avoidance, and motor behavior. Its one intentional difference is the
ACU-style collective turn event: when a robot first enters wall avoidance, it
advertises its current heading plus `u_turn_angle_rad` to neighbors. The default
angle is `0.4*pi` (72 degrees), so “U-turn” is a protocol name, not a literal
180-degree turn. A receiving robot temporarily follows that target; its own
wall avoidance always has priority. Events have a bounded lifetime and hop
count, and duplicate messages cannot extend them.

The new example uses a distinct `VU` message tag. It does not exchange
controller messages with `vicsek` or `acu` robots. Unlike the optional hint
in the original Vicsek example, this event starts on *entry* to wall avoidance
and does not wait for the local robot's settled escape heading.

To break a two-robot head-on symmetry, the higher-ID robot yields after 350 ms
and at least two fresh packets indicating nearly opposite headings. It first
backs away at 65% calibrated power for 0.6 s, then requests a pivot toward the
lower-ID robot's advertised target for up to 1.8 s. It resumes forward tracking
even if it has not finished the pivot, and releases the shared target once its
heading is close. The default reverse power exceeds this example's default
forward power so that the other robot cannot simply close the gap at its
nominal commanded speed. The lower-ID robot keeps its normal Vicsek behavior.
The selected target is held across brief IR loss during back-off; if the peer
remains absent, the maneuver ends after its bounded reverse-and-pivot window.
Wall avoidance and collective turn events override this pairwise maneuver.
This uses heading and IR reception, not measured position:
it cannot guarantee that a physically blocked robot moves within three seconds.
For simulator diagnosis, the data export includes `opposition_active`,
`opposition_reverse`, `opposition_pivot`, `opposition_target_rad`, and
`opposition_event_count`.

Use a flash image containing calibration for every simulated robot, as with
the original Vicsek example. From the repository root:

```sh
make -C examples/vicsek_u_turns sim
./examples/vicsek_u_turns/vicsek_u_turns -c conf/magnetometer.yaml
```

For hardware, `make -C examples/vicsek_u_turns bin` builds the same event logic
by default. The new parameters can be set under the YAML `parameters` key in
simulation: `enable_cluster_u_turn`, `u_turn_angle_rad`, and
`cluster_u_turn_duration_ms` (default 1500). All ordinary Vicsek parameters
retain their original defaults. Changing turn events cannot guarantee straight
trajectories while robots are aligning or physically interacting.
