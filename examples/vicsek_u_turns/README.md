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
