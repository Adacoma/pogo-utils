# Static ACU controller

This example ports the ACU motility law from the read-only
`libs/ACU-selfadapt` reference while using pogo-utils' current Vicsek
integration for sensing and actuation. The five controller parameters are
fixed at startup: alignment gain, angular-noise intensity, base speed,
collective U-turn phase, and crowding depth. There is no optimizer, objective
evaluation, genotype exchange, HIT/FT state, or dynamic allocator.

The compiled defaults and `conf/acu.yaml` reproduce the reference fixed
"flock + wall following" genotype: beta 9 rad/s, sigma 0 rad/sqrt(s), speed
0.8, U-turn phase 0.4 pi, and crowding depth 0. Other static parameter sets can
be tested by changing the YAML values without changing the binary.

Heading is loaded from the versioned magnetometer calibration record in flash.
For Pogosim, build both examples, export calibration first, then import it into
the ACU mission:

```console
./examples/magnetometer_calibration/magnetometer_calibration \
  -c conf/acu_calibration.yaml
./examples/acu/acu -c conf/acu.yaml
```

The two configurations must retain matching object categories and robot IDs.
The mission does not contain calibration collection or fitting code. Violet is
therefore reserved for an unrecoverable startup/configuration failure; runtime
avoidance faults use the same reset-and-reacquire path as the current Vicsek
example.

The `AC` wire format is deliberately separate from Vicsek's `VK` protocol.
Packets contain explicitly serialized headings and an optional bounded,
relative-lifetime collective U-turn event; they never contain raw C structs or
another robot's absolute timestamp.
