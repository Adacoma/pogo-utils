# Current project state

Last updated: 2026-10-01. This is a concise engineering/scientific handoff,
not a hardware-certification report. [Documentation map](index.md);
[historical audit](code_audit.md).

## Inspected

- Library architecture, all 44 installed public headers, all 35 examples,
  build/install rules, relevant source implementations and scenario configs.
- Current PFFS v3, append logs, magnetometer collection/runtime/storage split,
  motion ownership and recovery, ANN layouts, optimizer contracts, SSR and numerics.
- Earlier work included broader static audit and dependency/API inspection.
  External repositories were not modified for this documentation milestone.

## Understood

- Embedded research toolbox for Pogobot/Pogosim, version 0.1.0. Applications own
  lifecycle, acquisition, experiment policy and communication; library objects
  are generally caller-owned/per robot.
- Timestamp/reference-aware motion separates sensor acquisition, circular PID,
  heading-aware wall planning and one kinematic motor owner. Legacy direct
  avoidance remains separate.
- Calibration firmware collects/fits/stores; missions load the named record
  and warm a live window. No calibration ID is reserved or old format supported.
- PFFS uses 5,888 physical pages, 80 IDs and contiguous sector-owned extents:
  64 KiB catalog sectors and 1,408 KiB data. Replacement cannot resize.
  Writes/defrag are not transactional; creation can autoformat damaged catalogs.
- Logs use one-page caches and per-page CRC. New defaults are 64 KiB per log;
  existing sizes remain fixed. Uncommitted RAM bytes disappear on reset.
- Printing uses a standalone integer/fixed-point core and flash-first adapter:
  RAM-only producer, caller-sized staging, one pending spill/mirror record,
  user-called one-page service/checkpoint, optional byte-budgeted transport.
  printf_fixp keeps its terminal API but no longer delegates to libc printf.
- Quantized inference does not make setup/training/calibration/optimization/SSR
  floating-point-free. Standalone optimizers and the allocating facade differ
  in memory and evaluation contracts.
- Prior fixes cover dynamic-MLP buffer alternation, Q16.16 add/sub/abs saturation,
  allocator overflow/boundary/order/double-free checks and bounded SEP-CMA-ES.
  These do not establish correctness of every remaining numerical path.
- Rich motion examples reset/reacquire after post-start faults; startup failures
  may stop permanently. Vicsek-U-turns adds reverse/pivot yielding, not measured
  collision detection or guaranteed jam escape. ACU remains static, not optimized.

## Currently working analyses

- Latest printing validation: fresh Debug library build and all six host tests
  passed; two new tests also passed ASan/UBSan with warnings-as-errors. Coverage
  includes exhaustive 16-bit Q values at precisions 0..6, 32-bit bounds/random
  integer oracle checks, RAM-only print bursts, queue/terminal backpressure,
  file-full checks, one-page service limits, readback and torn-write failure.
- New print_log simulator target built (not run), and terminal-on/endless and
  default modes passed strict syntax checks. Formatter/adapter object symbols
  have no printf/double-math dependency. Target timing/size remain unmeasured.
- Earlier documentation validation checked 60 Markdown files/local anchors,
  seven tutorial C blocks and small ANN/optimizer/fixed-point host fixtures.
  All 34 earlier simulator targets rebuilt with installed dependencies; numerical
  targets needed explicit local-source path. Existing heading/fixed-point
  benchmark warnings remain. **No simulation/hardware program, training, physical
  flash format or installation was run in the printing milestone.** Firmware was not built.
- Configuration/build traps documented, not changed: ACU calibration exports
  acu.pgflash while mission imports magnetometer.pgflash; numerical Makefiles
  can silently skip wrong-path builds; optim example enables only five entries
  from a six-entry class table; distributed MNIST exporter/C basis defaults differ.
- Historical robot-23342 evidence: violet calibration was failure, catalog CRC
  damage was consistent with NOR writes without erase, and v2 sector-based format
  plus calibration succeeded. This is not physical validation of current v3.
  Earlier simulator runs do not imply new runs at this milestone.

## Current scientific decisions

- Heading consumers carry original timestamp, validity and frame identity;
  sample outages do not become fresh readings or trigger mission refitting.
- Local state/communication and explicit ask–tell/reward windows retain application
  control over scheduling and objective evaluation.
- Flash records are portable by policy, not bound to robot/motor identity.
  App chirality/offset/filtering are not persisted; compatibility remains a duty.
- PFFS favors bounded firmware/RAM and fast ID reads over resizing/transactions.
  Logs separately trade cached speed and page density against checkpoint loss.
- Printing deliberately supports a small syntax without float/double fallback;
  Q conversion quantizes finite inputs, and coarse decimal rounding can carry
  into an endpoint. Logging does not change controller arithmetic or scheduling.
- ACU's fixed genotype uses physical angular/noise units; collective-turn events
  and pairwise yielding remain application policies, not optimizer claims.
- Mathematical explanations describe implementations; no new accuracy, stability,
  classifier, learning-performance or worst-case timing claim was established.

## What remains unknown

Physical revision compatibility, firmware size/RAM/stack/deadline margins,
flash wear/power-failure behavior, calibration robustness, multi-robot recovery
and collective/learning performance across conditions. Clean-checkout dependency
pinning, public API stability/deprecation policy, packaging and licensing need
clarification. Host builds do not answer these questions.

## Known data limitations

No standardized experimental dataset, golden trajectory, calibration corpus or
acceptance tolerances. YAML scenarios are not evidence of outcomes. Generated
ANN assets lack standardized provenance; fixed-point plot data are hard-coded.
Example assertions/benchmarks are not a complete repeatable regression suite.

## Next concrete tasks

1. Address documented config/Makefile/facade traps in a separately authorized code change.
2. Add platform-neutral regression tests for PID/reference/time wrap, MLP buffers,
   fixed-point boundaries, allocator and optimizer invariants.
3. Validate v3 autoformat, logs/checkpoints and power interruption on disposable
   physical flash; measure erase/program, stack and deadline margins.
4. Test manual heading jumps, sensor/IR outages and jams on representative robots;
   log recovery and actual displacement separately.
5. Pin dependencies/toolchains and record firmware/map/RAM/stack measurements.
6. Define scientific objectives, datasets, seeds, metrics and acceptance criteria
   before comparing flocking, SSR or learned controllers.
7. Clarify release/API support, licensing and install/package interface.
