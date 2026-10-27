# Current project state

Last updated: 2026-10-28. This is a concise engineering/scientific handoff,
not a hardware-certification report. [Documentation map](index.md);
[historical audit](code_audit.md).

## Inspected

- Library architecture, all 45 installed public headers, all 36 examples,
  build/install rules, relevant source implementations and scenario configs.
- Current PFFS v4, append logs, magnetometer collection/runtime/storage split,
  motion ownership and recovery, ANN layouts, optimizer contracts, SSR and numerics.
- Earlier work included broader static audit and dependency/API inspection.
  Earlier audit milestones did not modify linked repositories.

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
  64 KiB catalog sectors and 1,408 KiB data. Exact byte lengths are packed into
  unchanged 48-byte entries; replacement retains page count but can change EOF.
  Writes/defrag are not transactional; creation can autoformat damaged catalogs.
- Logs use one-page caches and per-page CRC. New defaults are 64 KiB per log;
  existing sizes remain fixed. Catalog length reserves container capacity;
  handles reconstruct/cache committed payload length. RAM bytes disappear on reset.
- Flash shell UART transfers use 48-byte CRC/ACK frames and its existing page
  cache. Uploads default to raw native schemas with exact catalog length;
  `--envelope` opts into type 0x8002 with a payload prefix/content CRC. Downloads
  unwrap envelopes/logs automatically. Creation never autoformats;
  replacement is explicit, same-page-count and non-transactional.
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

- print_log selects `prints`/`csv` by name, resumes existing IDs/allocations,
  and creates missing logs at the first free ID. Ordinary-name collisions and
  a full ID table fail without replacement; explicit clear-on-boot remains
  supported. Missing/corrupt catalogs are recovered before opening handles.
  Startup NOR fixtures for retention and clear modes pass normally and under
  ASan/UBSan. The native example builds; no simulation or firmware run was used.

- The host PFFS uploader now accepts omitted `--id` for new files, trying IDs
  1–80 through existing PFT1 create preflight and reporting the committed ID.
  Explicit IDs retain their behavior; replacement still requires an ID. Only
  `CREATE` rejections advance the search; timeouts and other errors stop it.
  Generic firmware `CREATE` replies cannot distinguish occupied IDs, duplicate
  names and insufficient space. The wire protocol and firmware are unchanged.
  All 25 host transfer tests pass against the real C/NOR engine, including
  full-ID/duplicate-name rejection, READY loss, explicit replacement and CLI
  reporting. Serial/hardware verification of automatic selection remains pending.

- Shared `go_straight`/`vicsek` simulator Makefiles now compile SDK PFFS sources
  instead of requesting the removed local reader/name objects. This also covers
  go_straight_simple, ACU and Vicsek-U-turns. Heading-detection's LED conversion
  macro is guarded when the platform already defines it. The five affected
  motion examples and heading_detection build natively; none was launched.
  An isolated host fixture checks shared/inherited rules and custom build paths.

- PFFS and flash-log core now come from Pogobot SDK `src/pffs`, maintained in
  pogolib and linked by Pogobios and hardware applications. The old public
  headers forward to SDK headers; local duplicate implementations are removed.
  CMake and flash-example simulator builds select the explicit simulator
  backend. Existing v4 files and function signatures are preserved. All 15
  Debug CTests pass against the shared sources; BIOS NOR/console and 19 UART
  transfer tests also pass. The flash shell builds for hardware and the
  simulator using the SDK core. Hardware validation remains pending.

- PFFS v4 preserves 80 IDs, 32-byte names and two-read fast access. Byte-count
  writers pad short pages without overreading; page-count APIs remain wrappers.
  Calibration records now store 168 bytes. Shell touch uses bytes (default zero),
  cat respects EOF, df/stat separate physical allocation from lengths; log
  ls/stat and UART export distinguish reserved capacity from committed payload.
  Host NOR/transfer fixtures cover empty/partial/max files, fixed-page replacement,
  defrag EOF preservation and incompatible-version refusal without writes.
  Generic PFFS/log creation refuses incompatible versions. Calibration now logs
  `MAG_CAL_FLASH_REFORMAT`, explicitly formats on `UNSUPPORTED_FORMAT`, and creates
  its record; this discards all prior entries. No migration decoder is included.
  Regression fixtures cover first/later incompatible catalogs and format failure;
  targeted calibration/PFFS/log tests pass normally and under ASan/UBSan.
  All 15 Debug tests pass normally and under ASan/UBSan/float-cast-overflow;
  19 transfer cases run against the real C engine over host pipes. Shell/reset
  host smoke tests, both platform syntax branches and tutorial C snippets pass.
  No simulator, serial/hardware run or firmware build was used for this change.
- Latest high-priority fixes cover log ownership checks before flash I/O,
  legacy speed/turn recovery and clock wrap, wide Q1.15 dot products, safe float
  conversion endpoints/NaN, and SSR/photostart capacity bounds. All 15 Debug
  tests pass normally and with ASan/UBSan/float-cast-overflow instrumentation.
  See [details and remaining medium findings](code_audit.md#high-priority-fixes-after-follow-up-review).
  No simulation, hardware run, firmware build or external-repository work occurred.
- Raw upload is now the CLI and Python-client default; `--raw` remains accepted
  and `--envelope` enables the stored PFT1 header. Host pipe/NOR tests cover
  one-page 256-byte uploads, padding/empty inputs, replacement, optional headers,
  CLI parsing and download decoding. The original 18 transfer tests passed normally and
  with the ASan/UBSan bridge; no serial device or simulation was used.
- Latest audit fixes cover optimizer ownership/defaults/enums and example
  failure handling; symmetric ANN rounding, signed scaling, MLP shifts/gates,
  ESN configuration bounds, nonfinite HIT/SL adverts, isolated SSR time and
  contracting default weights, unique-origin MNIST voting and trimmed UART CRC.
  See [fix details](code_audit.md#2026-10-26-follow-up).
- Audit validation: all 12 Debug host tests passed normally and with ASan/UBSan.
  Fixtures include integer oracles/endpoints, deep MLP buffers/gates, allocator failure cleanup,
  SSR clock wrap/two-node contraction, vote relays and transfer/NOR failures.
  All six optimizer selections, full/block HIT and MNIST pass strict syntax
  checks. No simulation, hardware/serial run, firmware build or install occurred.
- Earlier validation covered printing, UART persistence, documentation fixtures
  and simulator builds. go_straight_simple retains annotated flash-loaded
  heading, PID/kinematics/walls and a 500 ms post-start retry policy.
- Configuration/build traps documented, not changed: ACU calibration exports
  acu.pgflash while mission imports magnetometer.pgflash; numerical Makefiles
  can silently skip wrong-path builds; distributed MNIST exporter/C basis
  defaults differ. Target size, stack and deadlines remain unmeasured.
- Historical robot-23342 evidence: violet calibration was failure, catalog CRC
  damage was consistent with NOR writes without erase, and v2 sector-based format
  plus calibration succeeded. This is not physical validation of current v4.
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
  Log entry fingerprints reject changed ownership before I/O; callers reopen
  after file movement/destructive operations and keep one writer per log.
- UART transfer uses PFFS v4 exact byte lengths; mission layouts use raw upload
  without an extra header. Optional PFT1 is retained for readers using that schema.
  Host downloads publish only verified data; CRC is not authentication.
- Printing deliberately supports a small syntax without float/double fallback;
  Q conversion quantizes finite inputs, and coarse decimal rounding can carry
  into an endpoint. Logging does not change controller arithmetic or scheduling.
- ACU's fixed genotype uses physical angular/noise units; collective-turn events
  and pairwise yielding remain application policies, not optimizer claims.
- ANN conversion rounds nearest with ties away from zero. SSR now retains
  self weight in Metropolis diffusion and requires updated protocol tags;
  existing thresholds/baselines need recalibration. MNIST counts unique origins
  within an epoch/image, bounded to 64 cached votes, with no finite-time global
  agreement guarantee under incomplete communication.
  Q1.15 dot products now retain wide partial sums; float conversion maps NaN
  to zero and infinities to endpoints. Target arithmetic costs need measurement.
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

1. Revalidate ANN exports/layouts, SSR thresholds and MNIST experiment baselines
   with matching peers after numerical/protocol changes.
2. Address the remaining medium findings listed in the audit, config/Makefile
   traps, and PID/sensor/reference coverage gaps.
3. Validate v4 autoformat, logs/checkpoints and power interruption on disposable
   physical flash; measure erase/program, stack and deadline margins.
4. Test manual heading jumps, sensor/IR outages and jams on representative robots;
   include go_straight_simple and log recovery/displacement separately.
5. Pin dependencies/toolchains and record firmware/map/RAM/stack measurements.
6. Define scientific objectives, datasets, seeds, metrics and acceptance criteria
   before comparing flocking, SSR or learned controllers.
7. Clarify release/API support, licensing and install/package interface.
