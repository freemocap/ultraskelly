# Working on UltraSkelly

Read [the 2026 audit](docs/audit-2026.md) before changing the launch path,
controllers, camera integration, dependencies, or cleanup scope. It records the
working-path evidence, unresolved hardware behavior, and staged revival plan.
For simulator changes, read [the simulator guide](docs/simulator.md) for shared
step methods, clock injection, model assumptions, launch commands, and verification.

- The development host can be Windows without a robot. Keep imports and the default
  `python -m ultraskelly` invocation free of hardware initialization. Hardware
  construction belongs in explicit factories; the replay harness injects fakes.
- Run `uv sync --locked` and `uv run --locked python -m pytest -q` for the portable
  suite. See README for replay commands. Run hardware code only on an explicitly
  selected Pi with the robot ready for motion. Several scratch scripts move motors
  immediately when imported.
- Pub/sub currently carries replaceable state: frames, poses, targets, servo state.
  Keep queues bounded and publication independent of consumer speed. Discrete
  commands require a separate delivery contract; they must not silently inherit
  the latest-value policy. Queue operations belong on the asyncio event loop.
- Preserve channel assignments, signs, and travel limits until a tested change
  justifies replacing them. The 120-second restart workaround was removed on
  2026-11-02 at the owner's direction; the historical freeze is not yet reproduced
  or fixed. Bench tests do not establish physical limits or prove the freeze fixed.
- Develop simulation against the same Python controller logic used on the Pi.
  Label commanded positions separately from measured or simulated positions.
  The current DC motor throttle is not a measured waist or jaw angle.
- Keep the viewer local and account-free. Foxglove, its SDK, and MCAP were explicitly
  rejected. At the owner's direction (2026-11-02) state and video travel on FreeMoCap's
  self-describing CBOR stream: `/stream`, encoded with `freemocap-stream` and decoded by
  the vendored `@freemocap/client` (both from FreeMoCap's `sfn26-ultraskelly` branch).
  Rebuild the bundle with `scripts/update-freemocap-client.sh` when that pin moves.
- Keep this file short. Put findings, acceptance criteria, and research links in
  the audit; update it when evidence changes. Retain hardware experiments until
  their wiring/calibration value has been assessed.
