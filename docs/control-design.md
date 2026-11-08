# Control design: the camera-to-motor loop

The robot follows a person by one perceptual-motor loop: the OAK's picture, the face found in it,
and the motors that turn the camera. The camera is the only sensor for motion. No gyro: the ZED's
IMU is on the head and too noisy for this (the owner, 2026-11-07). This note sets the design that
replaces the 2026-11-06 head aim and the waist's start/stop drive. It is built and tuned in the
simulator first, then checked on the robot.

Sources: the research notes behind this design, with citations, are kept out of git in
`research_notes/Visual tracking head and torso control/`. The headline references:
- Chaumette and Hutchinson 2006/2007 (visual servo control);
- Skogestad 2003 (SIMC tuning);
- Brown 1990, and Sharkey and Murray 1996 (gaze control with delays);
- Pattacini et al. 2010 and the iCub iKinGazeCtrl source (fast and slow joints);
- Hägglund 2021 (mid-ranging with a sticky actuator);
- Armstrong-Hélouvry et al. 1994 (friction);
- Ljung 1999 (identification);
- Hansen's CMA-ES tutorial.

## Why the old loop is slow, pausing and jumpy

- **Slow.** Each frame aimed at the angle commanded one delay earlier plus 30 % of the image
  error. That anchor ignores the head's own lag (its 1.9 Hz mode adds 50-70 ms). A larger share
  overshoots, so 30 % was forced, then smoothed again and speed-limited.
- **Pauses.** A far target needed 5 sightings in a row. The waist waited for a still head, then
  built its drive up slowly until it broke loose.
- **Waist overshoot.** It stopped by the bearing in a picture 0.15-0.2 s old, then coasted.
- **Jumps.** The head counter-rotated against a dead-reckoned body turn with one speed both ways
  (27 deg/s; measured 50 and 21.5) and no breakaway or coast.

## The loop

1. **Where the person is, in the body's frame.** For each detection at its exposure time t, add
   the face's image angle to where the camera pointed at t. "Where the camera pointed" comes from
   the identified head model driven by the commands sent (an efference copy, a Smith predictor
   in angle space), not from the command alone. Use the waist model for the body's share.
2. **Predict it forward** across the loop's delay. Use an alpha-beta (constant-velocity)
   predictor that is reset at jumps and never fed stale detections.
3. **Head: fast loop, waist: slow loop.**
   - **Head, robust law first** (the synthesis, `reports/`, corrects the first draft here). A
     proportional feedback on the predictor-corrected error, a view-rate gain of about 2.5 per
     second, closes the loop; no integral, derivative or filter sits in it. The zero-vibration
     shaper for the 1.9 Hz mode (second impulse about 0.28 s) goes only on planned paths: the
     move to someone newly believed, and the counter-rotation. A fully shaped direct command
     is about three times faster with an exact model, but overshoots 1.3 degrees with the
     gearing 15 % high and 3.5 at 40 % (pan has measured 0.26-0.36); it waits until
     identification pins the gearing to ±10 % and the mode to ±5 %. A hysteresis settled band
     (about 0.3/0.6 degrees of view) sits around pan's 0.65 degree backlash.
   - **Waist.** Mid-ranging: drive the waist by the head's settled pan offset from centre, not by
     the person's bearing. The thresholds scale with the head's reach R, about 19 degrees of view
     in pan's servo window (not the ±40 the gaze note assumed): start once the settled offset has
     passed about 0.5R (9-11 degrees) for 0.3-0.5 s, at once when the person is beyond R, and
     leave 3-5 degrees for the head. Each move is planned, not a continuous speed loop (about
     60 % of the old overshoot was travel during the camera's delay):
     - a short kick to break loose (30-80 ms near full drive);
     - a cruise from the per-direction inverse model;
     - a stop that starts at the model-predicted stopping distance, braking at low speed.
     There is no integral action on the waist, and small offsets stay with the head.
4. **Counter-rotation from the waist model, not dead reckoning.** The model gives each direction
   its own breakaway, stall, speed and coast. The head turns against the waist's *planned* turn
   as its own commands reach the camera.

## Identification, from vision only

Short scripted sessions with nothing in the loop. Closed-loop logs check the fit; they are not
used to make it.

- **Camera rotation.** Measure each frame against a fixed reference image (rotation-only
  homography K R K^-1), never by chaining frame-to-frame shifts. Mask the person, and correct
  the frame's exposure-time offset.
- **Head axes.**
  - Staircases with reversals give gain and slack.
  - Step ladders in random order, with jittered command times, give dead time and slew.
  - Small steps give the mode.
- **Waist.** Slow drive ramps both ways (breakaway, stall), constant-drive holds (speed map),
  coast-downs and brake stops.
- **Fit.** Output-error fits with SciPy `least_squares` into `simulator/plant.py`. Validate on
  held-out runs (fit %, RMS degrees). Then replay today's recorded following runs through
  `OrientationNode.step()` in the simulator.

## Tuning, in the simulator

- **Constraints first.** Pass/fail verdicts are constraints: no runaway, no oscillation, overshoot
  and reversals under limits.
- **Then a score:** a delay-aware error integral, settle time, smoothness and effort.
- **Scenarios.** Steps, ramps at walking rates, sinusoids, replays of recorded people, and
  perception faults. Keep a frozen held-out set.
- **Robustness.** Randomise gearing, delay, slack, the mode and friction within their measured
  uncertainty: 59 random plants all passing means a failure rate under 5 % with 95 % confidence.
- **Optimiser.** CMA-ES (pycma, restarts), staged: head, then waist, then both.
- **Owner's choice of speed.** Centring 20 degrees within 1 s needs about 80 degrees a second
  squared or more of acceleration. At 25 a 20 degree move takes about 1.8 s. The input shaper is
  what lets a higher acceleration run without ringing; the simulator shows both.

## On the robot

Sessions of 30 minutes or less, with limits and pass/fail set beforehand. When the robot and the
simulator disagree, re-identify the plant rather than hand-tune on the robot.

## Outcome in the simulator (2026-11-07)

Built as above, with these choices where the notes failed in simulation
(`motor/head_aim.py`, `motor/waist_drive.py`, `simulator/tuning.py`):

- **Confirming someone new: 4 of the last 5 frames, not 2 of 3.** 2 of 3 chased the battery's
  false detections, which come 1-3 frames at a time as the robot's did: "ghosts-middle" moved the
  camera 11.3 degrees (0.5 allowed), "edge-partial" 19.1. 3 of 4 still takes 3-frame bursts.
- **The shaper on planned paths only:** the move to someone new and the counter-rotation. The
  feedback is unshaped. Responsive head, worst overshoot (degrees, runs passing of 19):

  | Placement | measured | gearing x1.15 | x1.3 | x1.4 | +15 % gearing and pixels |
  |---|---|---|---|---|---|
  | planned paths only (built) | 0.49 (19) | 1.56 (19) | 2.79 (13) | 3.76 (13) | 2.70 (15) |
  | whole command, inside the modelled loop | 0.54 (19) | 1.88 (19) | 3.33 (13) | 4.34 (13) | 3.26 (15) |
  | none | 1.60 (19, 13 reversals) | 3.19 (13) | 4.73 (12) | 6.07 (11) | 4.44 (12) |

  At the calm head's 25 deg/s^2 all three pass everywhere; shaping the whole command only adds lag.
- **Learning the person's speed** only while they hold their place in the picture, never during a
  planned move or while the waist breaks loose or stops: otherwise the estimate's drift with a
  gearing error (up to 0.4 x the head's speed) is learned as walking and overshoots.
- **Pan is planned in the base's frame** and commanded less the body's turn as the waist *plans*
  it 0.17 s ahead, shaped the same way: a speed lead alone dipped the camera 2.8 degrees back at
  every brake. The waist starts only once the head's planned move has landed (head first).
- **A dropped frame is no loss:** the waist brakes only after 0.35 s without a person (one empty
  frame used to brake a move mid-turn, three moves where one was needed).

**Two tunings** (CMA-ES on the head battery plus 20-degree steps and walkers who stop, on the
measured plant, the corners, two stress plants and fresh random plants each generation):

- *calm*, launched: acceleration 25 deg/s^2, pan gain 1.7/s, a small planned move (reach 0.26),
  settled band 0.3/0.9 degrees; waist starts at 15 degrees held 0.47 s, at once past 21, runs at
  0.53-0.8, closing 0.84/s, no residual left, a 0.5 s lead on walkers, counter-rotation 1.13.
- *responsive* (`head_aim.RESPONSIVE`): the least acceleration that centres a 20-degree step
  within 1 s with the shaper on, 110 deg/s^2, with gain 2.9/s and reach 0.8. It fails the
  robustness gate (below), so it waits until the gearing is measured to within ±10 %.

**Before and after** (measured plant unless stated; settle within 1.5 degrees):

| | before (2026-11-06) | calm (launched) | responsive |
|---|---|---|---|
| head battery, 17 | 17 pass; median 1.48 s, worst 2.28 | 17; 1.11, 1.64 | 17; 0.67, 0.86 |
| head overshoot, reversals | 0.24 degrees, 2 | 0.00, 2 | 0.49, 3 |
| 20-degree step | 2.58 / 2.59 s | 2.26 / 2.23 | 1.00 / 0.97 |
| head held-out, 13 | 13; 1.76, 2.43 | 13; 1.29, 1.73 | 13; 0.69, 0.89 |
| head, 59 random plants (battery, held-out) | 59, 59 | 59, 59 | 53, 52 |
| head, 4 corners | 4 | 4 | 3 |
| full scenarios, 17 | 13 ("far" 15.4 degrees past, body 6.4) | 14 | 15 |
| full held-out, 8 | 3 | 3 | 8 |
| full, worst camera / body overshoot (degrees) | 15.4 / 6.4 | 0.54 / 0.13 | 0.64 / 0.00 |
| full on 59 random plants: runs passing (held-out) | 855 of 1003 (182 of 472) | 869 (270) | 863 (316) |
| full on 59 plants: worst camera / body overshoot | 20.0 / 12.5 | 3.9 / 2.5 | 3.6 / 0.0 |

What still fails, and why:

- *walk-away*: toward positive pan the waist cannot run slower than about 26 deg/s (stall drive
  x 50.4), so it follows a 15 deg/s walker in moves, each turning the head back and out again
  (4 reversals, 2 allowed). Fixing the 2.3:1 speed asymmetry would fix it.
- *far* and the held-out far cases with the calm head, the waist's fast side: the body brings
  someone 27 degrees off within reach in about 0.8 s; a head limited to 25 deg/s^2 needs 1.13 s for
  the 8 degrees beyond its reach, so it lands on them just after the waist stops, a third small
  turn (2 reversals, 1 allowed; the camera never passes them). The responsive head passes these.
- *tilt-noisy*: the roll, unchanged, as before.

The waist's model is in two places, both updatable: the simulated robot, `plant.WAIST`, from
which the random plants' ranges follow (`simulator/robustness.py`); and the controller's own
belief, `WaistDriveParams` (`breakaway`, `stall`, `speed_negative`, `speed_positive`,
`time_constant`, `brake_time`, `coast_time`), panel-tunable.
