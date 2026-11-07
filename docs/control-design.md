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
   - **Head.** Point at the predicted direction less the body's predicted turn. Use one gain
     inside the stability bound for the remaining delay. A zero-vibration input shaper for the
     1.9 Hz mode (second impulse about 0.26-0.28 s) replaces the smoothing and the low speed
     and acceleration limits. A new target is believed after 2 of 3 frames. A hysteresis settled
     band (about 0.3/0.6 degrees of view) sits around pan's 0.65 degree backlash.
   - **Waist.** Mid-ranging: drive the waist by the head's settled pan offset from centre, not by
     the person's bearing. There is a band (about 15-20 degrees of view, held 0.3-0.5 s; at once
     past about 30). Each move is planned:
     - a short kick to break loose (30-80 ms near full drive);
     - a cruise from the per-direction speed model;
     - a stop that starts at the per-direction stopping distance, braking at low speed.
     There is no integral action on the waist, and small offsets stay with the head.
4. **Counter-rotation from the waist model, not dead reckoning.** The model gives each direction
   its own breakaway, stall, speed and coast, and is checked against the image after each move.

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
