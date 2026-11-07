# Tuning list

Things to tune once the robot runs end to end. Each says what was seen and where the
evidence is. Tune on the simulator fitted to the robot first
([simulator guide](simulator.md)), then check on the robot. Add to it as things come up;
strike items out with the commit that settles them.

## Following

1. **Waist and head together.** The waist went back on untuned on 2026-11-07, and the owner
   saw it overshoot and lag. In the simulator with the waist on, the full scenarios fail at
   every deadband: "far" swings 14-20 degrees past the person, and "left" and "right" end
   2-5 degrees off ([audit](audit-2026.md#slower-pan-tighter-centring-2026-11-07)).
2. **When the waist may start.** A move starts only once the head has held still for
   `head_settle` (0.3 s). The tighter deadband (0.12) keeps the head making small
   corrections, so the waist starts later or not at all. In `tests/test_bot_recovery.py`,
   any deadband under 0.39 kept it from ever starting. Count small corrections as still, or
   tie `head_settle` to the deadband.
3. **Pan speed against lag.** A 30 degree a second cap on pan's servo was "way too slow" and
   came off the same day. The robot runs at full speed with a gentler acceleration, 25 degrees
   a second squared instead of 34. In the simulator the cap hardly changed settling (median
   1.56 s against 1.53). What makes the head slow is `reach` and `smoothing`, not top speed.
   Re-tune them with the waist on.
4. **Centring deadband.** 0.12 is checked only in the head battery. Watch the robot for
   hunting, small back-and-forth moves, from detection noise and pan's 2.5 degrees of slack.
5. **The head's constant wobble.** The IMU shows about ±0.5 degrees at about 2 Hz,
   unexplained.

## Tilt

6. **Tilt does not reach a face below.** Under servo power the head stops nodding down at
   about servo 100: tilt stepped from 100 to 170 moved it under 0.3 degrees (IMU, run
   20261107-115052). The owner later saw the head go lower at 180 than at 140, so since
   2026-11-07 following tilts down to the servo's own end, 180. With a face low in the
   picture it holds there and strains the servo (item 8). Fix the mechanism first; then set
   `tilt_range` to the travel the head really has, and re-measure `tilt_ratio` (0.08 so far).
7. **Tilt's sweep speed in tests.** Tilt tests sweep at `servo_speed` (150 servo degrees a
   second). Slow it like pan if the head jerks.

## Power and heat (before any long tuning run)

8. **Servo power.** Three power cuts (2026-11-06 23:47, 2026-11-07 11:02 and 13:38) came
   while tilt was held or driven at its end. Give the servo board its own supply.
9. **ZED temperature.** Its IMU read 37 to 70 degrees C in nine minutes, and 88-99 C over a
   long run. Ventilate the head, or slow the ZED.

## Dashboard

10. **Last error.** The control panel shows the newest error in about three hours of the
    bot's log, from earlier runs too.
11. **Pan arrows.** ◀ and ▶ are the robot's own left and right, as its camera's picture
    and the 3D view from behind show them; facing the robot they are mirrored. Decide which
    the owner prefers.

## Roll

12. **Face levelling has almost no reach.** Levelling went back on on 2026-11-07. The ZED's
    accelerometer measured roll servo 60 to 120 tipping the head 4.0 degrees: 0.067 per servo
    degree, as `RollLevelParams.ratio` (0.065) has it. Its ±25 `span` therefore straightens
    a tilted face only about 1.5 degrees. At servo 90 the head read 4.5-5.8 degrees,
    depending on the side it came from: about 1.3 degrees of play, some 19 servo degrees
    (`slack` is 10). It also leans 5-6 degrees by gravity at 90, which does not matter for
    levelling in the picture. A linkage giving 0.5 degrees or more per servo degree would
    make levelling useful. Until then, measure roll's travel past ±30 (power cuts: item 8)
    before raising `span` (at most 30 now).

## Plan (2026-11-07)

**What has been tuned, and what not.**

- **Tuned, 2026-11-06:** the head's aim, in the simulator fitted to the robot, with the waist
  and roll off (`simulator/tuning.py`). The values: `reach` 0.30, `smoothing` 0.69, 15
  degrees a second, 34 degrees a second squared, deadband 0.39, servo lag 55 ms, jumps
  believed after 5 sightings. Roll levelling was fitted separately the same day.
- **Changed since, not tuned:** acceleration 25 instead of 34 (after a short-lived pan speed
  cap); deadband 0.12; full servo ranges, tilt to 180; waist on; roll on. Each was only
  re-checked against the head battery.
- **Never tuned:** the waist together with the head. The waist drive itself has not been
  re-tuned since its motor was measured.

**Why it feels slow, pausing and jumpy** (from the code and the fitted plant):

1. **Slow to converge.** Each frame aims at only `reach` (30 %) of the face's offset, and
   `smoothing` filters that again. In the simulator this, not the top speed, sets the settle
   time.
2. **Gentle starts.** Acceleration is 25 degrees a second squared.
3. **Pauses.** A target more than 8 degrees from the goal must show up 5 frames in a row
   before the head moves.
4. **Waist pauses.** The waist starts only once the head has held still for 0.3 s. Its
   drive starts at 0.15, under the 0.65 where it breaks loose, and rises 1.0 a second:
   about half a second with nothing moving.
5. **Jumps.** The body's turn is not measured. `body_turn` integrates the drive at one
   speed, `degrees_per_second` (27 at full drive). The waist really turns 50 degrees a
   second one way and 21.5 the other, and not at all until it breaks loose. The head
   counter-rotates by that estimate: it turns off the face before the body moves, or by
   the wrong amount, and the camera then pulls it back.
6. **Waist overshoot, and one speed.** The waist runs at 0.5-0.6, between its stall (0.45)
   and breakaway (0.65), so once moving it has one speed. It decides to stop at 5 degrees
   of bearing. After that come about 0.2 s of camera delay, a 0.2 s ramp-down and a 0.3 s
   coast, so it travels 10-20 degrees further.

**Steps.** Short robot sessions; everything else in the simulator.

1. **Measure today's robot:**
   - Step responses of pan, tilt (to 180) and roll.
   - The waist both ways at several drives: breakaway, stall, speed, coast, measured
     from the image.
   - Update `simulator/plant.py`. Check fidelity (`simulator/fidelity.py`) against 3-5
     recorded following runs.
2. **Measure the delay chain** from a run: frame captured, pose, command, IMU motion.
3. **Write the targets** into the scenario verdicts and `tuning.py`'s cost:
   - A 20 degree step centred within 1 s by the head alone; 45 degrees within 3 s with the
     waist.
   - Overshoot under 1 degree for the head, under 3 for the body.
   - No reversals at rest.
   - No pause over 0.3 s once a person is believed.
   - Robust to gearing ±15 %, slack, and 50 ms more delay.
4. **Measure the body's turn with the camera, not a gyro** (the owner: the ZED's gyro is on
   the head and too noisy). Use the image's global shift (phase correlation, as
   `scripts/check_servo_travel.py` does) less the head's commanded pan, and a waist model
   identified from that. The head then counter-rotates by what the body does.
5. **Rebuild the waist as a graded speed loop:**
   - A wanted body speed proportional to the head's pan offset from centre: the head keeps
     the face centred, and the waist brings the head back to centre.
   - A loop on the body's speed as the image shows it, with a breakaway kick and a floor
     above stall.
   - A gain per direction (50 against 21.5).
   - A stop that begins early enough for its measured stopping distance.
6. **Tune the head**, waist off and then on: speed, acceleration, reach, smoothing,
   deadband, jump gating, servo lag. Aim for fast and smooth: a higher top speed with
   gentle acceleration, not a low top speed.
7. **Check on the robot** after each step: the same scenarios acted out, compared with the
   simulator.

**First:** item 8, the servo supply, so long runs do not cut the Pi's power. Item 6, the
tilt mechanism, so tilt's fit holds.
