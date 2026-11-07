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
3. **Pan speed against lag.** Pan's servo is held to 30 degrees a second, about 8 of view.
   The owner found pan "a bit delayed" before that, and the worst simulated settle grew from
   2.05 s to 2.45 s. Re-tune `max_speed`, `max_acceleration`, `reach` and `smoothing` with
   the waist on.
4. **Centring deadband.** 0.12 is checked only in the head battery. Watch the robot for
   hunting, small back-and-forth moves, from detection noise and pan's 2.5 degrees of slack.
5. **The head's constant wobble.** The IMU shows about ±0.5 degrees at about 2 Hz,
   unexplained.

## Tilt

6. **Tilt does not reach a face below.** Under servo power the head stops nodding down at
   about servo 100: tilt stepped from 100 to 170 moved it under 0.3 degrees (IMU, run
   20261107-115052). Tracking still drives tilt to 135, its window's end, and strains the
   servo. Fix the mechanism first; then set `tilt_range` to the travel the head really has,
   and re-measure `tilt_ratio` (0.08 so far).
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
