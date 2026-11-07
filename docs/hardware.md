# UltraSkelly hardware reconstruction

Draft 01 — 2026-10-29. This reconstructs the expected active wiring from the repository, your identification of a Raspberry Pi 5, and manufacturer documentation. It is not an inspection of the assembled robot. The diagram uses symbolic connector locations and schematic board shapes, not measured CAD footprints.

[Open the companion diagram](../ultraskelly/simulator/static/hardware.html) · [Scalable SVG](../ultraskelly/simulator/static/hardware.svg)

With the simulator running, select **Hardware map** in its header or open `http://127.0.0.1:8765/hardware.html`.

## Observed on the Pi, 2026-11-02

Observed through the re-imaged Pi and the owner's description, not by inspecting the assembled robot. This supersedes the camera rows and checks below, which describe the original IMX500 build. The companion diagram still shows the original ribbon camera.

- No camera on the Pi's CSI connectors; the kernel probes no sensor at boot.
- Luxonis OAK-1 (one IMX378 colour sensor) and Stereolabs ZED Mini (serial 12395566), each on a blue USB 3 port and streaming at USB 3 speed. The ZED Mini's video enumerates only with its USB-C plug in one orientation; if only its `2b03:f681` HID interface appears, flip the plug at the camera end.
- Hailo-8 AI HAT+ on PCIe, linked at Gen 3 with `dtparam=pciex1_gen=3`.
- I²C bus 1 answers at `0x40` (servo board), `0x60` (motor board) and `0x70` (PCA9685 all-call).
- The Pi reports a 5 A supply; `usb_max_current_enable=1`.

## Parts and evidence

| Part | Best-supported identification | Confidence and evidence |
| --- | --- | --- |
| Host | Raspberry Pi 5 | You identified the model. Python runs the camera and actuator controllers. |
| Camera | Raspberry Pi AI Camera / Sony IMX500 family (original build; not fitted as of 2026-11-02) | Its pose detector constructed IMX500 and Picamera2; that code was removed on 2026-11-07 and remains in git history at `d4fe10b`. Exact module branding and cable routing are unverified. |
| Servo controller | 16-channel PCA9685 board, possibly Adafruit Servo HAT, bonnet, or breakout | `ServoKit(channels=16)` uses default address `0x40` and 50 Hz. Library supports multiple physical boards; source cannot identify the SKU. |
| DC driver | Likely Adafruit DC & Stepper Motor HAT or compatible MotorKit board | Both active motor factories select `0x60`, 1600 Hz. Scratch code explicitly calls it a DC+Stepper HAT. HAT versus bonnet and revision remain unverified. |
| Head actuators | Three positional hobby servos | `servo[3].angle` controls pan, `servo[11].angle` tilt, `servo[7].angle` roll (held level at 90). Found 2026-11-06 by wiggling each channel with the user watching. Model, torque and rated supply voltage are absent. |
| Waist actuator | DC rotary motor, plausibly geared and encoder-equipped | Active `motor1.throttle`; historical encoder experiments use the same output. Gearbox and encoder identity are unknown. |
| Jaw actuator | Two-terminal load driven as a DC motor, mechanically spring-return | Active `motor2.throttle`; your description establishes powered-open and spring-closed behavior. Exact motor/solenoid mechanism is not established by the API. |
| Power | Pi supply plus actuator power rail(s) | Required functional paths, not a known inventory. The number of adapters, converters, jumpers, voltage and current ratings cannot be recovered from software. |

There is probably **one DC driver board, not two**: OrientationNode and MouthNode each create a software MotorKit object addressing the same `0x60` hardware. That also creates a shared-board ownership issue to review before Pi bring-up; two Python objects are not evidence of two physical HATs.

## Connection map

Channel numbers below are board channel labels, not Raspberry Pi GPIO numbers. Servo channels are zero-based: use the output marked **11**, not the eleventh connector counted from one.

| From | To | Conductors / interpretation |
| --- | --- | --- |
| Pi CAM/DISP port | AI Camera CSI socket | Original build; not fitted as of 2026-11-02. Appropriate Pi 5 22-pin to camera 15-pin ribbon. Port 0 versus 1 and contact orientation are not recorded. Follow connector markings and the camera installation guide. |
| Pi GPIO2 / physical pin 3 | Servo board SDA and motor board SDA | Shared I²C data net. A stacked HAT carries this through its header rather than needing an extra jumper. |
| Pi GPIO3 / physical pin 5 | Servo board SCL and motor board SCL | Shared I²C clock net. Both boards coexist at different addresses. |
| Pi ground, e.g. physical pin 6 | Both driver logic grounds | Common signal reference. Stacked headers can provide it. Exact return harness is not documented. |
| Servo board CH3 | Head pan servo | Signal, servo V+, ground. Match board silkscreen and servo connector pinout; do not infer physical row order from the diagram. |
| Servo board CH11 | Head tilt servo | Signal, servo V+, ground. Same connector checks as pan. |
| Servo board CH7 | Head roll servo | Signal, servo V+, ground. Held level; no controller turns it yet. |
| Motor driver M1 terminal pair | Waist motor's two motor leads | H-bridge outputs; neither lead is a permanent ground. Encoder leads, if present, are separate. Physical polarity is unrecorded. |
| Motor driver M2 terminal pair | Jaw actuator's two drive leads | Negative throttle is the software opening direction; zero releases power. Wire colors and terminal polarity must be identified on the mechanism. |
| Pi supply | Pi USB-C power input | Expected Pi 5 supply path; actual adapter unknown. |
| Servo-rated supply | Servo board V+ and its power return | Servo power is distinct from PCA9685 logic VCC. Voltage must match the actual servos and board. |
| Motor-rated supply | Motor board motor-power input and return | Match the actual driver and both loads. Do not assume waist and jaw can share an arbitrary voltage just because they share a board. |

For a separate PCA9685 breakout, the usual Pi connection also supplies **3.3 V logic VCC** from physical pin 1, with SDA, SCL and GND. This is conditional on identifying that breakout and its pinout. A HAT supplies its logic through the stacking header. Do not substitute servo V+ for logic VCC, or add supply jumpers to a stacked board based on the symbolic drawing.

The diagram's colored lines identify signal functions, not observed wire insulation colors. Dashed power paths represent unresolved wiring, not instructions to connect unknown supplies together. External actuator returns reference the driver grounds; do not join unknown positive rails. Board layout, stacking order, mounting spacers and Pi cooler clearance are not recoverable from the code.

## Encoder clue and inactive hardware

`ultraskelly/scratch/rotory_motor_test.py` and `rotary_motor_manual_drive.py` record:

- Encoder A: **green → GPIO16 (physical pin 36)**.
- Encoder B: **white → GPIO12 (physical pin 32)**.

These are historical comments, not confirmation of current wiring. Encoder supply, return, output type and logic level are absent. Do not apply an unverified encoder output to Pi GPIO; its signal must be compatible with 3.3 V logic. The active OrientationNode does not consume encoder data. The simulator's waist angle is an estimate, not an encoder measurement.

Old tracking scripts disagree on servo channels, and some include roll. The active launcher constructs OrientationNode and MouthNode, whose current defaults are pan 3, tilt 11, roll 7, waist M1, jaw M2. Roll is held level at 90; no controller turns it. The Hailo-8 is used by the default `HAILO_POSE` launch path; the ESP32 experiment is used by no launch path. Both are omitted from the primary diagram rather than assumed to be physically absent.

## Reassembly checks supported by the evidence

1. With power disconnected, compare board labels and connector markings against this map. Identify motor and servo supply ratings before reconnecting power. Keep unresolved power and encoder connections unconnected until identified.
2. Confirm the two servo plugs are on outputs 11 and 3, and the two actuator pairs are on M1 and M2. Inspect signal/power/ground orientation on the actual servo board.
3. Original IMX500 build only: confirm the camera ribbon fits the Pi 5 and AI Camera connectors. Its code (removed 2026-11-07, in git history at `d4fe10b`) discovered `imx500.camera_num`; it did not encode which CAM/DISP socket you used. The current cameras are USB; see the observations above.
4. After the physical wiring and supplies are established, check the expected control addresses `0x40` and `0x60`. Address presence alone does not establish correct motor wiring or supply voltage; board all-call addresses can also appear.
5. Plan the first powered actuator check with clearance for motion: the existing hardware factories immediately command head servos to 90 degrees and drive the jaw at +1 for 0.5 seconds, then -1 for 0.5 seconds. Since 2026-11-02 the launcher no longer recreates nodes periodically, so this happens once per launch. Do not launch it merely to identify connected boards.

The code's 0–180 degree servo range and simulated ±100 degree waist stops are not verified mechanical travel limits. Negative waist drive increasing simulated yaw is a model convention; physical direction must be checked against the assembled motor leads and linkage.

## What remains to confirm after this draft

The map is sufficient to locate the four active actuator outputs and the main data connections. A final wire-by-wire assembly drawing needs only the missing physical evidence:

- Servo board and motor board model/revision, and whether they stack or use jumper leads.
- Supply/converter labels, voltage/current ratings, and power-jumper positions.
- Servo and waist/jaw actuator models, actual lead polarity and plug orientation.
- Whether the historical waist encoder is still fitted, with its supply and logic levels.
- Camera port and ribbon orientation, plus any physically present but inactive modules.

## Sources

Repository evidence:

- [Active launcher](../ultraskelly/bot_launcher.py): `LaunchConfig`, `_create_nodes`.
- [Orientation controller](../ultraskelly/core/bot/motor/orientation_node.py): parameter defaults and `create`.
- [Jaw controller](../ultraskelly/core/bot/motor/mouth_node.py): channel 2, address and startup drive.
- IMX500 pose detector, `ultraskelly/core/bot/sensory/pose_detection_node.py` (removed 2026-11-07; in git history at `d4fe10b`): IMX500 model and camera discovery.
- [Historical waist/encoder experiment](../ultraskelly/scratch/rotory_motor_test.py): HAT description and GPIO assignments. Read only; do not import it for inspection.

Manufacturer references checked 2026-10-29:

- [Adafruit ServoKit API](https://docs.circuitpython.org/projects/servokit/en/latest/api.html): default address 64 (`0x40`), default 50 Hz, supported board families.
- [Adafruit DC & Stepper Motor HAT](https://learn.adafruit.com/adafruit-dc-and-stepper-motor-hat-for-raspberry-pi?view=all): I²C motor board family and hardware guidance.
- [PCA9685 connections and servo power](https://learn.adafruit.com/16-channel-pwm-servo-driver/hooking-it-up): distinct VCC/V+ rails and three-wire servo connection. Its Arduino examples are not Pi GPIO wiring instructions.
- [Raspberry Pi AI Camera](https://www.raspberrypi.com/documentation/accessories/ai-camera.html): IMX500 identity and integration.
- [Raspberry Pi camera hardware](https://www.raspberrypi.com/documentation/accessories/camera.html): camera connectors and installation.
- [Raspberry Pi hardware documentation](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html): GPIO and board interfaces.
