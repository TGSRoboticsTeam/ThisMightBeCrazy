package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;
@Disabled
@TeleOp(name = "disApolloDrive", group = "Swerve")
public class disApolloDrive extends LinearOpMode {

    // ============================================================
    //   PER-OPMODE CONFIG  (override these in a subclass)
    //   The turret is now MANUAL-only; the only thing variants need to
    //   change is which AprilTag they read for the distance-based flywheel
    //   adjustment and the light indicator.
    // ============================================================
    protected int getTargetTagId() { return 20; }

    // --- 1. HARDWARE DECLARATIONS ---
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private CRServo frontLeftSteer, frontRightSteer, backLeftSteer, backRightSteer;
    private AnalogInput frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder;
    private IMU imu;
    private VoltageSensor voltageSensor;

    // --- MECHANISM MOTORS ---
    // Intake motors are DcMotorEx so we can read per-motor current for stall detection.
    private DcMotorEx topIntake, bottomIntake;
    private DcMotor leftFly, rightFly;

    // --- BLOCKER SERVO ---
    private Servo blocker;

    // --- LIGHT SERVO (AprilTag status indicator) ---
    private Servo light;

    // --- PTO SERVO ---
    private Servo pto;
    private boolean ptoEngaged = false;
    private boolean ptoDpadUpPreviouslyPressed   = false;
    private boolean ptoDpadDownPreviouslyPressed = false;

    // --- TURRET SERVOS (MANUAL ONLY) ---
    private Servo leftTurret, rightTurret;
    private double turretPosition = 0.5;
    final double TURRET_DEADBAND = 0.05;

    // --- TURRET MANUAL MODE (G2 left stick X) ---
    // Three speeds: LT held = fast, RT held = slow, neither = normal.
    // If both triggers are held, LT (fast) wins.
    final double TURRET_SPEED_MANUAL = 0.016; // normal manual speed
    final double TURRET_SPEED_SLOW   = 0.008; // RT held — fine control
    final double TURRET_SPEED_FAST   = 0.032; // LT held — fast slew
    final double TURRET_RT_THRESHOLD = 0.5;
    final double TURRET_LT_THRESHOLD = 0.5;

    // --- TURRET ANGLE CALIBRATION (telemetry readout only) ---
    // turretPosition is a raw servo value. The turret-relative camera angle is:
    //     turretRelAngle = (turretPosition - 0.5) * 2*PI + TURRET_ANGLE_OFFSET
    // TURRET_ANGLE_OFFSET (radians) corrects for the camera NOT pointing
    // straight robot-forward when turretPosition == 0.5. Used only for the
    // "Turret rel. angle" telemetry line now that aiming is manual.
    final double TURRET_ANGLE_OFFSET = 0.0;

    // Last camera-measured straight-line distance to the tag (inches).
    // Drives the flywheel distance chart. -1 until first seen.
    private double tagDistanceInches = -1.0;

    // --- VISION (AprilTag) ---
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;

    // --- CAMERA EXPOSURE (G2 dpad_right = +, G2 dpad_left = -) ---
    // Exposure can only be touched once the camera is actually STREAMING, so
    // it's set up lazily on the first loop where that's true: the camera is
    // switched to MANUAL exposure and the current value is captured. Each
    // dpad tap then nudges it by EXPOSURE_STEP_MS, clamped to the camera's
    // reported min/max. Lower exposure = less motion blur (better AprilTag
    // detection while moving); higher = brighter in dim lighting.
    private ExposureControl exposureControl;
    private boolean exposureInitialized = false;
    private long exposureMs    = 0;
    private long exposureMinMs = 1;
    private long exposureMaxMs = 1;
    private boolean g2DpadRightPrev = false;
    private boolean g2DpadLeftPrev  = false;
    final long EXPOSURE_STEP_MS = 1; // ms per dpad tap

    // ============================================================
    //   FLYWHEEL DISTANCE -> POWER CHARTS  (two separate regimes)
    // ============================================================
    // The flywheel has TWO distinct charts, one per spin-up regime. They
    // are NOT one continuous curve and must not be connected — each is
    // valid only within its own distance band and its own spin-up time.
    //
    //   NEAR regime  (tag <  76.5 in): NEAR chart, 1.20 s spin-up.
    //   FAR  regime  (tag >= 76.5 in): FAR  chart, 3.15 s spin-up.
    //
    // When a tag is visible, flywheelSpeedManual is interpolated from
    // whichever chart matches the measured distance. With no tag, the
    // chart does not apply and the G2 dpad manual value is held.
    //
    // --- NEAR chart (tag < 76.5 in) ---
    //   distance < 29.7 in : clamped to the 29.7 in power (0.75).
    final double[] FLYWHEEL_CHART_DISTANCE = {
            76.4, 71.4, 65.8, 60.5, 55.5, 50.1, 44.8, 40.0, 35.0, 29.7
    };
    final double[] FLYWHEEL_CHART_POWER = {
            1.000, 0.975, 0.950, 0.910, 0.875, 0.850, 0.830, 0.815, 0.800, 0.750
    };
    // Boundary between the near and far regimes.
    final double FLYWHEEL_CHART_MAX_DISTANCE = 76.5; // inches

    // --- FAR chart (tag >= 76.5 in) ---
    // Measured points: 118 in -> 0.95, 122 in -> 0.97, 128 in -> 1.00.
    // These lie on a straight line of slope +0.005 power per inch:
    //     power = FAR_BASE_POWER + FAR_SLOPE * (distance - FAR_BASE_DIST)
    // The line is extrapolated across the WHOLE far range (76.5 in and
    // up), then clamped to a maximum of 1.0 — coming closer than ~108 in
    // the line would exceed full power, so it saturates at 1.0.
    final double FAR_BASE_DIST   = 128.0; // reference point distance (in)
    final double FAR_BASE_POWER  = 1.000; // power at FAR_BASE_DIST
    final double FAR_SLOPE       = 0.005; // power per inch (+ = farther needs more)
    final double FAR_POWER_MAX   = 1.000; // clamp ceiling


    // ============================================================
    //   SERVO POSITIONS  <-- SET YOUR VALUES HERE
    // ============================================================
    final double BLOCKER_BLOCKED_POSITION = 0.15;
    final double BLOCKER_LAUNCH_POSITION  = 0.45;

    // Blocker auto-return: after A moves the blocker to LAUNCH, it returns
    // to BLOCKED this many seconds later. Pressing B cancels the timer.
    final double BLOCKER_AUTO_RETURN_SECONDS = 2.0;

    // Light servo positions (AprilTag status indicator)
    final double LIGHT_TAG_NOT_SEEN  = 0.278;  // no tag detected
    final double LIGHT_TAG_LEFT      = 0.388;  // tag detected, off to the LEFT
    final double LIGHT_TAG_RIGHT     = 0.555;  // tag detected, off to the RIGHT
    final double LIGHT_TAG_CENTERED  = 0.480;  // tag detected and centered (<2 deg bearing)
    final double LIGHT_CENTERED_BEARING_DEG = 2.0;
    final double LIGHT_PTO_ENGAGED   = 0.002;    // PTO engaged — overrides tag states

    final double PTO_DISENGAGED = 0.8;
    final double PTO_ENGAGED    = 0.5;
    // ============================================================

    // --- 2. ROBOT GEOMETRY ---
    final double TRACK_WIDTH = 17.258;
    final double WHEELBASE   = 13.544;
    final double R = Math.hypot(TRACK_WIDTH, WHEELBASE);

    final double FRONT_LEFT_OFFSET  = 0.1200;
    final double FRONT_RIGHT_OFFSET = 1.3861;
    final double BACK_LEFT_OFFSET   = 1.6965;
    final double BACK_RIGHT_OFFSET  = 0.8225;

    // --- 4. TUNING PARAMETERS ---
    final double STEER_KP        = 0.6;
    final double DRIVE_DEADBAND  = 0.05;
    final double STEER_DEADBAND  = 0.05;

    // --- 5. SPEED CONTROL CONSTANTS ---
    // SWERVE_SPEED_PERCENT is the single global knob for the swerve drive's
    // top speed, as a fraction of full motor power (0.0..1.0). Set it to 0.90
    // for 90%. MAX_SPEED_GLOBAL derives from it, so changing this one line
    // changes the normal (non-slow-mode) drive cap everywhere.
    final double SWERVE_SPEED_PERCENT = 0.90;          // 0.90 = 90% drive speed
    final double MAX_SPEED_GLOBAL    = SWERVE_SPEED_PERCENT;
    final double MAX_SPEED_SLOW_MODE = 0.2;

    // --- 6. FLYWHEEL / INTAKE CONSTANTS ---
    final double MOTOR_COAST_RAMP_SECONDS = 0.5;

    // --- 6b. INTAKE SPEED ---
    // Intake runs at a single fixed speed (the G1 dpad speed presets were
    // removed). intakeSpeedTarget is kept as the named power level.
    final double INTAKE_SPEED_HIGH = 0.90;
    private double intakeSpeedTarget = INTAKE_SPEED_HIGH; // fixed intake power

    // --- 6c. FLYWHEEL MANUAL SPEED (G2 dpad up/down; RT = fine step) ---
    final double FLYWHEEL_SPEED_STEP_COARSE = 0.05;   // per-tap, RT not held
    final double FLYWHEEL_SPEED_STEP_FINE   = 0.01;   // per-tap, RT held
    private double flywheelSpeedManual = 1.0;         // 0.0 .. 1.0, scaled by voltageFactor
    private boolean g2DpadUpPrev   = false;
    private boolean g2DpadDownPrev = false;

    // --- 6d. INTAKE REVERSE-HOLD (G1 dpad_left) ---
    // While dpad_left is held, run intake in reverse at this power.
    // The intake's normal toggle state is preserved and resumes on release.
    //   PTO DISENGAGED -> INTAKE_REVERSE_HOLD_POWER
    //   PTO ENGAGED    -> INTAKE_REVERSE_HOLD_POWER_PTO (faster, for clearing
    //                     jams quickly during the climb sequence)
    final double INTAKE_REVERSE_HOLD_POWER     = 0.50;
    final double INTAKE_REVERSE_HOLD_POWER_PTO = 0.90;

    // --- 6e. INTAKE STALL DETECTION (current-based) ---
    // 5000 Series motor: stall current @12V = 9.2A. Two mechanically-linked motors
    // share the load, so 7.0A sustained on either motor indicates a genuine jam.
    // If max(topIntake, bottomIntake) current exceeds this for longer than
    // INTAKE_STALL_TIME_SECONDS continuously, the intake auto-toggles OFF.
    final double INTAKE_STALL_CURRENT_AMPS = 7.0;
    final double INTAKE_STALL_TIME_SECONDS = 0.75;
    private ElapsedTime intakeStallTimer = new ElapsedTime();
    private boolean intakeStallTiming = false;   // true while current is above threshold
    private boolean intakeStallTripped = false;  // true on the loop a stall shut the intake off

    // --- 7. TOGGLES / STATES ---
    private boolean rightStickButtonPreviouslyPressed = false;

    // Intake toggle
    private boolean intakeRunning = false;
    private boolean rightTriggerPreviouslyPressed = false;

    // Flywheel toggle
    private boolean flywheelRunning = false;
    private boolean leftTriggerPreviouslyPressed = false;
    // Reset each time the flywheel toggles ON; measures continuous run time.
    private ElapsedTime flywheelOnTimer = new ElapsedTime();

    // Blocker — A = launch, B = blocked
    private boolean aButtonPreviouslyPressed = false;
    private boolean bButtonPreviouslyPressed = false;

    // Blocker manual toggle — G1 X flips the blocker open <-> closed.
    // blockerOpen tracks the current physical state so X knows which way to go;
    // it's kept in sync everywhere the blocker position is commanded.
    private boolean xButtonPreviouslyPressed = false;
    private boolean blockerOpen = false;

    // Blocker auto-return timer: armed by A (launch), cancelled by B.
    private boolean blockerAutoReturnArmed = false;
    private ElapsedTime blockerAutoReturnTimer = new ElapsedTime();

    // Blocker-launch sequence: G1 A registers a pending-open request. The
    // blocker only actually opens once the flywheel has been running
    // continuously for the required spin-up time. After it opens, the
    // intake is forced to full power (the toggle speed) once
    // BLOCKER_LAUNCH_INTAKE_DELAY has elapsed. Closing the blocker cuts
    // intake AND flywheel.
    //
    // SPIN-UP TIME IS DISTANCE-DEPENDENT:
    //   - tag at/over FLYWHEEL_CHART_MAX_DISTANCE (76.5 in), or no tag:
    //         BLOCKER_FLYWHEEL_SPINUP_FAR  (3.15 s) — longer spin-up.
    //   - tag closer than that:
    //         BLOCKER_FLYWHEEL_SPINUP_NEAR (1.20 s).
    // The required time is recomputed each loop from the live tag distance.
    final double BLOCKER_FLYWHEEL_SPINUP_NEAR = 1.20; // tag < 76.5 in
    final double BLOCKER_FLYWHEEL_SPINUP_FAR  = 3.15; // tag >= 76.5 in or no tag
    final double BLOCKER_LAUNCH_INTAKE_DELAY  = 0.4;  // intake delay after blocker opens

    // --- FLYWHEEL FULL-POWER OVERRIDE  (G1 dpad_right) ---
    // dpad_right engages a full-power (1.0) override that ALSO forces the
    // blocker spin-up time. It has two states it toggles between, both at
    // full power, differing only in spin-up time:
    //     OVERRIDE_SPINUP_FAR  (3.35 s)  <-- engaged here on the first press
    //     OVERRIDE_SPINUP_NEAR (1.5 s)
    // The override starts OFF so the distance chart / G2 manual value govern
    // until the driver opts in. The first dpad_right press engages it at the
    // 3.35 s state; each subsequent press flips 3.35 s <-> 1.5 s. While active
    // it overrides the distance chart and the G2 manual speed entirely.
    // G1 Y switches the override back OFF, re-enabling the auto adjustment.
    final double OVERRIDE_SPINUP_FAR  = 3.35; // dpad_right "far"  state
    final double OVERRIDE_SPINUP_NEAR = 1.5;  // dpad_right "near" state
    private boolean flywheelOverrideActive = false; // false until first dpad_right press
    private boolean flywheelOverrideFar    = true;  // true = 3.35 s, false = 1.5 s
    private boolean g1DpadRightPrev        = false;
    private boolean blockerOpenPending = false;      // A pressed, waiting for flywheel spin-up
    private boolean blockerLaunchMode  = false;      // true between blocker-open and close
    private ElapsedTime blockerLaunchTimer = new ElapsedTime();

    // Ramp-down state for graceful coast-to-stop
    private boolean intakeRampingDown   = false;
    private boolean flywheelRampingDown = false;
    private ElapsedTime intakeRampTimer   = new ElapsedTime();
    private ElapsedTime flywheelRampTimer = new ElapsedTime();
    private double intakeRampStartPower   = 0.0;
    private double flywheelRampStartPower = 0.0;

    // Wheel 'planting'
    final int FRAMES_TO_PLANT_WHEELS = 5;
    private int framesSinceLastMoved = 0;

    // Heading
    private double headingOffset = 0.0;
    private boolean dpadUpPrev   = false;

    // --- DRIVE MODE (G1 LB + dpad_up toggles) ---
    // false = field-centric (IMU heading rotates stick input into robot frame).
    // true  = robot-centric  (sticks command the robot frame directly, no IMU
    //         math — same as the justSwerve op-mode).
    private boolean robotCentric = false;
    private boolean driveModeTogglePrev = false; // rising-edge tracker for LB+dpad_up

    // --- BACK-LEFT CASTER (G1 RB + X held) ---
    // While RB + X are held, the back-left module is released: its drive motor
    // is switched to FLOAT (coast) and commanded 0, and its steer servo is
    // commanded 0, so the wheel free-rolls and free-swivels like a caster.
    // backLeftCoasting tracks the drive motor's zero-power mode so we only send
    // a setZeroPowerBehavior command on transitions (not every loop).
    private boolean backLeftCoasting = false;

    @Override
    public void runOpMode() {
        initializeHardware();
        initializeVision();

        // No servo commands during init — servos hold their power-on position
        // until the main loop runs after start. Match-start positioning happens
        // on the first loop iteration below.

        telemetry.addData("Status", "ApolloDrive ready");
        telemetry.update();

        waitForStart();

        // One-time servo positioning, AFTER start (not during init, so nothing
        // moves on the field before the match). Runs once on the first loop.
        boolean servosInitialized = false;

        double targetAngleFL = 0, targetAngleFR = 0, targetAngleBL = 0, targetAngleBR = 0;

        while (opModeIsActive()) {

            if (!servosInitialized) {
                blocker.setPosition(BLOCKER_BLOCKED_POSITION);
                blockerOpen = false;
                pto.setPosition(PTO_DISENGAGED);
                leftTurret.setPosition(turretPosition);
                rightTurret.setPosition(turretPosition);
                light.setPosition(LIGHT_TAG_NOT_SEEN);
                servosInitialized = true;
            }

            boolean rbHeld = gamepad1.right_bumper;

            // ============================================================
            //   PTO ENGAGE   — RB + Dpad Up   (rising edge)
            //   PTO DISENGAGE — RB + Dpad Down (rising edge)
            // ============================================================
            boolean ptoDpadUpNow   = rbHeld && gamepad1.dpad_up;
            boolean ptoDpadDownNow = rbHeld && gamepad1.dpad_down;

            if (ptoDpadUpNow && !ptoDpadUpPreviouslyPressed) {
                ptoEngaged = true;
                pto.setPosition(PTO_ENGAGED);
                blocker.setPosition(BLOCKER_LAUNCH_POSITION);
                blockerOpen = true;
                // Stop intake toggle state so normal mode is clean on disengage
                intakeRunning     = false;
                intakeRampingDown = false;
                // Clear any stall-detection state so it doesn't carry across modes
                intakeStallTiming = false;
            }
            if (ptoDpadDownNow && !ptoDpadDownPreviouslyPressed) {
                ptoEngaged = false;
                pto.setPosition(PTO_DISENGAGED);
            }

            ptoDpadUpPreviouslyPressed   = ptoDpadUpNow;
            ptoDpadDownPreviouslyPressed = ptoDpadDownNow;

            // ============================================================
            //   FLYWHEEL MANUAL SPEED ADJUST  (G2 dpad up/down)
            //   Step = 0.05 normally, 0.01 with RT held.
            //   Rising-edge: each tap nudges once, clamped to [0, 1].
            //   NOTE: when the flywheel distance chart is active (tag visible
            //   and within range) it overwrites flywheelSpeedManual each loop,
            //   so this manual adjust only takes effect at long range / no tag.
            // ============================================================
            boolean g2RtHeld      = gamepad2.right_trigger > TURRET_RT_THRESHOLD;
            double  flywheelStep  = g2RtHeld ? FLYWHEEL_SPEED_STEP_FINE
                    : FLYWHEEL_SPEED_STEP_COARSE;
            boolean g2DpadUpNow   = gamepad2.dpad_up;
            boolean g2DpadDownNow = gamepad2.dpad_down;

            if (g2DpadUpNow && !g2DpadUpPrev) {
                flywheelSpeedManual += flywheelStep;
            }
            if (g2DpadDownNow && !g2DpadDownPrev) {
                flywheelSpeedManual -= flywheelStep;
            }
            flywheelSpeedManual = Math.max(0.0, Math.min(1.0, flywheelSpeedManual));

            g2DpadUpPrev   = g2DpadUpNow;
            g2DpadDownPrev = g2DpadDownNow;

            // ============================================================
            //   FLYWHEEL FULL-POWER OVERRIDE TOGGLE  (G1 dpad_right, rising edge)
            //   First press engages the override at the 3.35 s state. Each
            //   later press flips 3.35 s <-> 1.5 s. Always full power once
            //   engaged. G1 Y switches it back off (see below), restoring the
            //   distance chart and the G2 manual speed. The forced power is
            //   applied after the distance-chart block below, and the spin-up
            //   time is read by requiredSpinupSeconds(). Processed regardless
            //   of PTO (it only affects the flywheel, which is frozen during
            //   PTO anyway).
            // ============================================================
            boolean g1DpadRightNow = gamepad1.dpad_right;
            if (g1DpadRightNow && !g1DpadRightPrev) {
                if (!flywheelOverrideActive) {
                    flywheelOverrideActive = true;
                    flywheelOverrideFar    = true;  // engage at 3.35 s
                } else {
                    flywheelOverrideFar = !flywheelOverrideFar; // 3.35 s <-> 1.5 s
                }
            }
            g1DpadRightPrev = g1DpadRightNow;

            // ============================================================
            //   RE-ENABLE AUTO ADJUSTMENT  (G1 Y)
            //   Turns the full-power override OFF, handing flywheel speed and
            //   spin-up time back to the distance chart / G2 manual value.
            //   Idempotent (a held Y just keeps it off), so no edge tracking
            //   is needed. Processed regardless of PTO, like the dpad_right
            //   toggle above.
            // ============================================================
            if (gamepad1.y) {
                flywheelOverrideActive = false;
            }

            // ============================================================
            //   CAMERA EXPOSURE ADJUST  (G2 dpad_right = +, dpad_left = -)
            //   Lazily switches the camera to MANUAL exposure the first loop
            //   it is STREAMING, then each dpad tap nudges exposure by
            //   EXPOSURE_STEP_MS, clamped to the camera's reported range.
            // ============================================================
            if (!exposureInitialized
                    && visionPortal != null
                    && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                exposureControl = visionPortal.getCameraControl(ExposureControl.class);
                if (exposureControl != null) {
                    if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                        exposureControl.setMode(ExposureControl.Mode.Manual);
                    }
                    exposureMinMs = Math.max(1, exposureControl.getMinExposure(TimeUnit.MILLISECONDS));
                    exposureMaxMs = exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);
                    exposureMs    = exposureControl.getExposure(TimeUnit.MILLISECONDS);
                    if (exposureMs < exposureMinMs) exposureMs = exposureMinMs;
                    if (exposureMs > exposureMaxMs) exposureMs = exposureMaxMs;
                }
                exposureInitialized = true;
            }

            boolean g2DpadRightNow = gamepad2.dpad_right;
            boolean g2DpadLeftNow  = gamepad2.dpad_left;
            if (exposureControl != null) {
                if (g2DpadRightNow && !g2DpadRightPrev) {
                    exposureMs = Math.min(exposureMaxMs, exposureMs + EXPOSURE_STEP_MS);
                    exposureControl.setExposure(exposureMs, TimeUnit.MILLISECONDS);
                }
                if (g2DpadLeftNow && !g2DpadLeftPrev) {
                    exposureMs = Math.max(exposureMinMs, exposureMs - EXPOSURE_STEP_MS);
                    exposureControl.setExposure(exposureMs, TimeUnit.MILLISECONDS);
                }
            }
            g2DpadRightPrev = g2DpadRightNow;
            g2DpadLeftPrev  = g2DpadLeftNow;

            // ============================================================
            //   DRIVE MODE TOGGLE  (G1 LB + dpad_up, rising edge)
            //   Toggles between field-centric and robot-centric drive.
            //   Robot-centric skips the IMU heading transform entirely (the
            //   sticks drive the robot frame directly, like justSwerve).
            //   Checked before the plain dpad_up heading reset, and the reset
            //   is gated on !lbHeld so this combo does NOT also reset heading.
            // ============================================================
            boolean lbHeld = gamepad1.left_bumper;
            boolean driveModeToggleNow = lbHeld && gamepad1.dpad_up;
            if (driveModeToggleNow && !driveModeTogglePrev) {
                robotCentric = !robotCentric;
            }
            driveModeTogglePrev = driveModeToggleNow;

            // ============================================================
            //   FIELD HEADING RESET  (G1 dpad_up, RB and LB not held)
            //   Sets the current robot heading to 0 (+X) on the REV control-
            //   hub IMU used for field-centric drive. Matches justFieldSwerving.
            // ============================================================
            boolean dpadUpNow = gamepad1.dpad_up;
            if (dpadUpNow && !dpadUpPrev && !rbHeld && !lbHeld) {
                imu.resetYaw();
                headingOffset = 0.0;
            }
            dpadUpPrev = dpadUpNow;

            // ============================================================
            //   INTAKE MOTOR CONTROL
            //
            //   PTO ACTIVE:
            //     dpad_left held → intakes run reverse (jam-clear), priority
            //     Both triggers held simultaneously → intakes run backward
            //     Anything else → intakes stop
            //     All other mechanisms are frozen (no trigger/button changes
            //     processed for flywheel, blocker, turret)
            //     Current-based stall detection is NOT active here.
            //
            //   PTO INACTIVE:
            //     Normal right-trigger toggle behavior,
            //     plus dpad_left reverse-hold and current-based stall detection.
            // ============================================================
            double intakePower = 0.0;          // forward-positive command (pre-sign)
            boolean intakeReverseHoldActive = false;

            if (ptoEngaged) {
                boolean bothTriggersHeld = (gamepad1.left_trigger > 0.5) && (gamepad1.right_trigger > 0.5);

                // dpad_left reverse-hold also works while PTO is engaged so the
                // driver can clear a jam in either mode. It takes priority over
                // the both-triggers command. Same motor direction as the
                // PTO-inactive reverse-hold (normal mode runs setPower with a
                // positive value; the PTO branch sets power un-negated, so no
                // sign flip needed) but at the faster PTO power.
                if (gamepad1.dpad_left) {
                    topIntake.setPower(INTAKE_REVERSE_HOLD_POWER_PTO);
                    bottomIntake.setPower(INTAKE_REVERSE_HOLD_POWER_PTO);
                } else {
                    intakePower = bothTriggersHeld ? -1.0 : 0.0;
                    topIntake.setPower(intakePower);
                    bottomIntake.setPower(intakePower);
                }

                // Keep previous-press states in sync so normal mode resumes cleanly
                rightTriggerPreviouslyPressed = gamepad1.right_trigger > 0.5;
                leftTriggerPreviouslyPressed  = gamepad1.left_trigger  > 0.5;

                // Freeze flywheel (keep power at whatever it was — just don't update)
                // Update speed multiplier and drive as normal below

            } else {
                // --------------------------------------------------------
                //   NORMAL INTAKE TOGGLE  (Right Trigger)
                // --------------------------------------------------------
                boolean rightTriggerCurrentlyPressed = gamepad1.right_trigger > 0.5;
                if (rightTriggerCurrentlyPressed && !rightTriggerPreviouslyPressed) {
                    if (intakeRunning) {
                        intakeRampingDown    = true;
                        intakeRampStartPower = intakeSpeedTarget;
                        intakeRampTimer.reset();
                        intakeRunning = false;
                    } else {
                        intakeRampingDown = false;
                        intakeRunning     = true;
                    }
                }
                rightTriggerPreviouslyPressed = rightTriggerCurrentlyPressed;

                if (intakeRunning) {
                    intakePower = intakeSpeedTarget;
                } else if (intakeRampingDown) {
                    double fraction = intakeRampTimer.seconds() / MOTOR_COAST_RAMP_SECONDS;
                    if (fraction >= 1.0) {
                        intakeRampingDown = false;
                    } else {
                        intakePower = intakeRampStartPower * (1.0 - fraction);
                    }
                }

                // --------------------------------------------------------
                //   BLOCKER-LAUNCH INTAKE OVERRIDE
                //   While blocker-launch mode is active AND the 0.4s delay
                //   since the blocker opened has elapsed, force the intake on
                //   at the normal toggle speed (intakeSpeedTarget), overriding
                //   the right-trigger toggle even if it was off.
                // --------------------------------------------------------
                if (blockerLaunchMode
                        && blockerLaunchTimer.seconds() >= BLOCKER_LAUNCH_INTAKE_DELAY) {
                    intakePower = intakeSpeedTarget;
                    // Clear ramp-down so it doesn't fight the override.
                    intakeRampingDown = false;
                }

                // --------------------------------------------------------
                //   INTAKE REVERSE-HOLD  (G1 dpad_left)
                //   While held, override the intake command with a reverse
                //   run at 50%. Normal toggle state is untouched and resumes
                //   automatically when dpad_left is released.
                //   Reverse-hold wins over the blocker-launch override so the
                //   driver can always manually clear a jam.
                // --------------------------------------------------------
                intakeReverseHoldActive = gamepad1.dpad_left;
                if (intakeReverseHoldActive) {
                    intakePower = -INTAKE_REVERSE_HOLD_POWER;
                }

                // topIntake/bottomIntake spin forward on negative power in this
                // robot's wiring (see original mapping), so the command is negated.
                topIntake.setPower(-intakePower);
                bottomIntake.setPower(-intakePower);

                // --------------------------------------------------------
                //   INTAKE STALL DETECTION  (current-based, PTO inactive only)
                //   Only arm while the intake is actively commanded forward via
                //   the toggle. Reverse-hold and ramp-down are excluded so they
                //   can't trip the stall logic.
                // --------------------------------------------------------
                boolean stallCheckArmed = intakeRunning && !intakeReverseHoldActive;
                if (stallCheckArmed) {
                    double topCurrent = topIntake.getCurrent(CurrentUnit.AMPS);
                    double botCurrent = bottomIntake.getCurrent(CurrentUnit.AMPS);
                    double maxCurrent = Math.max(topCurrent, botCurrent);

                    if (maxCurrent > INTAKE_STALL_CURRENT_AMPS) {
                        if (!intakeStallTiming) {
                            intakeStallTiming = true;
                            intakeStallTimer.reset();
                        } else if (intakeStallTimer.seconds() >= INTAKE_STALL_TIME_SECONDS) {
                            // Sustained stall — toggle intake OFF.
                            // No lockout: driver may immediately re-toggle.
                            intakeRunning      = false;
                            intakeRampingDown  = false;
                            intakeStallTiming  = false;
                            intakeStallTripped = true;
                            // Cut power this loop so the jammed motors stop now.
                            topIntake.setPower(0.0);
                            bottomIntake.setPower(0.0);
                            intakePower = 0.0;
                        }
                    } else {
                        // Current dropped back below threshold — reset the timer.
                        intakeStallTiming = false;
                    }
                } else {
                    // Intake not in a state we monitor — keep stall timer disarmed.
                    intakeStallTiming = false;
                }

                // --------------------------------------------------------
                //   FLYWHEEL TOGGLE  (Left Trigger)
                // --------------------------------------------------------
                boolean leftTriggerCurrentlyPressed = gamepad1.left_trigger > 0.5;
                if (leftTriggerCurrentlyPressed && !leftTriggerPreviouslyPressed) {
                    if (flywheelRunning) {
                        double voltage = voltageSensor.getVoltage();
                        double vf = (voltage > 0) ? Math.min(12.0 / voltage, 1.0) : 1.0;
                        flywheelRampStartPower = flywheelSpeedManual * vf;
                        flywheelRampingDown    = true;
                        flywheelRampTimer.reset();
                        flywheelRunning = false;
                    } else {
                        flywheelRampingDown = false;
                        flywheelRunning     = true;
                        // Start measuring continuous run time for blocker spin-up gate.
                        flywheelOnTimer.reset();
                    }
                }
                leftTriggerPreviouslyPressed = leftTriggerCurrentlyPressed;

                // --------------------------------------------------------
                //   BLOCKER  (A = launch, B = blocked)
                //   G1 A is a one-press launch button:
                //     - flywheels OFF -> spin them up AND register the launch.
                //       The blocker opens automatically once the flywheel has
                //       run for the required spin-up time.
                //     - flywheels ON  -> register the launch; blocker opens once
                //       the spin-up gate is met (immediately if already met).
                //   Either way, a single A press takes it all the way to launch.
                //   The left trigger still toggles flywheels on/off independently.
                //   G1 B moves to BLOCKED, cancels everything, cuts mechanisms.
                //
                //   The required spin-up time depends on tag distance: see
                //   requiredSpinupSeconds() — 3.15 s at/over 76.5 in (or no
                //   tag), 1.20 s closer in.
                // --------------------------------------------------------
                boolean aButtonCurrentlyPressed = gamepad1.a;
                boolean bButtonCurrentlyPressed = gamepad1.b;

                if (aButtonCurrentlyPressed && !aButtonPreviouslyPressed) {
                    if (!flywheelRunning) {
                        // Flywheels off — A spins them up...
                        flywheelRampingDown = false;
                        flywheelRunning     = true;
                        flywheelOnTimer.reset();
                    }
                    // ...and registers the launch. The blocker open is gated on
                    // the spin-up time below, so one press goes cold -> launch.
                    blockerOpenPending = true;
                }

                // Open the blocker once the flywheel spin-up gate is satisfied:
                // flywheel must be running NOW and have been for at least the
                // distance-dependent required spin-up time.
                if (blockerOpenPending
                        && flywheelRunning
                        && flywheelOnTimer.seconds() >= requiredSpinupSeconds()) {
                    blocker.setPosition(BLOCKER_LAUNCH_POSITION);
                    blockerOpen = true;
                    blockerOpenPending = false;
                    // Arm the auto-return timer.
                    blockerAutoReturnArmed = true;
                    blockerAutoReturnTimer.reset();
                    // Begin blocker-launch mode: after a delay the intake is
                    // forced to full power. Timer counts from the open.
                    blockerLaunchMode = true;
                    blockerLaunchTimer.reset();
                }

                if (bButtonCurrentlyPressed && !bButtonPreviouslyPressed) {
                    blocker.setPosition(BLOCKER_BLOCKED_POSITION);
                    blockerOpen = false;
                    // B cancels a pending auto-return and any pending open.
                    blockerAutoReturnArmed = false;
                    blockerOpenPending     = false;
                    // Blocker closed by B — end launch mode and cut intake+flywheel.
                    if (blockerLaunchMode) {
                        blockerLaunchMode = false;
                        cutIntakeAndFlywheel();
                    }
                }

                // --------------------------------------------------------
                //   BLOCKER MANUAL TOGGLE  (G1 X, RB not held)
                //   Flips the blocker open <-> closed directly, ignoring the
                //   flywheel spin-up gate. Opening this way is a plain manual
                //   hold — no auto-return, so it stays open until X closes it.
                //   Closing cancels any pending open / auto-return and, if a
                //   launch sequence was active, cuts intake + flywheel (same as B).
                //   Gated on !rbHeld so the RB+X back-left caster combo does NOT
                //   also toggle the blocker. xButtonPreviouslyPressed still
                //   tracks the raw button so releasing RB while holding X can't
                //   produce a spurious toggle.
                // --------------------------------------------------------
                boolean xButtonCurrentlyPressed = gamepad1.x;
                if (xButtonCurrentlyPressed && !xButtonPreviouslyPressed && !rbHeld) {
                    if (blockerOpen) {
                        blocker.setPosition(BLOCKER_BLOCKED_POSITION);
                        blockerOpen            = false;
                        blockerAutoReturnArmed = false;
                        blockerOpenPending     = false;
                        if (blockerLaunchMode) {
                            blockerLaunchMode = false;
                            cutIntakeAndFlywheel();
                        }
                    } else {
                        blocker.setPosition(BLOCKER_LAUNCH_POSITION);
                        blockerOpen            = true;
                        blockerOpenPending     = false;
                        blockerAutoReturnArmed = false; // manual open holds until X closes it
                    }
                }
                xButtonPreviouslyPressed = xButtonCurrentlyPressed;

                // Auto-return: once 2s have elapsed since the blocker opened,
                // send it back to BLOCKED and disarm. This also ends launch
                // mode and cuts intake + flywheel.
                if (blockerAutoReturnArmed
                        && blockerAutoReturnTimer.seconds() >= BLOCKER_AUTO_RETURN_SECONDS) {
                    blocker.setPosition(BLOCKER_BLOCKED_POSITION);
                    blockerOpen = false;
                    blockerAutoReturnArmed = false;
                    if (blockerLaunchMode) {
                        blockerLaunchMode = false;
                        cutIntakeAndFlywheel();
                    }
                }

                aButtonPreviouslyPressed = aButtonCurrentlyPressed;
                bButtonPreviouslyPressed = bButtonCurrentlyPressed;
            }

            // ============================================================
            //   FLYWHEEL POWER (computed every loop, updated outside PTO block)
            //   Manual speed setting (G2) is scaled by the voltage factor.
            // ============================================================
            double flywheelPower = 0.0;
            if (flywheelRunning) {
                double voltage = voltageSensor.getVoltage();
                double vf = (voltage > 0) ? Math.min(12.0 / voltage, 1.0) : 1.0;
                flywheelPower = flywheelSpeedManual * vf;
            } else if (flywheelRampingDown) {
                double fraction = flywheelRampTimer.seconds() / MOTOR_COAST_RAMP_SECONDS;
                if (fraction >= 1.0) {
                    flywheelRampingDown = false;
                } else {
                    flywheelPower = flywheelRampStartPower * (1.0 - fraction);
                }
            }
            leftFly.setPower(flywheelPower);
            rightFly.setPower(flywheelPower);

            // ============================================================
            //   CALIBRATION MODE TOGGLE  (R3) — reserved
            // ============================================================
            boolean rightStickButtonCurrentlyPressed = gamepad1.right_stick_button;
            rightStickButtonPreviouslyPressed = rightStickButtonCurrentlyPressed;

            // --- Speed Limiter (slow mode only when PTO not engaged) ---
            double speedMultiplier = MAX_SPEED_GLOBAL;
            if (rbHeld && !ptoEngaged) speedMultiplier = MAX_SPEED_SLOW_MODE;

            // ============================================================
            //   SWERVE DRIVE  (field-centric, or robot-centric via LB+dpad_up)
            // ============================================================
            double fieldY = -gamepad1.left_stick_y * speedMultiplier;
            double fieldX =  gamepad1.left_stick_x * speedMultiplier;
            double rot    =  gamepad1.right_stick_x * speedMultiplier;

            double robotX, robotY;
            if (robotCentric) {
                // ROBOT-CENTRIC: sticks command the robot frame directly, no
                // IMU heading math (identical to the justSwerve op-mode).
                robotX = fieldX;
                robotY = fieldY;
            } else {
                // FIELD-CENTRIC: rotate the field-frame stick input into the
                // robot frame by the current IMU heading.
                double rawHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                double botHeading = wrapAngle(rawHeading - headingOffset);
                robotX = fieldX * Math.cos(-botHeading) - fieldY * Math.sin(-botHeading);
                robotY = fieldX * Math.sin(-botHeading) + fieldY * Math.cos(-botHeading);
            }

            boolean invertRobotX = false;
            if (invertRobotX) robotX = -robotX;

            double A = robotX - rot * (WHEELBASE / R);
            double B = robotX + rot * (WHEELBASE / R);
            double C = robotY - rot * (TRACK_WIDTH / R);
            double D = robotY + rot * (TRACK_WIDTH / R);

            double speedFrontLeft  = Math.hypot(B, D);
            double speedFrontRight = Math.hypot(B, C);
            double speedBackLeft   = Math.hypot(A, D);
            double speedBackRight  = Math.hypot(A, C);

            double maxSpeed = Math.max(
                    Math.max(speedFrontLeft, speedFrontRight),
                    Math.max(speedBackLeft,  speedBackRight)
            );
            if (maxSpeed > 1.0) {
                speedFrontLeft  /= maxSpeed;
                speedFrontRight /= maxSpeed;
                speedBackLeft   /= maxSpeed;
                speedBackRight  /= maxSpeed;
            }

            // Deadband acts on the RAW sticks, so it behaves the same in slow
            // mode as at full speed (independent of speedMultiplier).
            boolean driverActive =
                    Math.abs(gamepad1.left_stick_x)  > DRIVE_DEADBAND ||
                            Math.abs(gamepad1.left_stick_y)  > DRIVE_DEADBAND ||
                            Math.abs(gamepad1.right_stick_x) > DRIVE_DEADBAND;

            if (driverActive) {
                targetAngleFL = Math.atan2(B, D);
                targetAngleFR = Math.atan2(B, C);
                targetAngleBL = Math.atan2(A, D);
                targetAngleBR = Math.atan2(A, C);
                framesSinceLastMoved = 0;
            } else {
                speedFrontLeft = 0; speedFrontRight = 0;
                speedBackLeft  = 0; speedBackRight  = 0;
                framesSinceLastMoved++;
            }

            boolean lockWheels = gamepad1.left_stick_button || framesSinceLastMoved >= FRAMES_TO_PLANT_WHEELS;
            if (lockWheels) {
                targetAngleFL = -Math.PI / 4; targetAngleFR =  Math.PI / 4;
                targetAngleBL =  Math.PI / 4; targetAngleBR = -Math.PI / 4;
                speedFrontLeft = 0; speedFrontRight = 0;
                speedBackLeft  = 0; speedBackRight  = 0;
            }

            ModuleDebug fl = runModule(frontLeftDrive,  frontLeftSteer,  frontLeftEncoder,  FRONT_LEFT_OFFSET,  speedFrontLeft,  targetAngleFL, "FL");
            ModuleDebug fr = runModule(frontRightDrive, frontRightSteer, frontRightEncoder, FRONT_RIGHT_OFFSET, speedFrontRight, targetAngleFR, "FR");

            // BACK-LEFT CASTER: RB + X held releases the back-left module so it
            // free-rolls (drive coasts on FLOAT) and free-swivels (steer power
            // 0). Otherwise it runs as a normal swerve module. The drive
            // motor's zero-power mode is only changed on transitions.
            boolean backLeftCaster = rbHeld && gamepad1.x;
            ModuleDebug bl;
            if (backLeftCaster) {
                if (!backLeftCoasting) {
                    backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    backLeftCoasting = true;
                }
                backLeftDrive.setPower(0.0);  // FLOAT + 0 power => wheel coasts
                backLeftSteer.setPower(0.0);  // no steering hold => free swivel
                bl = null;
            } else {
                if (backLeftCoasting) {
                    backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    backLeftCoasting = false;
                }
                bl = runModule(backLeftDrive, backLeftSteer, backLeftEncoder, BACK_LEFT_OFFSET, speedBackLeft, targetAngleBL, "BL");
            }

            ModuleDebug br = runModule(backRightDrive,  backRightSteer,  backRightEncoder,  BACK_RIGHT_OFFSET,  speedBackRight,  targetAngleBR, "BR");

            // ============================================================
            //   APRILTAG DETECTION (target tag, see getTargetTagId())
            // ============================================================
            AprilTagDetection targetTag = null;
            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.id == getTargetTagId()) {
                    targetTag = detection;
                    break;
                }
            }

            // ============================================================
            //   LIGHT SERVO + TAG DISTANCE (camera's only roles now)
            //   PTO engaged    -> LIGHT_PTO_ENGAGED  (0.0)   — overrides all
            //   not seen       -> LIGHT_TAG_NOT_SEEN (0.278)
            //   seen, on LEFT  -> LIGHT_TAG_LEFT     (0.388)
            //   seen, centered -> LIGHT_TAG_CENTERED (0.480)
            //   seen, on RIGHT -> LIGHT_TAG_RIGHT    (0.555)
            //   Camera is landscape, so side/centering use bearing only.
            //   FTC convention: bearing > 0 = tag to the LEFT, < 0 = RIGHT.
            //   The camera also records tagDistanceInches, which the flywheel
            //   distance chart uses.
            // ============================================================
            double lightPosition = LIGHT_TAG_NOT_SEEN;
            boolean tagCentered = false;
            boolean tagVisibleThisLoop = false;
            if (ptoEngaged) {
                lightPosition = LIGHT_PTO_ENGAGED;
            } else if (targetTag != null && targetTag.ftcPose != null) {
                double bearing = targetTag.ftcPose.bearing;
                tagCentered = Math.abs(bearing) <= LIGHT_CENTERED_BEARING_DEG;
                if (tagCentered) {
                    lightPosition = LIGHT_TAG_CENTERED;
                } else if (bearing > 0) {
                    lightPosition = LIGHT_TAG_LEFT;
                } else {
                    lightPosition = LIGHT_TAG_RIGHT;
                }
                // Record camera-measured distance (drives the flywheel chart).
                tagDistanceInches = targetTag.ftcPose.range;
                tagVisibleThisLoop = true;
            }
            light.setPosition(lightPosition);

            // ============================================================
            //   FLYWHEEL DISTANCE CHART  (auto-set flywheelSpeedManual)
            //   When a tag is visible, flywheelSpeedManual is set from
            //   whichever chart matches the measured distance:
            //     distance <  76.5 in -> NEAR chart (table interpolation,
            //                            clamped to 0.75 below 29.7 in).
            //     distance >= 76.5 in -> FAR chart (linear, +0.005/in,
            //                            clamped to a max of 1.0).
            //   With no tag the chart does not apply and the G2 dpad manual
            //   value is held.
            // ============================================================
            if (tagVisibleThisLoop) {
                if (tagDistanceInches < FLYWHEEL_CHART_MAX_DISTANCE) {
                    flywheelSpeedManual = flywheelPowerForDistance(tagDistanceInches);
                } else {
                    flywheelSpeedManual = flywheelPowerForDistanceFar(tagDistanceInches);
                }
            }

            // Full-power override (G1 dpad_right) wins over BOTH the distance
            // chart and the G2 manual value. Applied last so nothing above can
            // undo it. Takes effect on the next loop's flywheel-power compute,
            // matching the chart's one-loop latency.
            if (flywheelOverrideActive) {
                flywheelSpeedManual = 1.0;
            }

            // ============================================================
            //   TURRET CONTROL  (MANUAL ONLY)
            //   Frozen entirely while PTO is engaged (same as all other
            //   mechanisms).
            //
            //   Manual: G2 left stick X (LT fast / RT slow). Snap presets
            //     X -> 0.25, Y -> 0.5, B -> 0.75, A -> nearest of 0/1.
            //
            //   Position wraps 0<->1 (turret travels a full 360deg; the
            //   servo range 0..1 is continuous; 0.5 = forward).
            // ============================================================
            if (!ptoEngaged) {
                // Snap presets:
                //   X -> 0.25  (left quarter)
                //   Y -> 0.5   (forward / center)
                //   B -> 0.75  (right quarter)
                //   A -> 0.0 or 1.0, whichever is closer to current position
                if (gamepad2.x) {
                    turretPosition = 0.25;
                } else if (gamepad2.y) {
                    turretPosition = 0.5;
                } else if (gamepad2.b) {
                    turretPosition = 0.75;
                } else if (gamepad2.a) {
                    // Snap to whichever end (0 or 1) the turret is currently
                    // closer to. Because the range wraps, 0 and 1 are the same
                    // physical position — this just picks the neater servo value.
                    turretPosition = (turretPosition <= 0.5) ? 0.0 : 1.0;
                } else {
                    double turretStick = gamepad2.left_stick_x;
                    if (Math.abs(turretStick) > TURRET_DEADBAND) {
                        // Speed select: LT held -> fast, RT held -> slow,
                        // neither -> normal. LT wins if both are held.
                        double turretSpeed;
                        if (gamepad2.left_trigger > TURRET_LT_THRESHOLD) {
                            turretSpeed = TURRET_SPEED_FAST;
                        } else if (gamepad2.right_trigger > TURRET_RT_THRESHOLD) {
                            turretSpeed = TURRET_SPEED_SLOW;
                        } else {
                            turretSpeed = TURRET_SPEED_MANUAL;
                        }
                        turretPosition = wrapUnit(turretPosition + turretStick * turretSpeed);
                    }
                }

                leftTurret.setPosition(turretPosition);
                rightTurret.setPosition(turretPosition);
            }

            // ============================================================
            //   TELEMETRY  (drive mode, PTO, exposure, camera/tag, flywheel, light)
            // ============================================================
            telemetry.addData("Drive", robotCentric ? "ROBOT-CENTRIC" : "FIELD-CENTRIC");
            telemetry.addData("PTO", ptoEngaged ? "ENGAGED" : "DISENGAGED");
            telemetry.addData("Blocker", blockerOpen ? "OPEN" : "CLOSED");
            telemetry.addData("CasterMode", backLeftCoasting);
            telemetry.addData("Exposure (ms)",
                    exposureControl != null
                            ? String.format("%d  [%d..%d]", exposureMs, exposureMinMs, exposureMaxMs)
                            : "camera not ready");
            telemetry.addData("Tag " + getTargetTagId(),
                    targetTag != null ? "SEEN" : "NOT SEEN");
            telemetry.addData("Tag distance (in)",
                    tagDistanceInches >= 0 ? String.format("%.1f", tagDistanceInches)
                            : "not measured");
            // The flywheel speed that will actually be applied: override (full)
            // wins; else the distance-chart value when a tag is visible; else
            // the G2 manual value. Voltage scaling is applied on top when running.
            String flywheelSource = flywheelOverrideActive ? " (override)"
                    : (tagVisibleThisLoop ? " (chart)" : " (manual)");
            telemetry.addData("Flywheel speed", "%.3f%s",
                    flywheelSpeedManual, flywheelSource);
            telemetry.addData("FW override (dpad_right / Y=auto)",
                    flywheelOverrideActive
                            ? (flywheelOverrideFar ? "FULL / 3.35s spin-up"
                            : "FULL / 1.5s spin-up")
                            : "off (chart/manual)");
            telemetry.addData("Light pos", "%.3f", lightPosition);
            telemetry.update();
        }

        // Clean up vision portal when opmode ends
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    // ============================================================
    //   FLYWHEEL DISTANCE CHART LOOKUP
    //   Linear interpolation of flywheelSpeedManual from the team chart.
    //   Distance is clamped to the charted range: nearer than the closest
    //   point uses that point's power; farther than the farthest point
    //   uses that point's power. (Callers only invoke this when distance
    //   <= FLYWHEEL_CHART_MAX_DISTANCE, so the far clamp is just a guard.)
    // ============================================================
    private double flywheelPowerForDistance(double distanceInches) {
        int n = FLYWHEEL_CHART_DISTANCE.length;

        // Find the min and max charted distance (the table is descending,
        // but don't assume — scan for the true bounds).
        int idxMin = 0, idxMax = 0;
        for (int i = 1; i < n; i++) {
            if (FLYWHEEL_CHART_DISTANCE[i] < FLYWHEEL_CHART_DISTANCE[idxMin]) idxMin = i;
            if (FLYWHEEL_CHART_DISTANCE[i] > FLYWHEEL_CHART_DISTANCE[idxMax]) idxMax = i;
        }

        // Clamp outside the charted range.
        if (distanceInches <= FLYWHEEL_CHART_DISTANCE[idxMin]) {
            return FLYWHEEL_CHART_POWER[idxMin];
        }
        if (distanceInches >= FLYWHEEL_CHART_DISTANCE[idxMax]) {
            return FLYWHEEL_CHART_POWER[idxMax];
        }

        // Find the charted pair that brackets this distance and interpolate.
        // The table order doesn't matter: we look for any i, j whose
        // distances straddle the query value.
        double bestLoDist = -Double.MAX_VALUE, bestLoPow = 0;
        double bestHiDist =  Double.MAX_VALUE, bestHiPow = 0;
        for (int i = 0; i < n; i++) {
            double d = FLYWHEEL_CHART_DISTANCE[i];
            if (d <= distanceInches && d > bestLoDist) {
                bestLoDist = d; bestLoPow = FLYWHEEL_CHART_POWER[i];
            }
            if (d >= distanceInches && d < bestHiDist) {
                bestHiDist = d; bestHiPow = FLYWHEEL_CHART_POWER[i];
            }
        }
        if (bestHiDist == bestLoDist) return bestLoPow; // exact charted hit
        double t = (distanceInches - bestLoDist) / (bestHiDist - bestLoDist);
        return bestLoPow + t * (bestHiPow - bestLoPow);
    }

    // ============================================================
    //   FAR-REGIME FLYWHEEL POWER  (tag >= 76.5 in)
    //   Linear model fitted to the measured far points
    //   (118 in -> 0.95, 122 in -> 0.97, 128 in -> 1.00), which lie on a
    //   straight line of slope FAR_SLOPE (+0.005 power per inch):
    //       power = FAR_BASE_POWER + FAR_SLOPE * (distance - FAR_BASE_DIST)
    //   The line is extrapolated across the whole far range. Coming nearer
    //   than ~108 in it would exceed full power, so the result is clamped
    //   to FAR_POWER_MAX (1.0). (Power is also floored at 0.0 as a guard.)
    // ============================================================
    private double flywheelPowerForDistanceFar(double distanceInches) {
        double power = FAR_BASE_POWER
                + FAR_SLOPE * (distanceInches - FAR_BASE_DIST);
        if (power > FAR_POWER_MAX) power = FAR_POWER_MAX;
        if (power < 0.0)           power = 0.0;
        return power;
    }

    // ============================================================
    //   REQUIRED FLYWHEEL SPIN-UP TIME (override- and distance-dependent)
    //   When the G1 dpad_right full-power override is active it dictates the
    //   spin-up time directly: 3.35 s in the "far" state, 1.5 s in the "near"
    //   state. Otherwise the blocker only opens after the flywheel has been
    //   running the distance-based time: far shots (tag at/over 76.5 in, or no
    //   tag at all) need 3.15 s; closer shots use 1.20 s.
    // ============================================================
    private double requiredSpinupSeconds() {
        if (flywheelOverrideActive) {
            return flywheelOverrideFar ? OVERRIDE_SPINUP_FAR : OVERRIDE_SPINUP_NEAR;
        }
        boolean far = (tagDistanceInches < 0)                       // no tag ever / not seen
                || (tagDistanceInches >= FLYWHEEL_CHART_MAX_DISTANCE); // 76.5 in or beyond
        return far ? BLOCKER_FLYWHEEL_SPINUP_FAR : BLOCKER_FLYWHEEL_SPINUP_NEAR;
    }

    // ============================================================
    //   BLOCKER-CLOSE CUT
    //   Hard-stops the intake and flywheel and clears all their run /
    //   ramp state, overriding the toggles. Called when the blocker
    //   returns to BLOCKED (auto-return or G1 B). Driver must re-press
    //   the triggers to restart either mechanism.
    // ============================================================
    private void cutIntakeAndFlywheel() {
        // Intake off
        intakeRunning     = false;
        intakeRampingDown = false;
        topIntake.setPower(0.0);
        bottomIntake.setPower(0.0);

        // Flywheel off — immediate hard stop, no graceful ramp.
        flywheelRunning     = false;
        flywheelRampingDown = false;
        leftFly.setPower(0.0);
        rightFly.setPower(0.0);
    }

    // ============================================================
    //   VISION INIT
    // ============================================================
    private void initializeVision() {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "turretCam"))
                .addProcessor(aprilTagProcessor)
                // 640x480 — native resolution. The SDK ships a real calibration
                // for this resolution, so AprilTag pose (range/bearing) is
                // accurate, which the distance-based flywheel chart depends on.
                .setCameraResolution(new android.util.Size(640, 480))
                // LiveView off — frees CPU/bandwidth, raises effective frame rate.
                .enableLiveView(false)
                // MJPEG lets the webcam stream at its highest supported frame
                // rate. Actual fps is set by the camera hardware's available
                // modes — there is no separate numeric fps cap to raise.
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        // Decimation 3 at 640x480 recovers much of the detection-rate gain that
        // the lower resolution was for, without sacrificing the calibration.
        // Higher decimation = faster detection, shorter effective range.
        aprilTagProcessor.setDecimation(3);
    }

    // ============================================================
    //   HARDWARE INIT
    // ============================================================
    private void initializeHardware() {
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive  = hardwareMap.get(DcMotor.class, "backRightDrive");

        frontLeftSteer  = hardwareMap.get(CRServo.class, "frontLeftSteer");
        frontRightSteer = hardwareMap.get(CRServo.class, "frontRightSteer");
        backLeftSteer   = hardwareMap.get(CRServo.class, "backLeftSteer");
        backRightSteer  = hardwareMap.get(CRServo.class, "backRightSteer");

        frontLeftEncoder  = hardwareMap.get(AnalogInput.class, "frontLeftEncoder");
        frontRightEncoder = hardwareMap.get(AnalogInput.class, "frontRightEncoder");
        backLeftEncoder   = hardwareMap.get(AnalogInput.class, "backLeftEncoder");
        backRightEncoder  = hardwareMap.get(AnalogInput.class, "backRightEncoder");

        // Intake motors as DcMotorEx for current sensing (stall detection).
        topIntake    = hardwareMap.get(DcMotorEx.class, "topIntake");
        bottomIntake = hardwareMap.get(DcMotorEx.class, "bottomIntake");
        leftFly      = hardwareMap.get(DcMotor.class, "leftFly");
        rightFly     = hardwareMap.get(DcMotor.class, "rightFly");

        topIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bottomIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftFly.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFly.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        topIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFly.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFly.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        blocker = hardwareMap.get(Servo.class, "blocker");
        pto     = hardwareMap.get(Servo.class, "pto");

        leftTurret  = hardwareMap.get(Servo.class, "leftTurret");
        rightTurret = hardwareMap.get(Servo.class, "rightTurret");

        light = hardwareMap.get(Servo.class, "light");

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.DOWN
                )
        );
        imu.initialize(parameters);
        imu.resetYaw();

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetMotors(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);

        headingOffset = 0.0;
    }

    // ============================================================
    //   MODULE HELPER  (matches justFieldSwerving — no voltage scaling,
    //   no STEER_MIN_POWER floor)
    // ============================================================
    private static class ModuleDebug {
        String name;
        double rawAngle, currentAngle, targetAngle, delta, servoPower, drivePower;
        boolean flipped;

        String toShortString() {
            return String.format(
                    "raw=%.2f adj=%.2f tgt=%.2f d=%.2f sp=%.2f drv=%.2f flip=%s",
                    rawAngle, currentAngle, targetAngle, delta, servoPower, drivePower, flipped
            );
        }
    }

    private ModuleDebug runModule(
            DcMotor driveMotor, CRServo steerServo, AnalogInput encoder,
            double encoderOffset, double speed, double targetAngle, String name
    ) {
        ModuleDebug dbg = new ModuleDebug();
        dbg.name = name;

        double rawAngle     = getRawAngle(encoder);
        double currentAngle = wrapAngle(rawAngle - encoderOffset);
        double delta        = wrapAngle(targetAngle - currentAngle);

        boolean flipped = false;
        if (Math.abs(delta) > Math.PI / 2) {
            delta = wrapAngle(delta + Math.PI);
            speed *= -1;
            flipped = true;
        }

        double servoPower = STEER_KP * delta;
        servoPower *= -1;

        // Final NaN guard before touching hardware: if anything upstream still
        // produced NaN, command zero rather than crash the OpMode.
        if (Double.isNaN(servoPower) || Math.abs(servoPower) < STEER_DEADBAND) servoPower = 0;
        servoPower = Math.max(-1, Math.min(1, servoPower));

        if (Double.isNaN(speed)) speed = 0;

        steerServo.setPower(servoPower);
        driveMotor.setPower(speed);

        dbg.rawAngle     = rawAngle;
        dbg.currentAngle = currentAngle;
        dbg.targetAngle  = targetAngle;
        dbg.delta        = delta;
        dbg.servoPower   = servoPower;
        dbg.drivePower   = speed;
        dbg.flipped      = flipped;
        return dbg;
    }

    // ============================================================
    //   UTILITY
    // ============================================================
    private double wrapAngle(double angle) {
        // NaN guard: an early/transient IMU or encoder read can return NaN.
        // Without this, the while-loops below skip (NaN comparisons are false)
        // and NaN propagates through the kinematics into a servo, crashing the
        // OpMode with "Illegal servo position NaN". Treat NaN as 0 for this loop.
        if (Double.isNaN(angle)) return 0.0;
        while (angle >  Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    /**
     * Wraps a servo position into [0, 1) treating the range as continuous:
     * going past 1 jumps to 0 and keeps increasing; going below 0 jumps to 1.
     * Used by the turret, which travels a finite 360deg across the 0..1 range.
     */
    private double wrapUnit(double pos) {
        if (Double.isNaN(pos)) return 0.5; // safe center if something went NaN
        while (pos >= 1.0) pos -= 1.0;
        while (pos <  0.0) pos += 1.0;
        return pos;
    }

    /**
     * Converts a raw turretPosition (0..1 servo value) into the angle the
     * CAMERA points, relative to the robot chassis, in radians. Used only for
     * the "Turret rel. angle" telemetry readout now that aiming is manual.
     */
    private double turretPosToRelAngle(double pos) {
        return wrapAngle((pos - 0.5) * 2.0 * Math.PI + TURRET_ANGLE_OFFSET);
    }

    private void resetMotors(DcMotor... motors) {
        for (DcMotor m : motors) m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private double getRawAngle(AnalogInput encoder) {
        return encoder.getVoltage() / 3.3 * (2 * Math.PI);
    }
}