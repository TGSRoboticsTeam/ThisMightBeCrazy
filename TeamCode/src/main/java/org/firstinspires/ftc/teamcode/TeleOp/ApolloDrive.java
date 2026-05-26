package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "ApolloDrive", group = "Swerve")
public class ApolloDrive extends LinearOpMode {

    // --- 1. HARDWARE DECLARATIONS ---
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private CRServo frontLeftSteer, frontRightSteer, backLeftSteer, backRightSteer;
    private AnalogInput frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder;
    private IMU imu;
    private VoltageSensor voltageSensor;

    // All REV hubs, used for bulk-cached sensor reads. With MANUAL bulk
    // caching, clearBulkCache() is called once per loop and every sensor
    // read that loop is served from a single bus transaction instead of
    // one round trip per device — large loop-time savings.
    private List<LynxModule> allHubs;

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

    // --- TURRET SERVOS ---
    private Servo leftTurret, rightTurret;
    private double turretPosition = 0.5;
    final double TURRET_SPEED    = 0.008; // position units per loop tick (legacy/reference)
    final double TURRET_DEADBAND = 0.05;

    // --- TURRET MANUAL MODE (G2 left stick X) ---
    // Three speeds: LT held = fast, RT held = slow, neither = normal.
    // If both triggers are held, LT (fast) wins.
    final double TURRET_SPEED_MANUAL = 0.016; // 2x — normal manual speed
    final double TURRET_SPEED_SLOW   = 0.008; // RT held — fine control
    final double TURRET_SPEED_FAST   = 0.032; // LT held — fast slew
    final double TURRET_RT_THRESHOLD = 0.5;
    final double TURRET_LT_THRESHOLD = 0.5;

    // --- TURRET AUTOPILOT (G2 RB + dpad_up toggles) ---
    private boolean turretAutopilot = false;            // false = manual, true = autopilot
    private boolean turretAutopilotTogglePrev = false;  // rising-edge tracker for RB+dpad_up

    // --- TURRET ODOMETRY TRACKING (absolute field coordinates) ---
    // The field origin (0, 0) is the BACK-LEFT corner. The robot is placed
    // there, facing +X, and G1 dpad-up zeroes the Pinpoint — from then on
    // odo.getPosX/Y report absolute field inches and odo.getHeading is the
    // field heading.
    //
    // The goal is at a FIXED, known field coordinate (constants below), so
    // the turret aims by pure geometry: atan2(goal - robotPose). There is
    // no derived/stored tag position and no camera input to the aim — the
    // camera now only drives the light servo and reports tag distance.
    // Because the goal is a constant, the camera and odometry can never
    // disagree about it, which removes the old "jump on re-acquire" bug.
    private GoBildaPinpointDriver odo;

    // Goal/tag field position, measured from the back-left corner origin.
    final double GOAL_FIELD_X = 128; // inches, +X = to the right
    final double GOAL_FIELD_Y = -128;  // inches, +Y = up the field

    // True once G1 dpad-up has zeroed the Pinpoint at the corner. Until
    // then the absolute pose is unknown and the turret should not auto-aim.
    private boolean fieldZeroed = false;

    // ODO AIM SIGN: flip to -1.0 if turret tracking aims the wrong way.
    final double TURRET_ODO_SIGN = -1.0;

    // --- TURRET ANGLE CALIBRATION ---
    // There are THREE distinct angles in the tracking system, and they must
    // not be confused:
    //   1. turretRelAngle  — where the CAMERA points, relative to the robot
    //                        chassis. Derived from turretPosition.
    //   2. robotHeading    — the robot's heading on the field (odo).
    //   3. fieldAngleToGoal — direction from robot to goal, in field frame.
    //
    // turretPosition is a raw servo value. The turret-relative angle is:
    //     turretRelAngle = (turretPosition - 0.5) * 2*PI + TURRET_ANGLE_OFFSET
    // TURRET_ANGLE_OFFSET (radians) corrects for the camera NOT pointing
    // straight robot-forward when turretPosition == 0.5. If the turret
    // tracking snaps a fixed amount off (e.g. 90 deg) and holds there, set
    // this to that offset. 90 deg = +/- Math.PI/2; 180 deg = Math.PI.
    final double TURRET_ANGLE_OFFSET = 0.0;

    // --- TURRET TRACKING tuning ---
    // Max turret position change per loop while auto-tracking. Slew limit so
    // a fast robot move can't snap the turret. If it lags behind fast robot
    // motion, raise it (max useful ~0.10); if it jitters, lower it.
    final double TURRET_ODO_MAX_STEP = 0.060;
    // Last camera-measured straight-line distance to the tag (inches).
    // Camera-only readout — does NOT feed the aim. -1 until first seen.
    private double tagDistanceInches = -1.0;

    // Pinpoint config (same hardware/config as the auto and pinpointTest).
    final GoBildaPinpointDriver.EncoderDirection ODO_X_DIR =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    final GoBildaPinpointDriver.EncoderDirection ODO_Y_DIR =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;
    final double ODO_X_OFFSET_MM =  41.9999922;
    final double ODO_Y_OFFSET_MM = -148.3535768;

    // --- VISION (AprilTag) ---
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    final int TARGET_TAG_ID = 24;

    // ============================================================
    //   SERVO POSITIONS  <-- SET YOUR VALUES HERE
    // ============================================================
    final double BLOCKER_BLOCKED_POSITION = 0.15;
    final double BLOCKER_LAUNCH_POSITION  = 0.45;

    // Blocker auto-return: after A moves the blocker to LAUNCH, it returns
    // to BLOCKED this many seconds later. Pressing B cancels the timer.
    final double BLOCKER_AUTO_RETURN_SECONDS = 3.0;

    // Light servo positions (AprilTag status indicator)
    final double LIGHT_TAG_NOT_SEEN  = 0.278;  // no tag detected
    final double LIGHT_TAG_LEFT      = 0.388;  // tag detected, off to the LEFT
    final double LIGHT_TAG_RIGHT     = 0.555;  // tag detected, off to the RIGHT
    final double LIGHT_TAG_CENTERED  = 0.480;  // tag detected and centered (<2 deg bearing)
    final double LIGHT_CENTERED_BEARING_DEG = 2.0;
    final double LIGHT_PTO_ENGAGED   = 0.0;    // PTO engaged — overrides tag states

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
    final double STEER_MIN_POWER = 0.08;

    // --- 5. SPEED CONTROL CONSTANTS ---
    final double MAX_SPEED_GLOBAL    = 0.8;
    final double MAX_SPEED_SLOW_MODE = 0.2;

    // --- 6. FLYWHEEL / INTAKE CONSTANTS ---
    final double MOTOR_COAST_RAMP_SECONDS = 0.5;

    // --- 6b. INTAKE SPEED PRESETS ---
    final double INTAKE_SPEED_HIGH = 0.90; // dpad_up
    final double INTAKE_SPEED_LOW  = 0.60; // dpad_down
    private double intakeSpeedTarget = INTAKE_SPEED_HIGH; // default

    // --- 6c. FLYWHEEL MANUAL SPEED (G2 dpad up/down; RT = fine step) ---
    final double FLYWHEEL_SPEED_STEP_COARSE = 0.05;   // per-tap, RT not held
    final double FLYWHEEL_SPEED_STEP_FINE   = 0.01;   // per-tap, RT held
    private double flywheelSpeedManual = 1.0;         // 0.0 .. 1.0, scaled by voltageFactor
    private boolean g2DpadUpPrev   = false;
    private boolean g2DpadDownPrev = false;

    // --- 6d. INTAKE REVERSE-HOLD (G1 dpad_left) ---
    // While dpad_left is held (PTO disengaged), run intake in reverse at this power.
    // The intake's normal toggle state is preserved and resumes on release.
    final double INTAKE_REVERSE_HOLD_POWER = 0.50;

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

    // Blocker auto-return timer: armed by A (launch), cancelled by B.
    private boolean blockerAutoReturnArmed = false;
    private ElapsedTime blockerAutoReturnTimer = new ElapsedTime();

    // Blocker-launch sequence: G1 A registers a pending-open request. The
    // blocker only actually opens once the flywheel has been running
    // continuously for BLOCKER_FLYWHEEL_SPINUP_SECONDS. After it opens, the
    // intake is forced to full power (the toggle speed) once
    // BLOCKER_LAUNCH_INTAKE_DELAY has elapsed. Closing the blocker cuts
    // intake AND flywheel.
    final double BLOCKER_FLYWHEEL_SPINUP_SECONDS = 1.2; // flywheel must be on this long
    final double BLOCKER_LAUNCH_INTAKE_DELAY     = 0.4; // intake delay after blocker opens
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
    private boolean dpadDownPrev = false;

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

            // MANUAL bulk caching: discard last loop's sensor snapshot. The
            // next sensor read triggers ONE bus fetch of every device on the
            // hub; all reads this loop are then served from that snapshot.
            // Must run before any sensor read — if omitted, sensors freeze.
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            if (!servosInitialized) {
                blocker.setPosition(BLOCKER_BLOCKED_POSITION);
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
            //   TURRET AUTOPILOT TOGGLE  (G2 RB + dpad_up)
            //   Rising edge toggles between autopilot and manual turret modes.
            //   Checked before the flywheel adjust so RB+dpad_up does NOT
            //   also nudge the flywheel speed.
            //   In autopilot the turret aims at the fixed goal by pure
            //   odometry geometry (requires the field to have been zeroed
            //   first via G1 dpad-up).
            // ============================================================
            boolean g2RbHeld = gamepad2.right_bumper;
            boolean turretAutopilotToggleNow = g2RbHeld && gamepad2.dpad_up;
            if (turretAutopilotToggleNow && !turretAutopilotTogglePrev) {
                turretAutopilot = !turretAutopilot;
            }
            turretAutopilotTogglePrev = turretAutopilotToggleNow;

            // ============================================================
            //   FLYWHEEL MANUAL SPEED ADJUST  (G2 dpad up/down)
            //   dpad adjusts ONLY when RB is not held (RB+dpad is the turret
            //   autopilot toggle). Step = 0.05 normally, 0.01 with RT held.
            //   Rising-edge: each tap nudges once, clamped to [0, 1].
            // ============================================================
            boolean g2RtHeld      = gamepad2.right_trigger > TURRET_RT_THRESHOLD;
            double  flywheelStep  = g2RtHeld ? FLYWHEEL_SPEED_STEP_FINE
                    : FLYWHEEL_SPEED_STEP_COARSE;
            boolean g2DpadUpNow   = gamepad2.dpad_up;
            boolean g2DpadDownNow = gamepad2.dpad_down;

            if (g2DpadUpNow && !g2DpadUpPrev && !g2RbHeld) {
                flywheelSpeedManual += flywheelStep;
            }
            if (g2DpadDownNow && !g2DpadDownPrev && !g2RbHeld) {
                flywheelSpeedManual -= flywheelStep;
            }
            flywheelSpeedManual = Math.max(0.0, Math.min(1.0, flywheelSpeedManual));

            g2DpadUpPrev   = g2DpadUpNow;
            g2DpadDownPrev = g2DpadDownNow;

            // ============================================================
            //   INTAKE SPEED PRESETS  (dpad_up = 90%, dpad_down = 60%)
            //   dpad_up also resets field heading — only when RB is NOT held.
            //   dpad_up ALSO zeroes the Pinpoint: the robot must be at the
            //   back-left field corner, facing +X. This establishes absolute
            //   field coordinates for the turret tracker.
            // ============================================================
            boolean dpadUpNow   = gamepad1.dpad_up;
            boolean dpadDownNow = gamepad1.dpad_down;

            if (dpadUpNow && !dpadUpPrev && !rbHeld) {
                intakeSpeedTarget = INTAKE_SPEED_HIGH;
                imu.resetYaw();
                headingOffset = 0.0;
                // Zero the Pinpoint at the field corner (0,0), heading 0 (+X).
                // From here, odo reports absolute field coordinates and the
                // turret can aim at the fixed goal by pure geometry.
                odo.resetPosAndIMU();
                fieldZeroed = true;
            }
            if (dpadDownNow && !dpadDownPrev && !rbHeld) {
                intakeSpeedTarget = INTAKE_SPEED_LOW;
            }
            dpadUpPrev   = dpadUpNow;
            dpadDownPrev = dpadDownNow;

            // ============================================================
            //   INTAKE MOTOR CONTROL
            //
            //   PTO ACTIVE:
            //     Both triggers held simultaneously → intakes run backward
            //     Anything else → intakes stop
            //     All other mechanisms are frozen (no trigger/button changes
            //     processed for flywheel, blocker, turret)
            //     dpad_left reverse-hold and stall detection are NOT active here.
            //
            //   PTO INACTIVE:
            //     Normal right-trigger toggle behavior,
            //     plus dpad_left reverse-hold and current-based stall detection.
            // ============================================================
            double intakePower = 0.0;          // forward-positive command (pre-sign)
            boolean intakeReverseHoldActive = false;

            if (ptoEngaged) {
                boolean bothTriggersHeld = (gamepad1.left_trigger > 0.5) && (gamepad1.right_trigger > 0.5);
                intakePower = bothTriggersHeld ? -1.0 : 0.0;

                topIntake.setPower(intakePower);
                bottomIntake.setPower(intakePower);

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
                //       run for BLOCKER_FLYWHEEL_SPINUP_SECONDS.
                //     - flywheels ON  -> register the launch; blocker opens once
                //       the 0.7s spin-up gate is met (immediately if already met).
                //   Either way, a single A press takes it all the way to launch.
                //   The left trigger still toggles flywheels on/off independently.
                //   G1 B moves to BLOCKED, cancels everything, cuts mechanisms.
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
                    // the 0.7s spin-up below, so one press goes cold -> launch.
                    blockerOpenPending = true;
                }

                // Open the blocker once the flywheel spin-up gate is satisfied:
                // flywheel must be running NOW and have been for >= 0.7s.
                if (blockerOpenPending
                        && flywheelRunning
                        && flywheelOnTimer.seconds() >= BLOCKER_FLYWHEEL_SPINUP_SECONDS) {
                    blocker.setPosition(BLOCKER_LAUNCH_POSITION);
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
                    // B cancels a pending auto-return and any pending open.
                    blockerAutoReturnArmed = false;
                    blockerOpenPending     = false;
                    // Blocker closed by B — end launch mode and cut intake+flywheel.
                    if (blockerLaunchMode) {
                        blockerLaunchMode = false;
                        cutIntakeAndFlywheel();
                    }
                }

                // Auto-return: once 2s have elapsed since the blocker opened,
                // send it back to BLOCKED and disarm. This also ends launch
                // mode and cuts intake + flywheel.
                if (blockerAutoReturnArmed
                        && blockerAutoReturnTimer.seconds() >= BLOCKER_AUTO_RETURN_SECONDS) {
                    blocker.setPosition(BLOCKER_BLOCKED_POSITION);
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
            //   FIELD-CENTRIC SWERVE DRIVE
            // ============================================================
            double fieldY = -gamepad1.left_stick_y * speedMultiplier;
            double fieldX =  gamepad1.left_stick_x * speedMultiplier;
            double rot    =  gamepad1.right_stick_x * speedMultiplier;

            double rawHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rawPitch   = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.RADIANS);
            double rawRoll    = imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.RADIANS);
            double botHeading = wrapAngle(rawHeading - headingOffset);

            double robotX = fieldX * Math.cos(-botHeading) - fieldY * Math.sin(-botHeading);
            double robotY = fieldX * Math.sin(-botHeading) + fieldY * Math.cos(-botHeading);

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

            boolean driverActiveForSwerve =
                    Math.abs(gamepad1.left_stick_x)  > DRIVE_DEADBAND ||
                            Math.abs(gamepad1.left_stick_y)  > DRIVE_DEADBAND ||
                            Math.abs(gamepad1.right_stick_x) > DRIVE_DEADBAND;

            if (driverActiveForSwerve) {
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

            double voltage       = voltageSensor.getVoltage();
            double voltageFactor = (voltage > 0) ? Math.min(12.0 / voltage, 1.0) : 1.0;

            ModuleDebug fl = runModule(frontLeftDrive,  frontLeftSteer,  frontLeftEncoder,  FRONT_LEFT_OFFSET,  speedFrontLeft,  targetAngleFL, "FL", voltageFactor);
            ModuleDebug fr = runModule(frontRightDrive, frontRightSteer, frontRightEncoder, FRONT_RIGHT_OFFSET, speedFrontRight, targetAngleFR, "FR", voltageFactor);
            ModuleDebug bl = runModule(backLeftDrive,   backLeftSteer,   backLeftEncoder,   BACK_LEFT_OFFSET,   speedBackLeft,   targetAngleBL, "BL", voltageFactor);
            ModuleDebug br = runModule(backRightDrive,  backRightSteer,  backRightEncoder,  BACK_RIGHT_OFFSET,  speedBackRight,  targetAngleBR, "BR", voltageFactor);

            // ============================================================
            //   APRILTAG DETECTION (Tag ID 24)
            // ============================================================
            AprilTagDetection targetTag = null;
            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.id == TARGET_TAG_ID) {
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
            //   The camera also records tagDistanceInches for telemetry.
            //   It does NOT feed the turret aim — the turret aims purely
            //   by odometry geometry against the fixed goal coordinate.
            // ============================================================
            double lightPosition = LIGHT_TAG_NOT_SEEN;
            boolean tagCentered = false;
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
                // Record camera-measured distance (telemetry only).
                tagDistanceInches = targetTag.ftcPose.range;
            }
            light.setPosition(lightPosition);

            // ============================================================
            //   TURRET CONTROL  (autopilot or manual)
            //   Frozen entirely while PTO is engaged (same as all other
            //   mechanisms). Mode is toggled by G2 RB + dpad_up above.
            //
            //   AUTOPILOT — absolute-coordinate aiming:
            //     The goal is at a FIXED, known field coordinate. The robot's
            //     absolute field pose comes from the Pinpoint (zeroed at the
            //     corner via G1 dpad-up). The turret aims by pure geometry:
            //         fieldAngle = atan2(goalY - robotY, goalX - robotX)
            //     converted to a robot-relative camera angle, then to a servo
            //     position. This tracks the goal through ANY robot motion —
            //     translation and rotation alike — with no camera input and
            //     therefore no "jump on re-acquire".
            //     If the field has not been zeroed yet, the turret holds.
            //
            //   MANUAL: G2 left stick X (LT fast / RT slow). Snap presets
            //     X -> 0.25, Y -> 0.5, B -> 0.75, A -> nearest of 0/1.
            //
            //   Position wraps 0<->1 (turret travels a full 360deg; the
            //   servo range 0..1 is continuous; 0.5 = forward).
            // ============================================================
            if (!ptoEngaged) {
                if (turretAutopilot) {
                    // ---------- AUTOPILOT (absolute-coordinate aiming) ----------
                    if (fieldZeroed) {
                        odo.update();
                        double rx = odo.getPosX(DistanceUnit.INCH);
                        double ry = odo.getPosY(DistanceUnit.INCH);
                        double rHeading = odo.getHeading(AngleUnit.RADIANS);

                        // Field-frame angle from the robot to the fixed goal.
                        // As the robot TRANSLATES, rx/ry change so this
                        // changes. As the robot ROTATES, subtracting rHeading
                        // counter-rotates the aim. One expression, both cases.
                        double fieldAngle = Math.atan2(GOAL_FIELD_Y - ry,
                                GOAL_FIELD_X - rx);
                        double robotRelativeAngle = wrapAngle(fieldAngle - rHeading);

                        // TURRET_ODO_SIGN flips turret rotation direction only.
                        // relAngleToTurretPos applies TURRET_ANGLE_OFFSET so a
                        // miscalibrated "forward" is corrected in one place.
                        double aimRelAngle = wrapAngle(TURRET_ODO_SIGN * robotRelativeAngle);
                        double targetPos = relAngleToTurretPos(aimRelAngle);

                        // Slew-limited move toward the target, shortest path
                        // around the 0<->1 wrap.
                        double err = targetPos - turretPosition;
                        if (err >  0.5) err -= 1.0;
                        if (err < -0.5) err += 1.0;
                        double step = Math.max(-TURRET_ODO_MAX_STEP,
                                Math.min(TURRET_ODO_MAX_STEP, err));
                        turretPosition = wrapUnit(turretPosition + step);
                    }
                    // else: field not yet zeroed — hold turret position.

                } else {
                    // ---------- MANUAL ----------
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
                        // Snap to whichever end (0 or 1) the turret is
                        // currently closer to. Because the range wraps, 0
                        // and 1 are the same physical position — this just
                        // picks the neater servo value for that end.
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
                }

                leftTurret.setPosition(turretPosition);
                rightTurret.setPosition(turretPosition);
            }

            // ============================================================
            //   TELEMETRY  (PTO, turret mode, tag visibility, field pose)
            // ============================================================
            telemetry.addData("PTO", ptoEngaged ? "ENGAGED" : "DISENGAGED");
            String turretMode;
            if (!turretAutopilot) {
                turretMode = "MANUAL";
            } else if (!fieldZeroed) {
                turretMode = "AUTO: WAITING (press G1 dpad-up to zero)";
            } else {
                turretMode = "AUTO: TRACKING GOAL";
            }

            double xIn = odo.getPosX(DistanceUnit.INCH);
            double yIn = odo.getPosY(DistanceUnit.INCH);
            double headingDeg = odo.getHeading(AngleUnit.DEGREES);
            // The two angles the tracking system depends on:
            //   turretRelDeg — where the camera points, relative to the robot.
            //   headingDeg   — where the robot points, on the field.
            double turretRelDeg = Math.toDegrees(turretPosToRelAngle(turretPosition));
            telemetry.addData("Turret", turretMode);
            telemetry.addData("Field zeroed", fieldZeroed ? "YES" : "NO");
            telemetry.addData("Tag " + TARGET_TAG_ID,
                    targetTag != null ? "VISIBLE" : "NOT SEEN");
            telemetry.addData("Tag distance (in)",
                    tagDistanceInches >= 0 ? String.format("%.1f", tagDistanceInches)
                            : "not measured");
            telemetry.addLine("=== LIVE READINGS ===");
            telemetry.addData("X (in)", "%.3f", xIn);
            telemetry.addData("Y (in)", "%.3f", yIn);
            telemetry.addData("Robot heading (deg)", "%.3f", headingDeg);
            telemetry.addData("Turret rel. angle (deg)", "%.1f", turretRelDeg);
            telemetry.addData("Turret pos (0-1)", "%.3f", turretPosition);
            telemetry.update();
        }

        // Clean up vision portal when opmode ends
        if (visionPortal != null) {
            visionPortal.close();
        }
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
                // accurate, which the odometry handoff depends on.
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

        // Pinpoint odometry — used by the turret odometry-tracking system.
        // Direction config matches the calibrated values:
        //   X = FORWARD, Y = REVERSED.
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(ODO_X_OFFSET_MM, ODO_Y_OFFSET_MM, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(ODO_X_DIR, ODO_Y_DIR);
        odo.resetPosAndIMU();

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

        // Bulk caching: read every sensor on a hub in one bus transaction.
        // MANUAL mode means clearBulkCache() must be called once at the top
        // of every loop iteration (see runOpMode) — every sensor read after
        // that, until the next clear, is served from one cached snapshot.
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        headingOffset = 0.0;
    }

    // ============================================================
    //   MODULE HELPER
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
            double encoderOffset, double speed, double targetAngle, String name,
            double voltageFactor
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

        double servoPower = STEER_KP * delta * -1;

        if (Math.abs(servoPower) > STEER_DEADBAND) {
            if (servoPower > 0 && servoPower < STEER_MIN_POWER)  servoPower =  STEER_MIN_POWER;
            if (servoPower < 0 && servoPower > -STEER_MIN_POWER) servoPower = -STEER_MIN_POWER;
        } else {
            servoPower = 0;
        }

        servoPower = Math.max(-1, Math.min(1, servoPower));

        double normalisedSpeed = speed * voltageFactor;
        normalisedSpeed = Math.max(-1, Math.min(1, normalisedSpeed));

        steerServo.setPower(servoPower);
        driveMotor.setPower(normalisedSpeed);

        dbg.rawAngle     = rawAngle;
        dbg.currentAngle = currentAngle;
        dbg.targetAngle  = targetAngle;
        dbg.delta        = delta;
        dbg.servoPower   = servoPower;
        dbg.drivePower   = normalisedSpeed;
        dbg.flipped      = flipped;
        return dbg;
    }

    // ============================================================
    //   UTILITY
    // ============================================================
    private double wrapAngle(double angle) {
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
        while (pos >= 1.0) pos -= 1.0;
        while (pos <  0.0) pos += 1.0;
        return pos;
    }

    /**
     * Converts a raw turretPosition (0..1 servo value) into the angle the
     * CAMERA points, relative to the robot chassis, in radians.
     * Applies TURRET_ANGLE_OFFSET so that a miscalibrated "forward" is
     * corrected in exactly one place.
     */
    private double turretPosToRelAngle(double pos) {
        return wrapAngle((pos - 0.5) * 2.0 * Math.PI + TURRET_ANGLE_OFFSET);
    }

    /**
     * Inverse of turretPosToRelAngle: given a desired camera angle relative
     * to the robot (radians), returns the turretPosition (0..1) that aims
     * the turret there. Used by the aim code to command the turret.
     */
    private double relAngleToTurretPos(double relAngle) {
        return wrapUnit(0.5 + (relAngle - TURRET_ANGLE_OFFSET) / (2.0 * Math.PI));
    }

    private void resetMotors(DcMotor... motors) {
        for (DcMotor m : motors) m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private double getRawAngle(AnalogInput encoder) {
        return encoder.getVoltage() / 3.3 * (2 * Math.PI);
    }
}