package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// ===== Vision / AprilTag =====
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;

@TeleOp(name = "blueFries", group = "Swerve")
public class blueFries extends LinearOpMode {

    /* ===================== DRIVE HARDWARE ===================== */
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private CRServo frontLeftSteer, frontRightSteer, backLeftSteer, backRightSteer;
    private AnalogInput frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder;
    private IMU imu;
    private VoltageSensor voltageSensor;

    /* ===================== MECHANISMS ===================== */
    private DcMotor frontIntake, backIntake;
    private DcMotor leftFly, rightFly;

    private Servo turretRotation1, turretRotation2;
    private Servo trigger;
    private Servo adjuster;

    // >>> LIGHT SERVO (config name: "lights") <<<
    private Servo light;

    private double turretPosition = 0.5;

    /* ===================== LIGHT SERVO POSITIONS ===================== */
    private static final double LIGHT_NO_TAG     = 0.0;
    private static final double LIGHT_TAG_SEEN   = 0.388;
    private static final double LIGHT_TAG_LOCKED = 0.5;
    private static final double LIGHT_LOCK_DEG   = 3.0;

    /* ===================== SIDE SORT SERVO ===================== */
    private Servo sideSort;
    private static final double SIDE_SORT_CENTERED = 0.4;
    private static final double SIDE_SORT_STOWED   = 0.955;
    private double sideSortPos = SIDE_SORT_CENTERED;

    // Manual side-sort jog (GP2 dpad left/right) when NOT launching.
    private static final double SIDE_SORT_JOG_STEP = 0.02;

    // Extra time for side sorter to physically slide
    private static final long SIDE_SORT_SLIDE_MS = 300;                 // your +300ms
    private static final long SIDE_SORT_SETTLE_BEFORE_FEED_MS = 250;    // extra settle
    private static final long SIDE_SORT_WAIT_CENTER_CLEAR_MS = 900;     // wait for center to clear after stow
    private static final long SIDE_SORT_WAIT_CENTER_FILL_MS  = 1100;    // wait for center to refill on return

    /* ===================== MULTIPLIER (AUTO FROM CAMERA) ===================== */
    private double multiplier = 1.0;

    /* ===================== SENSORS (BALL DISTANCE + COLOR) ===================== */
    private NormalizedColorSensor frontColor, centerColor, backColor;
    private DistanceSensor frontDist, centerDist, backDist;

    /* ===================== VOLTAGE COMP (FLYWHEEL) ===================== */
    private static final double REF_VOLTAGE = 12.0;

    /* ===================== VOLTAGE COMP (INTAKES) ===================== */
    private static final double INTAKE_REF_VOLTAGE = 9.0;

    private double intakeVoltageScale(double vbat) {
        if (Double.isNaN(vbat) || Double.isInfinite(vbat) || vbat <= 0.5) return 1.0;
        return clamp(INTAKE_REF_VOLTAGE / vbat, 0.0, 1.0);
    }

    private double intakeCommand(double desiredPower, double vbat) {
        return clamp(desiredPower * intakeVoltageScale(vbat), -1.0, 1.0);
    }

    /* ===================== SWERVE CONSTANTS ===================== */
    final double TRACK_WIDTH = 17.258;
    final double WHEELBASE   = 13.544;
    final double R = Math.hypot(TRACK_WIDTH, WHEELBASE);

    final double FRONT_LEFT_OFFSET  = 1.34;
    final double FRONT_RIGHT_OFFSET = 3.161;
    final double BACK_LEFT_OFFSET   = 1.589;
    final double BACK_RIGHT_OFFSET  = 1.237;

    final double STEER_KP = 0.8;
    final double DRIVE_DEADBAND = 0.05;
    final double STEER_DEADBAND = 0.08;

    final double MAX_SPEED_FAST = 0.8;
    final double MAX_SPEED_SLOW = 0.4;

    /* ===================== WHEEL PLANTING (X SNAP) ===================== */
    final int FRAMES_TO_PLANT_WHEELS = 5;
    private int framesSinceLastMoved = 0;

    /* ===================== TURRET (WORLD STABILIZATION + MANUAL) ===================== */
    private static final double TURRET_RANGE_DEG = 270.0;
    private static final double MIN_TURRET_ROTATION = 0.0;
    private static final double MAX_TURRET_ROTATION = 1.0;

    private static final double TURRET_INPUT_DEADBAND = 0.05;
    private static final double MANUAL_TURRET_DPS_FAST = 220.0;
    private static final double MANUAL_TURRET_DPS_SLOW = 75.0;
    private static final double TURRET_SMOOTHING = 0.25;

    private double turretWorldDeg = 0.0;
    private double turretDeg = 135.0;
    private double turretSmoothedInput = 0.0;
    private boolean turretCenterPrev = false;

    /* ===================== ADJUSTER LIMITS ===================== */
    final double ADJUSTER_MIN = 0.0;
    final double ADJUSTER_MAX = 0.75;
    private double adjusterPos = 0.4482;

    /* ===================== INTAKES ===================== */
    final double INTAKE_BASE = 0.90;

    final double INTAKE_SLOW_RATIO = 0.30;
    final double FAST_AFTER_SLOW_RATIO = 0.70;

    // 7% slower than previous “center occupied” regime
    final double FAST_AFTER_SLOW_AND_CENTER_RATIO = 0.625 * 0.93;

    final double MANUAL_FAST = INTAKE_BASE;
    final double MANUAL_SLOW = INTAKE_BASE * INTAKE_SLOW_RATIO;

    final double FAST_AFTER_SLOW = INTAKE_BASE * FAST_AFTER_SLOW_RATIO;
    final double FAST_AFTER_SLOW_AND_CENTER = INTAKE_BASE * FAST_AFTER_SLOW_AND_CENTER_RATIO;

    private boolean isIntakeOn = false;
    private boolean intakeTogglePrev = false;

    private boolean dpadUpPrev = false;

    // Mode 1: IN  -> front FAST, back SLOW
    // Mode 2: OUT -> back  FAST, front SLOW
    private boolean intakeModeOne = true;
    private boolean intakeModePrev = false;

    /* ===================== LAUNCH FEEDING ===================== */
    private static final double FRONT_FEED_POWER = 0.65;

    // Slow back feed by 10% during launch (your request)
    private static final double BACK_FEED_POWER = 0.375; // 10% slower

    private static final long FEED_TO_CENTER_MS = 450;

    /* ===================== OUTWARD BURP ===================== */
    private static final double SPINUP_OUTWARD_POWER = 0.4;
    private static final long SPINUP_OUTWARD_MS = 30;
    private static final long SHORT_BURP_MS = 20; // “burp 1/3 as long”

    /* ===================== TRIGGER SERVO ===================== */
    final double TRIGGER_FIRE = 0.0;
    final double TRIGGER_HOME = 0.225;

    final long LAUNCH_TRIGGER_HOLD_MS = 300;
    final long TRIGGER_RESET_WAIT_MS = 125;
    final long RETRY_EXTRA_WAIT_MS = 350;

    /* ===================== LAUNCH TIMING ===================== */
    final long FLYWHEEL_SPINUP_MS = 1000;
    final long FLYWHEEL_SPINDOWN_MS = 300;

    /* ===================== DISTANCE BALL DETECTION (HYSTERESIS) ===================== */
    final double FRONT_ON_CM  = 4.0, FRONT_OFF_CM  = 5.0;
    final double CENTER_ON_CM = 4.0, CENTER_OFF_CM = 5.0;
    final double BACK_ON_CM   = 4.0, BACK_OFF_CM   = 5.0;

    private boolean frontHasBall = false;
    private boolean centerHasBall = false;
    private boolean backHasBall = false;

    /* ===================== RETRY CONTROL ===================== */
    private static final int MAX_RETRIES_PER_SHOT = 3;
    private int shotRetryCount = 0;

    /* ===================== FULL-STACK LATCH ===================== */
    private boolean fullStackLatched = false;

    /* ===================== NORMAL (NON-SORTING) SNAPSHOT PLANS ===================== */
    private boolean planCenterShot = false;
    private boolean planFrontShot  = false;
    private boolean planBackShot   = false;

    /* ===================== FLYWHEEL PRE-SPIN TOGGLE (GP1 LEFT TRIGGER TAP) ===================== */
    private boolean preSpinLatched = false;
    private boolean ltPrev = false;
    private long preSpinStartMs = 0;

    private static final boolean AUTO_PRESPIN_WHEN_FULL = true;

    /* ===================== SPINUP REMAINING ===================== */
    private long spinupRemainingMs = 0;

    /* ===================== COLOR CLASSIFICATION ===================== */
    private enum BallColor { EMPTY, PURPLE, GREEN, UNKNOWN }
    private enum Slot { FRONT, CENTER, BACK }

    // Keep existing measurement approach; UNKNOWN assumed GREEN if present
    private static final double GREEN_RATIO = 1.35;
    private static final double PURPLE_RATIO = 1.12;
    private static final double GREEN_B_WEIGHT = 0.25;
    private static final double PURPLE_B_WEIGHT = 1.25;

    private double lastFrontGScore = 0, lastFrontPScore = 0;
    private double lastCenterGScore = 0, lastCenterPScore = 0;
    private double lastBackGScore = 0, lastBackPScore = 0;

    private BallColor frontBallColor = BallColor.EMPTY;
    private BallColor centerBallColor = BallColor.EMPTY;
    private BallColor backBallColor = BallColor.EMPTY;

    /* ===================== SORTING MODE (GP2 X) ===================== */
    private boolean sortingMode = false;
    private boolean gp2XPrev = false;

    /* ===================== PATTERN TAG MEMORY (21/22/23) ===================== */
    private int storedPatternId = 0;        // 0 = unknown
    private boolean patternLocked = false;  // once true, keep for match

    /* ===================== SORTING SNAPSHOT (taken at GP1 A) ===================== */
    private int snapPatternId = 0;
    private boolean snapF = false, snapC = false, snapB = false;
    private BallColor snapFC = BallColor.EMPTY, snapCC = BallColor.EMPTY, snapBC = BallColor.EMPTY;

    /* ===================== SORTED CASE PLAN ===================== */
    private enum SortedCase {
        NONE,

        // Pattern 23
        P23_GREEN_CENTER,
        P23_GREEN_BACK,
        P23_GREEN_FRONT,

        // Pattern 22
        P22_GREEN_CENTER,
        P22_GREEN_BACK,
        P22_GREEN_FRONT,

        // Pattern 21
        P21_GREEN_CENTER,
        P21_GREEN_FRONT,
        P21_GREEN_BACK
    }

    private SortedCase sortedCase = SortedCase.NONE;

    private enum SortedStep {
        // Stow center ball to side (move sideSort to STOWED) and wait for center to clear
        STOW_CENTER_TO_SIDE,

        // Fire whatever is currently in center (requires center has ball)
        FIRE_CENTER,

        // Feed a ball from FRONT/BACK into center then fire it
        FEED_FRONT_THEN_FIRE,
        FEED_BACK_THEN_FIRE,

        // Bring stored side ball back to center (move sideSort CENTERED), wait for center to fill, then fire
        RETURN_SIDE_TO_CENTER_THEN_FIRE
    }

    private SortedStep[] sortedSteps = new SortedStep[0];
    private int sortedStepIndex = 0;

    /* ===================== LAUNCH STATE MACHINE ===================== */
    private enum LaunchState {
        IDLE,

        // shared
        BURP,
        SPINUP,

        // NORMAL (non-sorting): same structure as loadedHotPotatoSwerve
        N_FIRE_1, N_RESET_1,
        N_FEED_2, N_FIRE_2, N_RESET_2,
        N_FEED_3, N_FIRE_3, N_RESET_3,

        // SORTED explicit-case runner
        S_STEP_BEGIN,
        S_STOW_WAIT_CLEAR,
        S_FEED_WAIT,
        S_FIRE_HOLD,
        S_RESET_WAIT,
        S_RETURN_WAIT_FILL,

        SPINDOWN
    }

    private LaunchState launchState = LaunchState.IDLE;
    private long stateTimer = 0;

    /* ===================== VISION (turretCam + AprilTag) ===================== */
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // ===== auto fit: distance inches -> multiplier =====
    private static final double AUTO_M = 0.0022636364;
    private static final double AUTO_B = 0.5415909091;
    private static final double AUTO_ADJUSTER_POS = 0.4482;

    private double lastKnownMultiplier = 0.66;
    private double lastKnownAdjusterPos = AUTO_ADJUSTER_POS;

    /* ===================== INIT VISION ===================== */
    private void initTurretCamAprilTags() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "turretCam"))
                .addProcessor(aprilTag)
                .build();
    }

    private AprilTagDetection getFirstTagById(int id) {
        if (aprilTag == null) return null;
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection d : detections) {
            if (d != null && d.id == id && d.ftcPose != null) return d;
        }
        return null;
    }

    private double getTag20RangeInches() {
        AprilTagDetection d = getFirstTagById(20);
        return (d == null) ? Double.NaN : d.ftcPose.range;
    }

    private double getTag20BearingDeg() {
        AprilTagDetection d = getFirstTagById(20);
        return (d == null) ? Double.NaN : d.ftcPose.bearing; // degrees, 0 = centered
    }

    private void tryLockPattern21_22_23() {
        if (patternLocked || aprilTag == null) return;
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection d : detections) {
            if (d == null) continue;
            if (d.id == 21 || d.id == 22 || d.id == 23) {
                storedPatternId = d.id;
                patternLocked = true;
                return;
            }
        }
    }

    private void updateLightServo(boolean tagVisible, double tagBearingDeg) {
        if (light == null) return;

        if (!tagVisible) {
            light.setPosition(LIGHT_NO_TAG);
            return;
        }

        if (!Double.isNaN(tagBearingDeg) && Math.abs(tagBearingDeg) <= LIGHT_LOCK_DEG) {
            light.setPosition(LIGHT_TAG_LOCKED);
        } else {
            light.setPosition(LIGHT_TAG_SEEN);
        }
    }

    /* ===================== MAIN OPMODE ===================== */
    @Override
    public void runOpMode() {

        initHardware();
        initTurretCamAprilTags();

        if (trigger != null) trigger.setPosition(TRIGGER_HOME);

        // Init turret world lock
        double yawDeg0 = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        turretDeg = 135.0;
        turretWorldDeg = yawDeg0 + turretDeg;
        //setTurretServosFromTurretDeg();

        // Adjuster init
        adjusterPos = lastKnownAdjusterPos;
        if (adjuster != null) adjuster.setPosition(adjusterPos);

        // Side sort init
        setSideSortCentered();

        // Light default
        if (light != null) light.setPosition(LIGHT_NO_TAG);

        waitForStart();

        double targetAngleFL = 0, targetAngleFR = 0, targetAngleBL = 0, targetAngleBR = 0;
        long lastLoopMs = System.currentTimeMillis();

        while (opModeIsActive()) {

            long nowMs = System.currentTimeMillis();
            double dt = (nowMs - lastLoopMs) / 1000.0;
            lastLoopMs = nowMs;
            if (dt <= 0) dt = 0.02;

            /* ===================== SORTING MODE TOGGLE (GP2 X) ===================== */
            boolean xNow = gamepad2.x;
            if (xNow && !gp2XPrev) sortingMode = !sortingMode;
            gp2XPrev = xNow;

            /* ===================== BALL DISTANCE SENSING ===================== */
            double fCm = safeDistanceCm(frontDist);
            double cCm = safeDistanceCm(centerDist);
            double bCm = safeDistanceCm(backDist);

            frontHasBall  = hysteresisBall(frontHasBall,  fCm, FRONT_ON_CM,  FRONT_OFF_CM);
            centerHasBall = hysteresisBall(centerHasBall, cCm, CENTER_ON_CM, CENTER_OFF_CM);
            backHasBall   = hysteresisBall(backHasBall,   bCm, BACK_ON_CM,   BACK_OFF_CM);

            /* ===================== BALL COLOR (ONLY IF PRESENT) ===================== */
            frontBallColor  = classifyBallColor(frontColor,  Slot.FRONT,  frontHasBall);
            centerBallColor = classifyBallColor(centerColor, Slot.CENTER, centerHasBall);
            backBallColor   = classifyBallColor(backColor,   Slot.BACK,   backHasBall);

            // Rule: if present and UNKNOWN, assume GREEN
            if (frontHasBall && frontBallColor == BallColor.UNKNOWN) frontBallColor = BallColor.GREEN;
            if (centerHasBall && centerBallColor == BallColor.UNKNOWN) centerBallColor = BallColor.GREEN;
            if (backHasBall && backBallColor == BallColor.UNKNOWN) backBallColor = BallColor.GREEN;

            /* ===================== DRIVE SPEED MODE ===================== */
            double speedMultiplier = gamepad1.right_bumper ? MAX_SPEED_SLOW : MAX_SPEED_FAST;

            // --- Field-Centric Reset (Gamepad 1 dpad_up) ---
            boolean dpadUpNow = gamepad1.y;
            if (dpadUpNow && !dpadUpPrev) imu.resetYaw();
            dpadUpPrev = dpadUpNow;

            /* ===================== DRIVE INPUTS (field-centric) ===================== */
            double fieldY = -gamepad1.left_stick_y * speedMultiplier;
            double fieldX =  gamepad1.left_stick_x * speedMultiplier;
            double rot    =  gamepad1.right_stick_x * speedMultiplier;

            double rawHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double botHeading = wrapAngle(rawHeading);

            double robotX = fieldX * Math.cos(-botHeading) - fieldY * Math.sin(-botHeading);
            double robotY = fieldX * Math.sin(-botHeading) + fieldY * Math.cos(-botHeading);

            double A = robotX - rot * (WHEELBASE / R);
            double B = robotX + rot * (WHEELBASE / R);
            double C = robotY - rot * (TRACK_WIDTH / R);
            double D = robotY + rot * (TRACK_WIDTH / R);

            double speedFrontLeft  = Math.hypot(B, D);
            double speedFrontRight = Math.hypot(B, C);
            double speedBackLeft   = Math.hypot(A, D);
            double speedBackRight  = Math.hypot(A, C);

            double maxSpeed = Math.max(Math.max(speedFrontLeft, speedFrontRight),
                    Math.max(speedBackLeft, speedBackRight));
            if (maxSpeed > 1.0) {
                speedFrontLeft  /= maxSpeed;
                speedFrontRight /= maxSpeed;
                speedBackLeft   /= maxSpeed;
                speedBackRight  /= maxSpeed;
            }

            if (Math.abs(robotX) > DRIVE_DEADBAND || Math.abs(robotY) > DRIVE_DEADBAND || Math.abs(rot) > DRIVE_DEADBAND) {
                targetAngleFL = Math.atan2(B, D);
                targetAngleFR = Math.atan2(B, C);
                targetAngleBL = Math.atan2(A, D);
                targetAngleBR = Math.atan2(A, C);
                framesSinceLastMoved = 0;
            } else {
                speedFrontLeft = 0; speedFrontRight = 0; speedBackLeft = 0; speedBackRight = 0;
                framesSinceLastMoved += 1;
            }

            if (gamepad1.left_stick_button || framesSinceLastMoved >= FRAMES_TO_PLANT_WHEELS) {
                targetAngleFL = -Math.PI / 4; targetAngleFR =  Math.PI / 4;
                targetAngleBL =  Math.PI / 4; targetAngleBR = -Math.PI / 4;
                speedFrontLeft = 0; speedFrontRight = 0; speedBackLeft = 0; speedBackRight = 0;
            }

            runModule(frontLeftDrive,  frontLeftSteer,  frontLeftEncoder,  FRONT_LEFT_OFFSET,  speedFrontLeft,  targetAngleFL);
            runModule(frontRightDrive, frontRightSteer, frontRightEncoder, FRONT_RIGHT_OFFSET, speedFrontRight, targetAngleFR);
            runModule(backLeftDrive,   backLeftSteer,   backLeftEncoder,   BACK_LEFT_OFFSET,   speedBackLeft,   targetAngleBL);
            runModule(backRightDrive,  backRightSteer,  backRightEncoder,  BACK_RIGHT_OFFSET,  speedBackRight,  targetAngleBR);

            /* ===================== VOLTAGE + FLY CMD ===================== */
            double vbat = (voltageSensor != null) ? voltageSensor.getVoltage() : Double.NaN;
            double flyCmd = flywheelCommand(vbat);

            /* ===================== VISION: Tag20 -> multiplier/adjuster/light ; Tags 21/22/23 -> pattern memory ===================== */
            double tagRangeIn = getTag20RangeInches();
            double tagBearingDeg = getTag20BearingDeg();
            boolean tagVisible = !Double.isNaN(tagRangeIn);

            updateLightServo(tagVisible, tagBearingDeg);

            if (tagVisible) {
                multiplier = clamp(AUTO_M * tagRangeIn + AUTO_B, 0.0, 1.0);
                adjusterPos = clamp(AUTO_ADJUSTER_POS, ADJUSTER_MIN, ADJUSTER_MAX);
                lastKnownMultiplier = multiplier;
                lastKnownAdjusterPos = adjusterPos;
                if (adjuster != null) adjuster.setPosition(adjusterPos);
            } else {
                multiplier = clamp(lastKnownMultiplier, 0.0, 1.0);
                adjusterPos = clamp(lastKnownAdjusterPos, ADJUSTER_MIN, ADJUSTER_MAX);
                if (adjuster != null) adjuster.setPosition(adjusterPos);
            }

            tryLockPattern21_22_23();

            /* ===================== TURRET CENTER (GP2 A) ===================== */
            boolean centerBtn = gamepad2.a;
            if (centerBtn && !turretCenterPrev) {
                double yawDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                turretDeg = 135.0;
                turretWorldDeg = yawDeg + turretDeg;
                turretSmoothedInput = 0.0;
            }
            turretCenterPrev = centerBtn;

            /* ===================== TURRET MANUAL INPUT (still stabilizes) ===================== */
            if (Math.abs(-gamepad2.right_stick_x) < 0.95) {
                turretPosition += Math.signum(-gamepad2.right_stick_x) * 0.15;
            }
            if (gamepad2.dpad_left) {
                turretPosition += 0.04;
            }
            if (gamepad2.dpad_right) {
                turretPosition -= 0.04;
            }
            turretRotation1.setPosition(turretPosition);
            turretRotation2.setPosition(1.0 - turretPosition);
            /*double rawInputT = -gamepad2.right_stick_x;
            if (Math.abs(rawInputT) < TURRET_INPUT_DEADBAND) rawInputT = 0.0;
            turretSmoothedInput = turretSmoothedInput + TURRET_SMOOTHING * (rawInputT - turretSmoothedInput);
             */

            double manualDps = gamepad2.right_bumper ? MANUAL_TURRET_DPS_SLOW : MANUAL_TURRET_DPS_FAST;
            turretWorldDeg += turretSmoothedInput * manualDps * dt;

            double yawDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double desiredTurretDeg = turretWorldDeg - yawDeg;
            turretDeg = clamp(desiredTurretDeg, 0.0, TURRET_RANGE_DEG);
            //setTurretServosFromTurretDeg();

            /* ===================== SIDE SORT MANUAL (GP2 dpad left/right) when NOT launching ===================== */
            /*
            if (launchState == LaunchState.IDLE) {
                if (gamepad2.dpad_left) {
                    sideSortPos = clamp(sideSortPos - SIDE_SORT_JOG_STEP, 0.0, 1.0);
                    if (sideSort != null) sideSort.setPosition(sideSortPos);
                } else if (gamepad2.dpad_right) {
                    sideSortPos = clamp(sideSortPos + SIDE_SORT_JOG_STEP, 0.0, 1.0);
                    if (sideSort != null) sideSort.setPosition(sideSortPos);
                }
            }
             */

            /* ===================== PRE-SPIN TOGGLE (GP1 LT tap) ===================== */
            boolean ltNow = (gamepad1.left_trigger > 0.5);
            boolean ltPressedEdge = ltNow && !ltPrev;
            ltPrev = ltNow;

            if (launchState == LaunchState.IDLE && ltPressedEdge) {
                preSpinLatched = !preSpinLatched;
                if (preSpinLatched) preSpinStartMs = System.currentTimeMillis();
                else {
                    preSpinStartMs = 0;
                    if (leftFly != null) leftFly.setPower(0);
                    if (rightFly != null) rightFly.setPower(0);
                }
            }

            if (preSpinLatched && launchState == LaunchState.IDLE) {
                if (leftFly != null) leftFly.setPower(flyCmd);
                if (rightFly != null) rightFly.setPower(flyCmd);
                if (frontIntake != null) frontIntake.setPower(0);
                if (backIntake != null) backIntake.setPower(0);
                if (trigger != null) trigger.setPosition(TRIGGER_HOME);
            }

            /* ===================== MANUAL INTAKE (when not launching) ===================== */
            if (launchState == LaunchState.IDLE) {
                boolean intakeToggle = gamepad1.right_trigger > 0.5;
                if (intakeToggle && !intakeTogglePrev) isIntakeOn = !isIntakeOn;
                intakeTogglePrev = intakeToggle;

                boolean modeToggle = gamepad1.dpad_left;
                if (modeToggle && !intakeModePrev) intakeModeOne = !intakeModeOne;
                intakeModePrev = modeToggle;

                // Must be centered before allowing intakes to turn on
                boolean sideSortCenteredEnough = (Math.abs(sideSortPos - SIDE_SORT_CENTERED) <= 0.04);

                if (!sideSortCenteredEnough) {
                    if (frontIntake != null) frontIntake.setPower(0);
                    if (backIntake != null) backIntake.setPower(0);
                } else if (!preSpinLatched) {
                    if (isIntakeOn) applySmartIntake(vbat, flyCmd);
                    else {
                        if (frontIntake != null) frontIntake.setPower(0);
                        if (backIntake != null) backIntake.setPower(0);
                    }
                }
            }

            /* ===================== LAUNCH START (GP1 A) ===================== */
            if (launchState == LaunchState.IDLE && gamepad1.a) {

                // Decide if we are doing SORTED explicit-case, or NORMAL sequence
                boolean doSorted = sortingMode && patternLocked && (storedPatternId == 21 || storedPatternId == 22 || storedPatternId == 23);

                // flywheel spin timing
                long start = System.currentTimeMillis();
                long preSpinElapsed = (preSpinStartMs > 0) ? (start - preSpinStartMs) : 0;
                spinupRemainingMs = Math.max(0, FLYWHEEL_SPINUP_MS - preSpinElapsed);

                // Launch owns flywheels; consume latches
                preSpinLatched = false;
                preSpinStartMs = 0;

                // Start flywheels now
                if (leftFly != null) leftFly.setPower(flyCmd);
                if (rightFly != null) rightFly.setPower(flyCmd);

                // Stop intakes now
                isIntakeOn = false;
                if (frontIntake != null) frontIntake.setPower(0);
                if (backIntake != null) backIntake.setPower(0);

                if (trigger != null) trigger.setPosition(TRIGGER_HOME);
                shotRetryCount = 0;

                // Always start with BURP then SPINUP
                stateTimer = start;

                if (doSorted) {
                    // SNAPSHOT for cases
                    takeSortedSnapshot(storedPatternId);
                    buildSortedCasePlan(); // sets sortedCase + sortedSteps
                } else {
                    // NORMAL snapshot plans (same as loadedHotPotatoSwerve)
                    boolean runAllThree = fullStackLatched;
                    fullStackLatched = false;

                    planCenterShot = true;
                    planFrontShot  = runAllThree || frontHasBall;
                    planBackShot   = runAllThree || backHasBall;
                }

                launchState = LaunchState.BURP;
            }

            /* ===================== LAUNCH ABORT (GP1 B) ===================== */
            if (gamepad1.b && launchState != LaunchState.IDLE) abortLaunch();

            /* ===================== LAUNCH SEQUENCE ===================== */
            switch (launchState) {

                case IDLE:
                    break;

                case BURP: {
                    // outward burp
                    if (leftFly != null) leftFly.setPower(flyCmd);
                    if (rightFly != null) rightFly.setPower(flyCmd);
                    if (trigger != null) trigger.setPosition(TRIGGER_HOME);

                    if (frontIntake != null) frontIntake.setPower(intakeCommand(-SPINUP_OUTWARD_POWER, vbat));
                    if (backIntake != null)  backIntake.setPower(intakeCommand(+SPINUP_OUTWARD_POWER, vbat));

                    if (System.currentTimeMillis() - stateTimer >= SPINUP_OUTWARD_MS) {
                        if (frontIntake != null) frontIntake.setPower(0);
                        if (backIntake != null) backIntake.setPower(0);

                        stateTimer = System.currentTimeMillis();
                        if (spinupRemainingMs > 0) {
                            launchState = LaunchState.SPINUP;
                        } else {
                            // choose next based on whether sorted plan exists
                            if (sortedSteps.length > 0) {
                                sortedStepIndex = 0;
                                launchState = LaunchState.S_STEP_BEGIN;
                            } else {
                                // normal: fire 1 immediately
                                if (trigger != null) trigger.setPosition(TRIGGER_FIRE);
                                shotRetryCount = 0;
                                stateTimer = System.currentTimeMillis();
                                launchState = LaunchState.N_FIRE_1;
                            }
                        }
                    }
                    break;
                }

                case SPINUP: {
                    if (leftFly != null) leftFly.setPower(flyCmd);
                    if (rightFly != null) rightFly.setPower(flyCmd);
                    if (frontIntake != null) frontIntake.setPower(0);
                    if (backIntake != null) backIntake.setPower(0);
                    if (trigger != null) trigger.setPosition(TRIGGER_HOME);

                    if (System.currentTimeMillis() - stateTimer >= spinupRemainingMs) {
                        stateTimer = System.currentTimeMillis();
                        if (sortedSteps.length > 0) {
                            sortedStepIndex = 0;
                            launchState = LaunchState.S_STEP_BEGIN;
                        } else {
                            if (trigger != null) trigger.setPosition(TRIGGER_FIRE);
                            shotRetryCount = 0;
                            launchState = LaunchState.N_FIRE_1;
                        }
                    }
                    break;
                }

                /* ============ NORMAL (NON-SORTING) — matches loadedHotPotatoSwerve structure ============ */

                case N_FIRE_1:
                    holdFireThenReset(LaunchState.N_RESET_1, flyCmd);
                    break;

                case N_RESET_1:
                    if (retryIfCenterStillHasBall(LaunchState.N_FIRE_1, flyCmd)) break;
                    stateTimer = System.currentTimeMillis();
                    if (planFrontShot) launchState = LaunchState.N_FEED_2;
                    else if (planBackShot) launchState = LaunchState.N_FEED_3;
                    else launchState = LaunchState.SPINDOWN;
                    break;

                case N_FEED_2:
                    setFlyHold(flyCmd);
                    if (trigger != null) trigger.setPosition(TRIGGER_HOME);

                    if (frontIntake != null) frontIntake.setPower(intakeCommand(+FRONT_FEED_POWER, vbat));
                    if (backIntake != null) backIntake.setPower(0);

                    if (System.currentTimeMillis() - stateTimer >= FEED_TO_CENTER_MS) {
                        stopIntakes();
                        if (trigger != null) trigger.setPosition(TRIGGER_FIRE);
                        shotRetryCount = 0;
                        stateTimer = System.currentTimeMillis();
                        launchState = LaunchState.N_FIRE_2;
                    }
                    break;

                case N_FIRE_2:
                    holdFireThenReset(LaunchState.N_RESET_2, flyCmd);
                    break;

                case N_RESET_2:
                    if (retryIfCenterStillHasBall(LaunchState.N_FIRE_2, flyCmd)) break;
                    stateTimer = System.currentTimeMillis();
                    if (planBackShot) launchState = LaunchState.N_FEED_3;
                    else launchState = LaunchState.SPINDOWN;
                    break;

                case N_FEED_3:
                    setFlyHold(flyCmd);
                    if (trigger != null) trigger.setPosition(TRIGGER_HOME);

                    if (backIntake != null) backIntake.setPower(intakeCommand(-BACK_FEED_POWER, vbat));
                    if (frontIntake != null) frontIntake.setPower(0);

                    if (System.currentTimeMillis() - stateTimer >= FEED_TO_CENTER_MS) {
                        stopIntakes();
                        if (trigger != null) trigger.setPosition(TRIGGER_FIRE);
                        shotRetryCount = 0;
                        stateTimer = System.currentTimeMillis();
                        launchState = LaunchState.N_FIRE_3;
                    }
                    break;

                case N_FIRE_3:
                    holdFireThenReset(LaunchState.N_RESET_3, flyCmd);
                    break;

                case N_RESET_3:
                    if (retryIfCenterStillHasBall(LaunchState.N_FIRE_3, flyCmd)) break;
                    stateTimer = System.currentTimeMillis();
                    launchState = LaunchState.SPINDOWN;
                    break;

                /* ============ SORTED (EXPLICIT CASES) ============ */

                case S_STEP_BEGIN: {
                    // Safety: no feeding while moving sideSort
                    stopIntakes();
                    setFlyHold(flyCmd);
                    if (trigger != null) trigger.setPosition(TRIGGER_HOME);

                    if (sortedStepIndex >= sortedSteps.length) {
                        launchState = LaunchState.SPINDOWN;
                        stateTimer = System.currentTimeMillis();
                        break;
                    }

                    SortedStep step = sortedSteps[sortedStepIndex];

                    switch (step) {

                        case STOW_CENTER_TO_SIDE: {
                            // Move to side and wait
                            stowSideSort();
                            stateTimer = System.currentTimeMillis();
                            launchState = LaunchState.S_STOW_WAIT_CLEAR;
                            break;
                        }

                        case FIRE_CENTER: {
                            // If center empty, just skip this step (prevents deadlocks)
                            if (!centerHasBall) {
                                sortedStepIndex++;
                                launchState = LaunchState.S_STEP_BEGIN;
                                break;
                            }

                            if (trigger != null) trigger.setPosition(TRIGGER_FIRE);
                            shotRetryCount = 0;
                            stateTimer = System.currentTimeMillis();
                            launchState = LaunchState.S_FIRE_HOLD;
                            break;
                        }

                        case FEED_FRONT_THEN_FIRE: {
                            // Feed from front into center, then fire
                            stateTimer = System.currentTimeMillis();
                            if (frontIntake != null) frontIntake.setPower(intakeCommand(+FRONT_FEED_POWER, vbat));
                            if (backIntake != null) backIntake.setPower(0);
                            launchState = LaunchState.S_FEED_WAIT;
                            break;
                        }

                        case FEED_BACK_THEN_FIRE: {
                            stateTimer = System.currentTimeMillis();
                            if (backIntake != null) backIntake.setPower(intakeCommand(-BACK_FEED_POWER, vbat));
                            if (frontIntake != null) frontIntake.setPower(0);
                            launchState = LaunchState.S_FEED_WAIT;
                            break;
                        }

                        case RETURN_SIDE_TO_CENTER_THEN_FIRE: {
                            // Burp briefly BEFORE sliding back, then center sideSort and wait for center fill
                            doShortOutwardBurp(vbat);
                            setSideSortCentered();
                            stateTimer = System.currentTimeMillis();
                            launchState = LaunchState.S_RETURN_WAIT_FILL;
                            break;
                        }
                    }
                    break;
                }

                case S_STOW_WAIT_CLEAR: {
                    // Exclusivity: do not feed during this
                    stopIntakes();
                    setFlyHold(flyCmd);
                    if (trigger != null) trigger.setPosition(TRIGGER_HOME);

                    long elapsed = System.currentTimeMillis() - stateTimer;

                    // always wait for slide+settle, then wait for center to clear
                    if (elapsed < (SIDE_SORT_SLIDE_MS + SIDE_SORT_SETTLE_BEFORE_FEED_MS)) break;

                    // after the physical motion is done, wait for center to actually clear (or timeout)
                    if (!centerHasBall || elapsed >= (SIDE_SORT_SLIDE_MS + SIDE_SORT_SETTLE_BEFORE_FEED_MS + SIDE_SORT_WAIT_CENTER_CLEAR_MS)) {
                        sortedStepIndex++;
                        launchState = LaunchState.S_STEP_BEGIN;
                    }
                    break;
                }

                case S_FEED_WAIT: {
                    setFlyHold(flyCmd);
                    if (trigger != null) trigger.setPosition(TRIGGER_HOME);

                    long elapsed = System.currentTimeMillis() - stateTimer;
                    if (elapsed >= FEED_TO_CENTER_MS) {
                        stopIntakes();
                        // Fire immediately after feed
                        if (trigger != null) trigger.setPosition(TRIGGER_FIRE);
                        shotRetryCount = 0;
                        stateTimer = System.currentTimeMillis();
                        launchState = LaunchState.S_FIRE_HOLD;
                    }
                    break;
                }

                case S_RETURN_WAIT_FILL: {
                    // Exclusivity: no feeding while returning
                    stopIntakes();
                    setFlyHold(flyCmd);
                    if (trigger != null) trigger.setPosition(TRIGGER_HOME);

                    long elapsed = System.currentTimeMillis() - stateTimer;

                    // wait for slide motion minimum
                    if (elapsed < (SIDE_SORT_SLIDE_MS + SIDE_SORT_SETTLE_BEFORE_FEED_MS)) break;

                    // wait for center to become occupied (or timeout)
                    if (centerHasBall || elapsed >= (SIDE_SORT_SLIDE_MS + SIDE_SORT_SETTLE_BEFORE_FEED_MS + SIDE_SORT_WAIT_CENTER_FILL_MS)) {
                        // If still empty, skip (prevents hanging)
                        if (!centerHasBall) {
                            sortedStepIndex++;
                            launchState = LaunchState.S_STEP_BEGIN;
                            break;
                        }
                        if (trigger != null) trigger.setPosition(TRIGGER_FIRE);
                        shotRetryCount = 0;
                        stateTimer = System.currentTimeMillis();
                        launchState = LaunchState.S_FIRE_HOLD;
                    }
                    break;
                }

                case S_FIRE_HOLD: {
                    // Hold trigger, then reset
                    setFlyHold(flyCmd);
                    stopIntakes();

                    if (System.currentTimeMillis() - stateTimer >= LAUNCH_TRIGGER_HOLD_MS) {
                        if (trigger != null) trigger.setPosition(TRIGGER_HOME);
                        stateTimer = System.currentTimeMillis();
                        launchState = LaunchState.S_RESET_WAIT;
                    }
                    break;
                }

                case S_RESET_WAIT: {
                    // IMPORTANT: wait for trigger reset BEFORE deciding retry, then ensure center clears
                    setFlyHold(flyCmd);
                    stopIntakes();
                    if (trigger != null) trigger.setPosition(TRIGGER_HOME);

                    long elapsed = System.currentTimeMillis() - stateTimer;
                    if (elapsed < TRIGGER_RESET_WAIT_MS) break;

                    if (centerHasBall) {
                        // give it extra time then retry
                        if (elapsed < (TRIGGER_RESET_WAIT_MS + RETRY_EXTRA_WAIT_MS)) break;

                        if (shotRetryCount >= MAX_RETRIES_PER_SHOT) {
                            abortLaunch();
                            break;
                        }
                        shotRetryCount++;
                        if (trigger != null) trigger.setPosition(TRIGGER_FIRE);
                        stateTimer = System.currentTimeMillis();
                        launchState = LaunchState.S_FIRE_HOLD;
                        break;
                    }

                    // success: advance to next sorted step
                    shotRetryCount = 0;
                    sortedStepIndex++;
                    launchState = LaunchState.S_STEP_BEGIN;
                    break;
                }

                case SPINDOWN: {
                    setFlyHold(flyCmd);
                    stopIntakes();
                    if (trigger != null) trigger.setPosition(TRIGGER_HOME);

                    if (System.currentTimeMillis() - stateTimer >= FLYWHEEL_SPINDOWN_MS) {
                        if (leftFly != null) leftFly.setPower(0);
                        if (rightFly != null) rightFly.setPower(0);
                        launchState = LaunchState.IDLE;

                        // reset launch vars
                        spinupRemainingMs = 0;
                        planCenterShot = false;
                        planFrontShot = false;
                        planBackShot = false;

                        // reset sorted
                        sortedSteps = new SortedStep[0];
                        sortedStepIndex = 0;
                        sortedCase = SortedCase.NONE;

                        // return side sort to center as safe default
                        setSideSortCentered();
                    }
                    break;
                }
            }

            /* ===================== TELEMETRY ===================== */
            telemetry.addData("SortingMode", sortingMode);
            telemetry.addData("PatternLocked", "%s (%d)", patternLocked, storedPatternId);

            telemetry.addData("BallsPresent", "F:%s C:%s B:%s", frontHasBall, centerHasBall, backHasBall);
            telemetry.addData("BallColors", "F:%s C:%s B:%s", frontBallColor, centerBallColor, backBallColor);

            telemetry.addData("SortedCase", "%s", sortedCase);
            telemetry.addData("SortedStep", "%d/%d", sortedStepIndex, sortedSteps.length);

            telemetry.addData("Tag20", tagVisible ? "VISIBLE" : "NO");
            telemetry.addData("Tag20 range(in)", "%.2f", tagRangeIn);
            telemetry.addData("Tag20 bearing(deg)", "%.2f", tagBearingDeg);
            telemetry.addData("LightPos", (light != null) ? "%.3f" : "null", (light != null) ? light.getPosition() : 0.0);

            telemetry.addData("TurretDeg", "%.1f (0..270)", turretDeg);
            telemetry.addData("YawDeg", "%.1f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

            telemetry.addData("SideSortPos", "%.3f", sideSortPos);

            telemetry.addData("F scores", "G=%.3f P=%.3f", lastFrontGScore, lastFrontPScore);
            telemetry.addData("C scores", "G=%.3f P=%.3f", lastCenterGScore, lastCenterPScore);
            telemetry.addData("B scores", "G=%.3f P=%.3f", lastBackGScore, lastBackPScore);

            telemetry.addData("FullStackLatched", fullStackLatched);
            telemetry.addData("PreSpin", preSpinLatched);
            telemetry.addData("LaunchState", launchState);

            telemetry.update();
        }

        if (visionPortal != null) visionPortal.close();
    }

    /* ===================== SORTED CASE: SNAPSHOT + PLAN BUILDER ===================== */

    private void takeSortedSnapshot(int patternId) {
        snapPatternId = patternId;

        snapF = frontHasBall;
        snapC = centerHasBall;
        snapB = backHasBall;

        snapFC = snapF ? frontBallColor : BallColor.EMPTY;
        snapCC = snapC ? centerBallColor : BallColor.EMPTY;
        snapBC = snapB ? backBallColor : BallColor.EMPTY;

        // Assume UNKNOWN as GREEN already handled earlier, but keep safe
        if (snapF && snapFC == BallColor.UNKNOWN) snapFC = BallColor.GREEN;
        if (snapC && snapCC == BallColor.UNKNOWN) snapCC = BallColor.GREEN;
        if (snapB && snapBC == BallColor.UNKNOWN) snapBC = BallColor.GREEN;
    }

    private void buildSortedCasePlan() {
        sortedCase = SortedCase.NONE;
        sortedSteps = new SortedStep[0];
        sortedStepIndex = 0;

        // Determine where GREEN is in the snapshot
        boolean gC = snapC && snapCC == BallColor.GREEN;
        boolean gF = snapF && snapFC == BallColor.GREEN;
        boolean gB = snapB && snapBC == BallColor.GREEN;

        if (snapPatternId == 23) {
            // Pattern 23 rules you gave:
            // If green center: stow, feed+shoot front, feed+shoot back, bring green back and shoot
            // If green back: launch center, launch front, launch back
            // If green front: launch center, launch back, launch front
            if (gC) {
                sortedCase = SortedCase.P23_GREEN_CENTER;
                sortedSteps = new SortedStep[] {
                        SortedStep.STOW_CENTER_TO_SIDE,
                        SortedStep.FEED_FRONT_THEN_FIRE,
                        SortedStep.FEED_BACK_THEN_FIRE,
                        SortedStep.RETURN_SIDE_TO_CENTER_THEN_FIRE
                };
            } else if (gB) {
                sortedCase = SortedCase.P23_GREEN_BACK;
                sortedSteps = new SortedStep[] {
                        SortedStep.FIRE_CENTER,
                        SortedStep.FEED_FRONT_THEN_FIRE,
                        SortedStep.FEED_BACK_THEN_FIRE
                };
            } else if (gF) {
                sortedCase = SortedCase.P23_GREEN_FRONT;
                sortedSteps = new SortedStep[] {
                        SortedStep.FIRE_CENTER,
                        SortedStep.FEED_BACK_THEN_FIRE,
                        SortedStep.FEED_FRONT_THEN_FIRE
                };
            } else {
                // fallback: just do center then front then back
                sortedSteps = new SortedStep[] {
                        SortedStep.FIRE_CENTER,
                        SortedStep.FEED_FRONT_THEN_FIRE,
                        SortedStep.FEED_BACK_THEN_FIRE
                };
            }
            return;
        }

        if (snapPatternId == 22) {
            // Pattern 22 rules:
            // If green center: stow, launch front, bring green back, launch center, launch back
            // if green back: launch center, launch back, launch front
            // if green front: launch center, launch front, launch back
            if (gC) {
                sortedCase = SortedCase.P22_GREEN_CENTER;
                sortedSteps = new SortedStep[] {
                        SortedStep.STOW_CENTER_TO_SIDE,
                        SortedStep.FEED_FRONT_THEN_FIRE,
                        SortedStep.RETURN_SIDE_TO_CENTER_THEN_FIRE,
                        SortedStep.FIRE_CENTER,
                        SortedStep.FEED_BACK_THEN_FIRE
                };
            } else if (gB) {
                sortedCase = SortedCase.P22_GREEN_BACK;
                sortedSteps = new SortedStep[] {
                        SortedStep.FIRE_CENTER,
                        SortedStep.FEED_BACK_THEN_FIRE,
                        SortedStep.FEED_FRONT_THEN_FIRE
                };
            } else if (gF) {
                sortedCase = SortedCase.P22_GREEN_FRONT;
                sortedSteps = new SortedStep[] {
                        SortedStep.FIRE_CENTER,
                        SortedStep.FEED_FRONT_THEN_FIRE,
                        SortedStep.FEED_BACK_THEN_FIRE
                };
            } else {
                sortedSteps = new SortedStep[] {
                        SortedStep.FIRE_CENTER,
                        SortedStep.FEED_FRONT_THEN_FIRE,
                        SortedStep.FEED_BACK_THEN_FIRE
                };
            }
            return;
        }

        if (snapPatternId == 21) {
            // Pattern 21 rules:
            // If green center: fire center, fire front, fire back
            // If green front: stow center, fire front, fire back, bring back purple and launch center
            // If green back: stow center, fire back, fire front, return purple to center and fire
            if (gC) {
                sortedCase = SortedCase.P21_GREEN_CENTER;
                sortedSteps = new SortedStep[] {
                        SortedStep.FIRE_CENTER,
                        SortedStep.FEED_FRONT_THEN_FIRE,
                        SortedStep.FEED_BACK_THEN_FIRE
                };
            } else if (gF) {
                sortedCase = SortedCase.P21_GREEN_FRONT;
                sortedSteps = new SortedStep[] {
                        SortedStep.STOW_CENTER_TO_SIDE,
                        SortedStep.FEED_FRONT_THEN_FIRE,
                        SortedStep.FEED_BACK_THEN_FIRE,
                        SortedStep.RETURN_SIDE_TO_CENTER_THEN_FIRE
                };
            } else if (gB) {
                sortedCase = SortedCase.P21_GREEN_BACK;
                sortedSteps = new SortedStep[] {
                        SortedStep.STOW_CENTER_TO_SIDE,
                        SortedStep.FEED_BACK_THEN_FIRE,
                        SortedStep.FEED_FRONT_THEN_FIRE,
                        SortedStep.RETURN_SIDE_TO_CENTER_THEN_FIRE
                };
            } else {
                sortedSteps = new SortedStep[] {
                        SortedStep.FIRE_CENTER,
                        SortedStep.FEED_FRONT_THEN_FIRE,
                        SortedStep.FEED_BACK_THEN_FIRE
                };
            }
        }
    }

    /* ===================== SORT HELPERS ===================== */
    private void doShortOutwardBurp(double vbat) {
        // tiny outward burp, then stop
        if (frontIntake != null) frontIntake.setPower(intakeCommand(-SPINUP_OUTWARD_POWER, vbat));
        if (backIntake != null)  backIntake.setPower(intakeCommand(+SPINUP_OUTWARD_POWER, vbat));
        sleep(SHORT_BURP_MS);
        stopIntakes();
    }

    /* ===================== NORMAL LAUNCH HELPERS (same behavior) ===================== */
    private void holdFireThenReset(LaunchState nextResetState, double flyCmd) {
        setFlyHold(flyCmd);
        stopIntakes();

        if (System.currentTimeMillis() - stateTimer >= LAUNCH_TRIGGER_HOLD_MS) {
            if (trigger != null) trigger.setPosition(TRIGGER_HOME);
            stateTimer = System.currentTimeMillis();
            launchState = nextResetState;
        }
    }

    private boolean retryIfCenterStillHasBall(LaunchState fireStateToRetry, double flyCmd) {
        setFlyHold(flyCmd);
        stopIntakes();
        if (trigger != null) trigger.setPosition(TRIGGER_HOME);

        long elapsed = System.currentTimeMillis() - stateTimer;
        if (elapsed < TRIGGER_RESET_WAIT_MS) return true;

        if (centerHasBall) {
            if (elapsed < (TRIGGER_RESET_WAIT_MS + RETRY_EXTRA_WAIT_MS)) return true;

            if (shotRetryCount >= MAX_RETRIES_PER_SHOT) {
                abortLaunch();
                return true;
            }
            shotRetryCount++;
            if (trigger != null) trigger.setPosition(TRIGGER_FIRE);
            stateTimer = System.currentTimeMillis();
            launchState = fireStateToRetry;
            return true;
        }

        shotRetryCount = 0;
        return false;
    }

    private void setFlyHold(double flyCmd) {
        if (leftFly != null) leftFly.setPower(flyCmd);
        if (rightFly != null) rightFly.setPower(flyCmd);
    }

    private void stopIntakes() {
        if (frontIntake != null) frontIntake.setPower(0);
        if (backIntake != null) backIntake.setPower(0);
    }

    /* ===================== SIDE SORT HELPERS ===================== */
    private void setSideSortCentered() {
        sideSortPos = SIDE_SORT_CENTERED;
        if (sideSort != null) sideSort.setPosition(sideSortPos);
    }

    private void stowSideSort() {
        sideSortPos = SIDE_SORT_STOWED;
        if (sideSort != null) sideSort.setPosition(sideSortPos);
    }

    /* ===================== TURRET SERVO OUTPUT ===================== */
    private void setTurretServosFromTurretDeg() {
        double pos = clamp(turretDeg / TURRET_RANGE_DEG, MIN_TURRET_ROTATION, MAX_TURRET_ROTATION);
        if (turretRotation1 != null) turretRotation1.setPosition(pos);
        if (turretRotation2 != null) turretRotation2.setPosition(1.0 - pos);
    }

    /* ===================== COLOR CLASSIFICATION ===================== */
    private BallColor classifyBallColor(NormalizedColorSensor s, Slot slot, boolean slotHasBall) {
        if (!slotHasBall) return BallColor.EMPTY;
        if (s == null) return BallColor.UNKNOWN;

        NormalizedRGBA c = s.getNormalizedColors();
        double r = c.red, g = c.green, b = c.blue;

        double scoreGreen  = g + GREEN_B_WEIGHT  * b;
        double scorePurple = r + PURPLE_B_WEIGHT * b;

        if (slot == Slot.FRONT) {
            lastFrontGScore = scoreGreen; lastFrontPScore = scorePurple;
        } else if (slot == Slot.CENTER) {
            lastCenterGScore = scoreGreen; lastCenterPScore = scorePurple;
        } else {
            lastBackGScore = scoreGreen; lastBackPScore = scorePurple;
        }

        if (scoreGreen > scorePurple * GREEN_RATIO) return BallColor.GREEN;
        if (scorePurple > scoreGreen * PURPLE_RATIO) return BallColor.PURPLE;
        return BallColor.UNKNOWN;
    }

    /* ===================== VOLTAGE COMP HELPERS ===================== */
    private double flywheelCommand(double vbat) {
        if (Double.isNaN(vbat) || Double.isInfinite(vbat) || vbat <= 0.5) return multiplier;
        double base = (vbat < REF_VOLTAGE) ? 1.0 : (REF_VOLTAGE / vbat);
        return clamp(multiplier * base, 0.0, 1.0);
    }

    /* ===================== SMART INTAKE LOGIC ===================== */
    private void applySmartIntake(double vbat, double flyCmd) {
        DcMotor fastMotor, slowMotor;
        double dir = intakeModeOne ? +1.0 : -1.0;

        if (intakeModeOne) {
            fastMotor = frontIntake;
            slowMotor = backIntake;
        } else {
            fastMotor = backIntake;
            slowMotor = frontIntake;
        }

        boolean fastBall = intakeModeOne ? frontHasBall : backHasBall;
        boolean slowBall = intakeModeOne ? backHasBall : frontHasBall;

        double slowPower = slowBall ? 0.0 : (dir * MANUAL_SLOW);

        double fastPower;
        if (!slowBall) {
            fastPower = dir * MANUAL_FAST;
        } else {
            fastPower = dir * (centerHasBall ? FAST_AFTER_SLOW_AND_CENTER : FAST_AFTER_SLOW);

            // AUTO STOP FULLY when center + fast slot are both occupied
            if (centerHasBall && fastBall) {
                fastPower = 0.0;
                slowPower = 0.0;
                isIntakeOn = false;

                fullStackLatched = true;

                if (AUTO_PRESPIN_WHEN_FULL && launchState == LaunchState.IDLE) {
                    preSpinLatched = true;
                    preSpinStartMs = System.currentTimeMillis();
                    setFlyHold(flyCmd);
                }
            }
        }

        double fastCmd = intakeCommand(fastPower, vbat);
        double slowCmd = intakeCommand(slowPower, vbat);

        if (fastMotor != null) fastMotor.setPower(fastCmd);
        if (slowMotor != null) slowMotor.setPower(slowCmd);
    }

    /* ===================== ABORT ===================== */
    private void abortLaunch() {
        stopIntakes();
        if (leftFly != null) leftFly.setPower(0);
        if (rightFly != null) rightFly.setPower(0);
        if (trigger != null) trigger.setPosition(TRIGGER_HOME);

        shotRetryCount = 0;
        launchState = LaunchState.IDLE;

        preSpinLatched = false;
        preSpinStartMs = 0;
        spinupRemainingMs = 0;
        ltPrev = false;

        fullStackLatched = false;

        planCenterShot = false;
        planFrontShot = false;
        planBackShot = false;

        // reset sorted
        sortedSteps = new SortedStep[0];
        sortedStepIndex = 0;
        sortedCase = SortedCase.NONE;

        setSideSortCentered();
    }

    /* ===================== DISTANCE HELPERS ===================== */
    private double safeDistanceCm(DistanceSensor s) {
        if (s == null) return 999.0;
        double cm = s.getDistance(DistanceUnit.CM);
        if (Double.isNaN(cm) || Double.isInfinite(cm)) return 999.0;
        return cm;
    }

    private boolean hysteresisBall(boolean prev, double cm, double onCm, double offCm) {
        return prev ? (cm <= offCm) : (cm <= onCm);
    }

    /* ===================== SWERVE HELPERS ===================== */
    private void runModule(DcMotor drive, CRServo steer, AnalogInput enc,
                           double offset, double speed, double target) {

        double current = wrapAngle(getRawAngle(enc) - offset);
        double delta = wrapAngle(target - current);

        if (Math.abs(delta) > Math.PI / 2) {
            delta = wrapAngle(delta + Math.PI);
            speed *= -1;
        }

        double steerPower = clamp(-STEER_KP * delta, -1, 1);
        if (Math.abs(steerPower) < STEER_DEADBAND) steerPower = 0;

        if (steer != null) steer.setPower(steerPower);
        if (drive != null) drive.setPower(speed);
    }

    private double getRawAngle(AnalogInput enc) {
        if (enc == null) return 0.0;
        return enc.getVoltage() / 3.3 * (2 * Math.PI);
    }

    private double wrapAngle(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private void resetMotors(DcMotor... motors) {
        for (DcMotor m : motors) {
            if (m != null) m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /* ===================== HARDWARE INIT ===================== */
    private void initHardware() {

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

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetMotors(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);

        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
        backIntake  = hardwareMap.get(DcMotor.class, "backIntake");

        leftFly  = hardwareMap.get(DcMotor.class, "leftFly");
        rightFly = hardwareMap.get(DcMotor.class, "rightFly");
        leftFly.setDirection(DcMotor.Direction.REVERSE);
        rightFly.setDirection(DcMotor.Direction.FORWARD);

        turretRotation1 = hardwareMap.get(Servo.class, "turret_rotation_1");
        turretRotation2 = hardwareMap.get(Servo.class, "turret_rotation_2");
        if (turretRotation1 != null) turretRotation1.setDirection(Servo.Direction.FORWARD);
        if (turretRotation2 != null) turretRotation2.setDirection(Servo.Direction.REVERSE);

        trigger  = hardwareMap.get(Servo.class, "trigger");
        adjuster = hardwareMap.get(Servo.class, "adjuster");
        sideSort = hardwareMap.get(Servo.class, "side_sort");

        // Correct config name
        light = hardwareMap.get(Servo.class, "lights");

        try {
            RevColorSensorV3 f = hardwareMap.get(RevColorSensorV3.class, "frontColor");
            RevColorSensorV3 c = hardwareMap.get(RevColorSensorV3.class, "centerColor");
            RevColorSensorV3 b = hardwareMap.get(RevColorSensorV3.class, "backColor");

            frontColor  = f;
            centerColor = c;
            backColor   = b;

            frontDist  = f;
            centerDist = c;
            backDist   = b;

        } catch (Exception e) {
            frontColor  = hardwareMap.get(NormalizedColorSensor.class, "frontColor");
            centerColor = hardwareMap.get(NormalizedColorSensor.class, "centerColor");
            backColor   = hardwareMap.get(NormalizedColorSensor.class, "backColor");

            frontDist  = hardwareMap.get(DistanceSensor.class, "frontColor");
            centerDist = hardwareMap.get(DistanceSensor.class, "centerColor");
            backDist   = hardwareMap.get(DistanceSensor.class, "backColor");
        }
    }
}
