package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "deepFriedPotato", group = "Swerve")
public class deepFriedPotato extends LinearOpMode {

    // ─── Drive Hardware ──────────────────────────────────────────────────────
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private CRServo frontLeftSteer, frontRightSteer, backLeftSteer, backRightSteer;
    private AnalogInput frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder;
    private IMU imu;
    private VoltageSensor voltageSensor;

    // ─── Mechanism Hardware ──────────────────────────────────────────────────
    private DcMotor frontIntake, backIntake;
    private DcMotor leftFly, rightFly;
    private Servo turretRotation1, turretRotation2;
    private Servo trigger, adjuster, sideSort, light;

    // ─── Sensors ─────────────────────────────────────────────────────────────
    private NormalizedColorSensor frontColor, centerColor, backColor;
    private DistanceSensor frontDist, centerDist, backDist;

    // ─── Vision ──────────────────────────────────────────────────────────────
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // =========================================================================
    // CONSTANTS
    // =========================================================================

    // Light servo positions
    private static final double LIGHT_NO_TAG     = 0.0;
    private static final double LIGHT_TAG_SEEN   = 0.388;
    private static final double LIGHT_TAG_LOCKED = 0.5;
    private static final double LIGHT_LOCK_DEG   = 3.0;

    // Side sort servo
    private static final double SIDE_SORT_CENTERED          = 0.4;
    private static final double SIDE_SORT_STOWED             = 0.955;
    private static final double SIDE_SORT_JOG_STEP           = 0.02;
    private static final long   SIDE_SORT_SLIDE_MS            = 300;
    private static final long   SIDE_SORT_SETTLE_BEFORE_FEED_MS = 250;
    private static final long   SIDE_SORT_WAIT_CENTER_CLEAR_MS  = 900;
    private static final long   SIDE_SORT_WAIT_CENTER_FILL_MS   = 1100;

    // Swerve geometry
    private static final double TRACK_WIDTH = 17.258;
    private static final double WHEELBASE   = 13.544;
    private static final double R = Math.hypot(TRACK_WIDTH, WHEELBASE);

    // Swerve encoder offsets
    private static final double FRONT_LEFT_OFFSET  = 1.34;
    private static final double FRONT_RIGHT_OFFSET = 3.161;
    private static final double BACK_LEFT_OFFSET   = 1.589;
    private static final double BACK_RIGHT_OFFSET  = 1.237;

    // Swerve control
    private static final double STEER_KP        = 0.8;
    private static final double DRIVE_DEADBAND  = 0.05;
    private static final double STEER_DEADBAND  = 0.08;
    private static final double MAX_SPEED_FAST  = 0.8;
    private static final double MAX_SPEED_SLOW  = 0.4;
    private static final int    FRAMES_TO_PLANT = 5;

    // Turret
    private static final double TURRET_RANGE_DEG         = 270.0;
    private static final double MIN_TURRET_ROTATION       = 0.0;
    private static final double MAX_TURRET_ROTATION       = 1.0;
    private static final double TURRET_INPUT_DEADBAND     = 0.05;
    private static final double MANUAL_TURRET_DPS_FAST    = 220.0;
    private static final double MANUAL_TURRET_DPS_SLOW    = 75.0;
    private static final double TURRET_SMOOTHING          = 0.25;

    // Adjuster
    private static final double ADJUSTER_MIN = 0.0;
    private static final double ADJUSTER_MAX = 0.75;

    // Voltage compensation
    private static final double REF_VOLTAGE        = 12.0;
    private static final double INTAKE_REF_VOLTAGE = 9.0;

    // Intake ratios
    private static final double INTAKE_BASE                    = 0.90;
    private static final double INTAKE_SLOW_RATIO              = 0.30;
    private static final double FAST_AFTER_SLOW_RATIO          = 0.70;
    private static final double FAST_AFTER_SLOW_AND_CENTER_RATIO = 0.625 * 0.93;

    private static final double MANUAL_FAST               = INTAKE_BASE;
    private static final double MANUAL_SLOW               = INTAKE_BASE * INTAKE_SLOW_RATIO;
    private static final double FAST_AFTER_SLOW           = INTAKE_BASE * FAST_AFTER_SLOW_RATIO;
    private static final double FAST_AFTER_SLOW_AND_CENTER = INTAKE_BASE * FAST_AFTER_SLOW_AND_CENTER_RATIO;

    // Launch feeding
    private static final double FRONT_FEED_POWER     = 0.65;
    private static final double BACK_FEED_POWER      = 0.375;
    private static final long   FEED_TO_CENTER_MS    = 450;

    // Outward burp
    private static final double SPINUP_OUTWARD_POWER = 0.4;
    private static final long   SPINUP_OUTWARD_MS    = 30;
    private static final long   SHORT_BURP_MS        = 20;

    // Trigger servo
    private static final double TRIGGER_FIRE = 0.0;
    private static final double TRIGGER_HOME = 0.225;

    // Launch timing
    private static final long LAUNCH_TRIGGER_HOLD_MS = 300;
    private static final long TRIGGER_RESET_WAIT_MS  = 125;
    private static final long RETRY_EXTRA_WAIT_MS    = 350;
    private static final long FLYWHEEL_SPINUP_MS     = 1000;
    private static final long FLYWHEEL_SPINDOWN_MS   = 300;

    // Ball distance detection thresholds (hysteresis)
    private static final double FRONT_ON_CM  = 4.0, FRONT_OFF_CM  = 5.0;
    private static final double CENTER_ON_CM = 4.0, CENTER_OFF_CM = 5.0;
    private static final double BACK_ON_CM   = 4.0, BACK_OFF_CM   = 5.0;

    // Retry
    private static final int MAX_RETRIES_PER_SHOT = 3;

    // Auto-prespin
    private static final boolean AUTO_PRESPIN_WHEN_FULL = true;

    // AprilTag auto-fit: distance (inches) → multiplier
    private static final double AUTO_M            = 0.0022636364;
    private static final double AUTO_B            = 0.5415909091;
    private static final double AUTO_ADJUSTER_POS = 0.4482;

    // Color classification weights
    private static final double GREEN_RATIO      = 1.35;
    private static final double PURPLE_RATIO     = 1.12;
    private static final double GREEN_B_WEIGHT   = 0.25;
    private static final double PURPLE_B_WEIGHT  = 1.25;

    // =========================================================================
    // ENUMS
    // =========================================================================

    private enum BallColor  { EMPTY, PURPLE, GREEN, UNKNOWN }
    private enum Slot       { FRONT, CENTER, BACK }

    private enum SortedCase {
        NONE,
        P23_GREEN_CENTER, P23_GREEN_BACK, P23_GREEN_FRONT,
        P22_GREEN_CENTER, P22_GREEN_BACK, P22_GREEN_FRONT,
        P21_GREEN_CENTER, P21_GREEN_FRONT, P21_GREEN_BACK
    }

    private enum SortedStep {
        STOW_CENTER_TO_SIDE,
        FIRE_CENTER,
        FEED_FRONT_THEN_FIRE,
        FEED_BACK_THEN_FIRE,
        RETURN_SIDE_TO_CENTER_THEN_FIRE
    }

    private enum LaunchState {
        IDLE,
        BURP, SPINUP,
        // Normal (non-sorting) sequence
        N_FIRE_1, N_RESET_1,
        N_FEED_2, N_FIRE_2, N_RESET_2,
        N_FEED_3, N_FIRE_3, N_RESET_3,
        // Sorted explicit-case runner
        S_STEP_BEGIN,
        S_STOW_WAIT_CLEAR,
        S_FEED_WAIT,
        S_FIRE_HOLD,
        S_RESET_WAIT,
        S_RETURN_WAIT_FILL,
        SPINDOWN
    }

    // =========================================================================
    // MUTABLE STATE
    // =========================================================================

    // Drive
    private int framesSinceLastMoved = 0;

    // Turret
    private double turretPosition    = 0.5;
    private double turretWorldDeg    = 0.0;
    private double turretDeg         = 135.0;
    private double turretSmoothedInput = 0.0;
    private boolean turretCenterPrev = false;

    // Adjuster / multiplier
    private double adjusterPos           = AUTO_ADJUSTER_POS;
    private double multiplier            = 1.0;
    private double lastKnownMultiplier   = 0.66;
    private double lastKnownAdjusterPos  = AUTO_ADJUSTER_POS;

    // Side sort
    private double sideSortPos = SIDE_SORT_CENTERED;

    // Ball detection
    private boolean frontHasBall  = false;
    private boolean centerHasBall = false;
    private boolean backHasBall   = false;

    // Ball color
    private BallColor frontBallColor  = BallColor.EMPTY;
    private BallColor centerBallColor = BallColor.EMPTY;
    private BallColor backBallColor   = BallColor.EMPTY;

    // Color debug scores
    private double lastFrontGScore = 0, lastFrontPScore = 0;
    private double lastCenterGScore = 0, lastCenterPScore = 0;
    private double lastBackGScore = 0, lastBackPScore = 0;

    // Intake
    private boolean isIntakeOn       = false;
    private boolean intakeTogglePrev = false;
    private boolean intakeModeOne    = true;   // true = IN (front fast, back slow)
    private boolean intakeModePrev   = false;
    private boolean dpadUpPrev       = false;
    private boolean fullStackLatched = false;

    // Normal launch plans
    private boolean planCenterShot = false;
    private boolean planFrontShot  = false;
    private boolean planBackShot   = false;

    // Pre-spin
    private boolean preSpinLatched  = false;
    private boolean ltPrev          = false;
    private long    preSpinStartMs  = 0;
    private long    spinupRemainingMs = 0;

    // Retry
    private int shotRetryCount = 0;

    // Sorting mode
    private boolean sortingMode = false;
    private boolean gp2XPrev    = false;

    // Pattern memory (tags 21/22/23)
    private int     storedPatternId = 0;
    private boolean patternLocked   = false;

    // Sorted snapshot
    private int       snapPatternId = 0;
    private boolean   snapF = false, snapC = false, snapB = false;
    private BallColor snapFC = BallColor.EMPTY, snapCC = BallColor.EMPTY, snapBC = BallColor.EMPTY;

    // Sorted plan
    private SortedCase  sortedCase      = SortedCase.NONE;
    private SortedStep[] sortedSteps    = new SortedStep[0];
    private int          sortedStepIndex = 0;

    // Launch state machine
    private LaunchState launchState = LaunchState.IDLE;
    private long        stateTimer  = 0;

    // =========================================================================
    // MAIN OPMODE
    // =========================================================================

    @Override
    public void runOpMode() {
        initHardware();
        initTurretCamAprilTags();

        trigger.setPosition(TRIGGER_HOME);

        double yawDeg0 = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        turretDeg      = 135.0;
        turretWorldDeg = yawDeg0 + turretDeg;

        adjusterPos = lastKnownAdjusterPos;
        adjuster.setPosition(adjusterPos);
        setSideSortCentered();
        light.setPosition(LIGHT_NO_TAG);

        waitForStart();

        double targetAngleFL = 0, targetAngleFR = 0, targetAngleBL = 0, targetAngleBR = 0;
        long lastLoopMs = System.currentTimeMillis();

        while (opModeIsActive()) {
            long nowMs = System.currentTimeMillis();
            double dt  = Math.max((nowMs - lastLoopMs) / 1000.0, 0.02);
            lastLoopMs = nowMs;

            updateSortingModeToggle();
            updateBallSensors();
            updateBallColors();

            double vbat   = voltageSensor != null ? voltageSensor.getVoltage() : Double.NaN;
            double flyCmd = flywheelCommand(vbat);

            updateVision();
            updateTurret(dt);
            updatePreSpin(flyCmd);

            double speedMul = gamepad1.right_bumper ? MAX_SPEED_SLOW : MAX_SPEED_FAST;

            // Field-centric yaw reset
            boolean dpadUpNow = gamepad1.y;
            if (dpadUpNow && !dpadUpPrev) imu.resetYaw();
            dpadUpPrev = dpadUpNow;

            // Drive
            double[] driveResults = computeDrive(speedMul);
            targetAngleFL = driveResults[0];
            targetAngleFR = driveResults[1];
            targetAngleBL = driveResults[2];
            targetAngleBR = driveResults[3];
            double sFL = driveResults[4], sFR = driveResults[5], sBL = driveResults[6], sBR = driveResults[7];

            runModule(frontLeftDrive,  frontLeftSteer,  frontLeftEncoder,  FRONT_LEFT_OFFSET,  sFL, targetAngleFL);
            runModule(frontRightDrive, frontRightSteer, frontRightEncoder, FRONT_RIGHT_OFFSET, sFR, targetAngleFR);
            runModule(backLeftDrive,   backLeftSteer,   backLeftEncoder,   BACK_LEFT_OFFSET,   sBL, targetAngleBL);
            runModule(backRightDrive,  backRightSteer,  backRightEncoder,  BACK_RIGHT_OFFSET,  sBR, targetAngleBR);

            // Manual intake (idle only)
            if (launchState == LaunchState.IDLE) {
                updateManualIntake(vbat, flyCmd);
            }

            // Launch start / abort
            handleLaunchTriggers(flyCmd, vbat);

            // Launch state machine
            runLaunchStateMachine(flyCmd, vbat);

            updateTelemetry();
        }

        if (visionPortal != null) visionPortal.close();
    }

    // =========================================================================
    // PER-LOOP UPDATE HELPERS
    // =========================================================================

    private void updateSortingModeToggle() {
        boolean xNow = gamepad2.x;
        if (xNow && !gp2XPrev) sortingMode = !sortingMode;
        gp2XPrev = xNow;
    }

    private void updateBallSensors() {
        double fCm = safeDistanceCm(frontDist);
        double cCm = safeDistanceCm(centerDist);
        double bCm = safeDistanceCm(backDist);

        frontHasBall  = hysteresisBall(frontHasBall,  fCm, FRONT_ON_CM,  FRONT_OFF_CM);
        centerHasBall = hysteresisBall(centerHasBall, cCm, CENTER_ON_CM, CENTER_OFF_CM);
        backHasBall   = hysteresisBall(backHasBall,   bCm, BACK_ON_CM,   BACK_OFF_CM);
    }

    private void updateBallColors() {
        frontBallColor  = classifyBallColor(frontColor,  Slot.FRONT,  frontHasBall);
        centerBallColor = classifyBallColor(centerColor, Slot.CENTER, centerHasBall);
        backBallColor   = classifyBallColor(backColor,   Slot.BACK,   backHasBall);

        // Treat UNKNOWN as GREEN if ball is present
        if (frontHasBall  && frontBallColor  == BallColor.UNKNOWN) frontBallColor  = BallColor.GREEN;
        if (centerHasBall && centerBallColor == BallColor.UNKNOWN) centerBallColor = BallColor.GREEN;
        if (backHasBall   && backBallColor   == BallColor.UNKNOWN) backBallColor   = BallColor.GREEN;
    }

    private void updateVision() {
        double tagRangeIn    = getTag24RangeInches();
        double tagBearingDeg = getTag24BearingDeg();
        boolean tagVisible   = !Double.isNaN(tagRangeIn);

        updateLightServo(tagVisible, tagBearingDeg);

        if (tagVisible) {
            multiplier   = clamp(AUTO_M * tagRangeIn + AUTO_B, 0.0, 1.0);
            adjusterPos  = clamp(AUTO_ADJUSTER_POS, ADJUSTER_MIN, ADJUSTER_MAX);
            lastKnownMultiplier  = multiplier;
            lastKnownAdjusterPos = adjusterPos;
        } else {
            multiplier  = clamp(lastKnownMultiplier, 0.0, 1.0);
            adjusterPos = clamp(lastKnownAdjusterPos, ADJUSTER_MIN, ADJUSTER_MAX);
        }
        adjuster.setPosition(adjusterPos);

        tryLockPattern21_22_23();
    }

    private void updateTurret(double dt) {
        // Center button (GP2 A)
        boolean centerBtn = gamepad2.a;
        if (centerBtn && !turretCenterPrev) {
            double yaw    = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            turretDeg     = 135.0;
            turretWorldDeg = yaw + turretDeg;
            turretSmoothedInput = 0.0;
        }
        turretCenterPrev = centerBtn;

        // Manual position adjustment
        if (Math.abs(-gamepad2.right_stick_x) < 0.95)
            turretPosition += Math.signum(-gamepad2.right_stick_x) * 0.15;
        if (gamepad2.dpad_left)  turretPosition += 0.04;
        if (gamepad2.dpad_right) turretPosition -= 0.04;

        turretRotation1.setPosition(turretPosition);
        turretRotation2.setPosition(1.0 - turretPosition);

        // World-stabilization
        double manualDps = gamepad2.right_bumper ? MANUAL_TURRET_DPS_SLOW : MANUAL_TURRET_DPS_FAST;
        turretWorldDeg  += turretSmoothedInput * manualDps * dt;

        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        turretDeg  = clamp(turretWorldDeg - yaw, 0.0, TURRET_RANGE_DEG);
    }

    private void updatePreSpin(double flyCmd) {
        boolean ltNow         = gamepad1.left_trigger > 0.5;
        boolean ltPressedEdge = ltNow && !ltPrev;
        ltPrev = ltNow;

        if (launchState == LaunchState.IDLE && ltPressedEdge) {
            preSpinLatched = !preSpinLatched;
            if (preSpinLatched) {
                preSpinStartMs = System.currentTimeMillis();
            } else {
                preSpinStartMs = 0;
                leftFly.setPower(0);
                rightFly.setPower(0);
            }
        }

        if (preSpinLatched && launchState == LaunchState.IDLE) {
            leftFly.setPower(flyCmd);
            rightFly.setPower(flyCmd);
            frontIntake.setPower(0);
            backIntake.setPower(0);
            trigger.setPosition(TRIGGER_HOME);
        }
    }

    private void updateManualIntake(double vbat, double flyCmd) {
        boolean intakeToggle = gamepad1.right_trigger > 0.5;
        if (intakeToggle && !intakeTogglePrev) isIntakeOn = !isIntakeOn;
        intakeTogglePrev = intakeToggle;

        boolean modeToggle = gamepad1.dpad_left;
        if (modeToggle && !intakeModePrev) intakeModeOne = !intakeModeOne;
        intakeModePrev = modeToggle;

        boolean sideSortOk = Math.abs(sideSortPos - SIDE_SORT_CENTERED) <= 0.04;

        if (!sideSortOk) {
            frontIntake.setPower(0);
            backIntake.setPower(0);
        } else if (!preSpinLatched) {
            if (isIntakeOn) applySmartIntake(vbat, flyCmd);
            else {
                frontIntake.setPower(0);
                backIntake.setPower(0);
            }
        }
    }

    private void handleLaunchTriggers(double flyCmd, double vbat) {
        // Abort
        if (gamepad1.b && launchState != LaunchState.IDLE) {
            abortLaunch();
            return;
        }

        // Start
        if (launchState != LaunchState.IDLE || !gamepad1.a) return;

        boolean doSorted = sortingMode && patternLocked
                && (storedPatternId == 21 || storedPatternId == 22 || storedPatternId == 23);

        long start          = System.currentTimeMillis();
        long preSpinElapsed = preSpinStartMs > 0 ? start - preSpinStartMs : 0;
        spinupRemainingMs   = Math.max(0, FLYWHEEL_SPINUP_MS - preSpinElapsed);

        preSpinLatched = false;
        preSpinStartMs = 0;

        leftFly.setPower(flyCmd);
        rightFly.setPower(flyCmd);
        isIntakeOn = false;
        frontIntake.setPower(0);
        backIntake.setPower(0);
        trigger.setPosition(TRIGGER_HOME);
        shotRetryCount = 0;
        stateTimer = start;

        if (doSorted) {
            takeSortedSnapshot(storedPatternId);
            buildSortedCasePlan();
        } else {
            boolean runAll    = fullStackLatched;
            fullStackLatched  = false;
            planCenterShot    = true;
            planFrontShot     = runAll || frontHasBall;
            planBackShot      = runAll || backHasBall;
        }

        launchState = LaunchState.BURP;
    }

    // =========================================================================
    // LAUNCH STATE MACHINE
    // =========================================================================

    private void runLaunchStateMachine(double flyCmd, double vbat) {
        switch (launchState) {

            case IDLE:
                break;

            case BURP: {
                setFlyHold(flyCmd);
                trigger.setPosition(TRIGGER_HOME);
                frontIntake.setPower(intakeCommand(-SPINUP_OUTWARD_POWER, vbat));
                backIntake.setPower(intakeCommand(+SPINUP_OUTWARD_POWER, vbat));

                if (System.currentTimeMillis() - stateTimer >= SPINUP_OUTWARD_MS) {
                    stopIntakes();
                    stateTimer = System.currentTimeMillis();
                    launchState = spinupRemainingMs > 0 ? LaunchState.SPINUP : firstLaunchState();
                    if (launchState != LaunchState.SPINUP) primeFirstShot();
                }
                break;
            }

            case SPINUP: {
                setFlyHold(flyCmd);
                stopIntakes();
                trigger.setPosition(TRIGGER_HOME);

                if (System.currentTimeMillis() - stateTimer >= spinupRemainingMs) {
                    stateTimer  = System.currentTimeMillis();
                    launchState = firstLaunchState();
                    primeFirstShot();
                }
                break;
            }

            // ── Normal sequence ───────────────────────────────────────────

            case N_FIRE_1: holdFireThenReset(LaunchState.N_RESET_1, flyCmd); break;

            case N_RESET_1:
                if (retryIfCenterStillHasBall(LaunchState.N_FIRE_1, flyCmd)) break;
                stateTimer  = System.currentTimeMillis();
                launchState = planFrontShot ? LaunchState.N_FEED_2
                        : planBackShot ? LaunchState.N_FEED_3
                        : LaunchState.SPINDOWN;
                break;

            case N_FEED_2: feedThenFire(true, vbat, flyCmd, LaunchState.N_FIRE_2);   break;
            case N_FIRE_2: holdFireThenReset(LaunchState.N_RESET_2, flyCmd);          break;

            case N_RESET_2:
                if (retryIfCenterStillHasBall(LaunchState.N_FIRE_2, flyCmd)) break;
                stateTimer  = System.currentTimeMillis();
                launchState = planBackShot ? LaunchState.N_FEED_3 : LaunchState.SPINDOWN;
                break;

            case N_FEED_3: feedThenFire(false, vbat, flyCmd, LaunchState.N_FIRE_3);  break;
            case N_FIRE_3: holdFireThenReset(LaunchState.N_RESET_3, flyCmd);          break;

            case N_RESET_3:
                if (retryIfCenterStillHasBall(LaunchState.N_FIRE_3, flyCmd)) break;
                stateTimer  = System.currentTimeMillis();
                launchState = LaunchState.SPINDOWN;
                break;

            // ── Sorted sequence ───────────────────────────────────────────

            case S_STEP_BEGIN:  runSortedStepBegin(flyCmd, vbat);  break;
            case S_STOW_WAIT_CLEAR: runStowWaitClear(flyCmd);       break;
            case S_FEED_WAIT:   runSFeedWait(flyCmd, vbat);         break;
            case S_FIRE_HOLD:   runSFireHold(flyCmd);               break;
            case S_RESET_WAIT:  runSResetWait(flyCmd);              break;
            case S_RETURN_WAIT_FILL: runSReturnWaitFill(flyCmd);    break;

            case SPINDOWN: {
                setFlyHold(flyCmd);
                stopIntakes();
                trigger.setPosition(TRIGGER_HOME);

                if (System.currentTimeMillis() - stateTimer >= FLYWHEEL_SPINDOWN_MS) {
                    leftFly.setPower(0);
                    rightFly.setPower(0);
                    resetLaunchState();
                    setSideSortCentered();
                }
                break;
            }
        }
    }

    /** Returns the first non-SPINUP launch state to transition into. */
    private LaunchState firstLaunchState() {
        return sortedSteps.length > 0 ? LaunchState.S_STEP_BEGIN : LaunchState.N_FIRE_1;
    }

    /** Starts the first sorted or normal step. Only call after firstLaunchState() resolves. */
    private void primeFirstShot() {
        if (sortedSteps.length > 0) {
            sortedStepIndex = 0;
        } else {
            trigger.setPosition(TRIGGER_FIRE);
            shotRetryCount = 0;
        }
    }

    // ── Sorted state handlers ─────────────────────────────────────────────────

    private void runSortedStepBegin(double flyCmd, double vbat) {
        stopIntakes();
        setFlyHold(flyCmd);
        trigger.setPosition(TRIGGER_HOME);

        if (sortedStepIndex >= sortedSteps.length) {
            launchState = LaunchState.SPINDOWN;
            stateTimer  = System.currentTimeMillis();
            return;
        }

        switch (sortedSteps[sortedStepIndex]) {

            case STOW_CENTER_TO_SIDE:
                stowSideSort();
                stateTimer  = System.currentTimeMillis();
                launchState = LaunchState.S_STOW_WAIT_CLEAR;
                break;

            case FIRE_CENTER:
                if (!centerHasBall) { sortedStepIndex++; launchState = LaunchState.S_STEP_BEGIN; break; }
                trigger.setPosition(TRIGGER_FIRE);
                shotRetryCount = 0;
                stateTimer     = System.currentTimeMillis();
                launchState    = LaunchState.S_FIRE_HOLD;
                break;

            case FEED_FRONT_THEN_FIRE:
                frontIntake.setPower(intakeCommand(+FRONT_FEED_POWER, vbat));
                backIntake.setPower(0);
                stateTimer  = System.currentTimeMillis();
                launchState = LaunchState.S_FEED_WAIT;
                break;

            case FEED_BACK_THEN_FIRE:
                backIntake.setPower(intakeCommand(-BACK_FEED_POWER, vbat));
                frontIntake.setPower(0);
                stateTimer  = System.currentTimeMillis();
                launchState = LaunchState.S_FEED_WAIT;
                break;

            case RETURN_SIDE_TO_CENTER_THEN_FIRE:
                doShortOutwardBurp(vbat);
                setSideSortCentered();
                stateTimer  = System.currentTimeMillis();
                launchState = LaunchState.S_RETURN_WAIT_FILL;
                break;
        }
    }

    private void runStowWaitClear(double flyCmd) {
        stopIntakes();
        setFlyHold(flyCmd);
        trigger.setPosition(TRIGGER_HOME);

        long elapsed       = System.currentTimeMillis() - stateTimer;
        long motionDone    = SIDE_SORT_SLIDE_MS + SIDE_SORT_SETTLE_BEFORE_FEED_MS;
        long clearDeadline = motionDone + SIDE_SORT_WAIT_CENTER_CLEAR_MS;

        if (elapsed < motionDone) return;
        if (!centerHasBall || elapsed >= clearDeadline) {
            sortedStepIndex++;
            launchState = LaunchState.S_STEP_BEGIN;
        }
    }

    private void runSFeedWait(double flyCmd, double vbat) {
        setFlyHold(flyCmd);
        trigger.setPosition(TRIGGER_HOME);

        if (System.currentTimeMillis() - stateTimer >= FEED_TO_CENTER_MS) {
            stopIntakes();
            trigger.setPosition(TRIGGER_FIRE);
            shotRetryCount = 0;
            stateTimer     = System.currentTimeMillis();
            launchState    = LaunchState.S_FIRE_HOLD;
        }
    }

    private void runSFireHold(double flyCmd) {
        setFlyHold(flyCmd);
        stopIntakes();

        if (System.currentTimeMillis() - stateTimer >= LAUNCH_TRIGGER_HOLD_MS) {
            trigger.setPosition(TRIGGER_HOME);
            stateTimer  = System.currentTimeMillis();
            launchState = LaunchState.S_RESET_WAIT;
        }
    }

    private void runSResetWait(double flyCmd) {
        setFlyHold(flyCmd);
        stopIntakes();
        trigger.setPosition(TRIGGER_HOME);

        long elapsed = System.currentTimeMillis() - stateTimer;
        if (elapsed < TRIGGER_RESET_WAIT_MS) return;

        if (centerHasBall) {
            if (elapsed < TRIGGER_RESET_WAIT_MS + RETRY_EXTRA_WAIT_MS) return;
            if (shotRetryCount >= MAX_RETRIES_PER_SHOT) { abortLaunch(); return; }
            shotRetryCount++;
            trigger.setPosition(TRIGGER_FIRE);
            stateTimer  = System.currentTimeMillis();
            launchState = LaunchState.S_FIRE_HOLD;
            return;
        }

        shotRetryCount = 0;
        sortedStepIndex++;
        launchState = LaunchState.S_STEP_BEGIN;
    }

    private void runSReturnWaitFill(double flyCmd) {
        stopIntakes();
        setFlyHold(flyCmd);
        trigger.setPosition(TRIGGER_HOME);

        long elapsed       = System.currentTimeMillis() - stateTimer;
        long motionDone    = SIDE_SORT_SLIDE_MS + SIDE_SORT_SETTLE_BEFORE_FEED_MS;
        long fillDeadline  = motionDone + SIDE_SORT_WAIT_CENTER_FILL_MS;

        if (elapsed < motionDone) return;

        if (centerHasBall || elapsed >= fillDeadline) {
            if (!centerHasBall) { sortedStepIndex++; launchState = LaunchState.S_STEP_BEGIN; return; }
            trigger.setPosition(TRIGGER_FIRE);
            shotRetryCount = 0;
            stateTimer     = System.currentTimeMillis();
            launchState    = LaunchState.S_FIRE_HOLD;
        }
    }

    // ── Normal feed helper ────────────────────────────────────────────────────

    /** Feeds from front (fromFront=true) or back, waits FEED_TO_CENTER_MS, then fires. */
    private void feedThenFire(boolean fromFront, double vbat, double flyCmd, LaunchState fireState) {
        setFlyHold(flyCmd);
        trigger.setPosition(TRIGGER_HOME);

        if (fromFront) {
            frontIntake.setPower(intakeCommand(+FRONT_FEED_POWER, vbat));
            backIntake.setPower(0);
        } else {
            backIntake.setPower(intakeCommand(-BACK_FEED_POWER, vbat));
            frontIntake.setPower(0);
        }

        if (System.currentTimeMillis() - stateTimer >= FEED_TO_CENTER_MS) {
            stopIntakes();
            trigger.setPosition(TRIGGER_FIRE);
            shotRetryCount = 0;
            stateTimer  = System.currentTimeMillis();
            launchState = fireState;
        }
    }

    // =========================================================================
    // SORTED CASE: SNAPSHOT + PLAN BUILDER
    // =========================================================================

    private void takeSortedSnapshot(int patternId) {
        snapPatternId = patternId;
        snapF  = frontHasBall;
        snapC  = centerHasBall;
        snapB  = backHasBall;
        snapFC = snapF ? frontBallColor  : BallColor.EMPTY;
        snapCC = snapC ? centerBallColor : BallColor.EMPTY;
        snapBC = snapB ? backBallColor   : BallColor.EMPTY;

        if (snapF && snapFC == BallColor.UNKNOWN) snapFC = BallColor.GREEN;
        if (snapC && snapCC == BallColor.UNKNOWN) snapCC = BallColor.GREEN;
        if (snapB && snapBC == BallColor.UNKNOWN) snapBC = BallColor.GREEN;
    }

    private void buildSortedCasePlan() {
        sortedCase   = SortedCase.NONE;
        sortedSteps  = new SortedStep[0];
        sortedStepIndex = 0;

        boolean gC = snapC && snapCC == BallColor.GREEN;
        boolean gF = snapF && snapFC == BallColor.GREEN;
        boolean gB = snapB && snapBC == BallColor.GREEN;

        switch (snapPatternId) {
            case 23:
                if (gC) {
                    sortedCase  = SortedCase.P23_GREEN_CENTER;
                    sortedSteps = new SortedStep[]{
                            SortedStep.STOW_CENTER_TO_SIDE,
                            SortedStep.FEED_FRONT_THEN_FIRE,
                            SortedStep.FEED_BACK_THEN_FIRE,
                            SortedStep.RETURN_SIDE_TO_CENTER_THEN_FIRE
                    };
                } else if (gB) {
                    sortedCase  = SortedCase.P23_GREEN_BACK;
                    sortedSteps = new SortedStep[]{
                            SortedStep.FIRE_CENTER,
                            SortedStep.FEED_FRONT_THEN_FIRE,
                            SortedStep.FEED_BACK_THEN_FIRE
                    };
                } else if (gF) {
                    sortedCase  = SortedCase.P23_GREEN_FRONT;
                    sortedSteps = new SortedStep[]{
                            SortedStep.FIRE_CENTER,
                            SortedStep.FEED_BACK_THEN_FIRE,
                            SortedStep.FEED_FRONT_THEN_FIRE
                    };
                } else {
                    sortedSteps = new SortedStep[]{
                            SortedStep.FIRE_CENTER,
                            SortedStep.FEED_FRONT_THEN_FIRE,
                            SortedStep.FEED_BACK_THEN_FIRE
                    };
                }
                break;

            case 22:
                if (gC) {
                    sortedCase  = SortedCase.P22_GREEN_CENTER;
                    sortedSteps = new SortedStep[]{
                            SortedStep.STOW_CENTER_TO_SIDE,
                            SortedStep.FEED_FRONT_THEN_FIRE,
                            SortedStep.RETURN_SIDE_TO_CENTER_THEN_FIRE,
                            SortedStep.FIRE_CENTER,
                            SortedStep.FEED_BACK_THEN_FIRE
                    };
                } else if (gB) {
                    sortedCase  = SortedCase.P22_GREEN_BACK;
                    sortedSteps = new SortedStep[]{
                            SortedStep.FIRE_CENTER,
                            SortedStep.FEED_BACK_THEN_FIRE,
                            SortedStep.FEED_FRONT_THEN_FIRE
                    };
                } else if (gF) {
                    sortedCase  = SortedCase.P22_GREEN_FRONT;
                    sortedSteps = new SortedStep[]{
                            SortedStep.FIRE_CENTER,
                            SortedStep.FEED_FRONT_THEN_FIRE,
                            SortedStep.FEED_BACK_THEN_FIRE
                    };
                } else {
                    sortedSteps = new SortedStep[]{
                            SortedStep.FIRE_CENTER,
                            SortedStep.FEED_FRONT_THEN_FIRE,
                            SortedStep.FEED_BACK_THEN_FIRE
                    };
                }
                break;

            case 21:
                if (gC) {
                    sortedCase  = SortedCase.P21_GREEN_CENTER;
                    sortedSteps = new SortedStep[]{
                            SortedStep.FIRE_CENTER,
                            SortedStep.FEED_FRONT_THEN_FIRE,
                            SortedStep.FEED_BACK_THEN_FIRE
                    };
                } else if (gF) {
                    sortedCase  = SortedCase.P21_GREEN_FRONT;
                    sortedSteps = new SortedStep[]{
                            SortedStep.STOW_CENTER_TO_SIDE,
                            SortedStep.FEED_FRONT_THEN_FIRE,
                            SortedStep.FEED_BACK_THEN_FIRE,
                            SortedStep.RETURN_SIDE_TO_CENTER_THEN_FIRE
                    };
                } else if (gB) {
                    sortedCase  = SortedCase.P21_GREEN_BACK;
                    sortedSteps = new SortedStep[]{
                            SortedStep.STOW_CENTER_TO_SIDE,
                            SortedStep.FEED_BACK_THEN_FIRE,
                            SortedStep.FEED_FRONT_THEN_FIRE,
                            SortedStep.RETURN_SIDE_TO_CENTER_THEN_FIRE
                    };
                } else {
                    sortedSteps = new SortedStep[]{
                            SortedStep.FIRE_CENTER,
                            SortedStep.FEED_FRONT_THEN_FIRE,
                            SortedStep.FEED_BACK_THEN_FIRE
                    };
                }
                break;
        }
    }

    // =========================================================================
    // DRIVE
    // =========================================================================

    /**
     * Computes swerve drive velocities and target angles.
     * Returns [angleFL, angleFR, angleBL, angleBR, speedFL, speedFR, speedBL, speedBR].
     */
    private double[] computeDrive(double speedMultiplier) {
        double fieldY = -gamepad1.left_stick_y * speedMultiplier;
        double fieldX =  gamepad1.left_stick_x * speedMultiplier;
        double rot    =  gamepad1.right_stick_x * speedMultiplier;

        double rawHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double heading    = wrapAngle(rawHeading);
        double robotX     = fieldX * Math.cos(-heading) - fieldY * Math.sin(-heading);
        double robotY     = fieldX * Math.sin(-heading) + fieldY * Math.cos(-heading);

        double A = robotX - rot * (WHEELBASE  / R);
        double B = robotX + rot * (WHEELBASE  / R);
        double C = robotY - rot * (TRACK_WIDTH / R);
        double D = robotY + rot * (TRACK_WIDTH / R);

        double sFL = Math.hypot(B, D);
        double sFR = Math.hypot(B, C);
        double sBL = Math.hypot(A, D);
        double sBR = Math.hypot(A, C);

        double maxS = Math.max(Math.max(sFL, sFR), Math.max(sBL, sBR));
        if (maxS > 1.0) { sFL /= maxS; sFR /= maxS; sBL /= maxS; sBR /= maxS; }

        double aFL, aFR, aBL, aBR;

        if (Math.abs(robotX) > DRIVE_DEADBAND || Math.abs(robotY) > DRIVE_DEADBAND || Math.abs(rot) > DRIVE_DEADBAND) {
            aFL = Math.atan2(B, D);
            aFR = Math.atan2(B, C);
            aBL = Math.atan2(A, D);
            aBR = Math.atan2(A, C);
            framesSinceLastMoved = 0;
        } else {
            sFL = sFR = sBL = sBR = 0;
            framesSinceLastMoved++;
            // Use previous target angles (will be filled by caller)
            aFL = aFR = aBL = aBR = Double.NaN;
        }

        // X-snap (plant wheels)
        if (gamepad1.left_stick_button || framesSinceLastMoved >= FRAMES_TO_PLANT) {
            aFL = -Math.PI / 4; aFR =  Math.PI / 4;
            aBL =  Math.PI / 4; aBR = -Math.PI / 4;
            sFL = sFR = sBL = sBR = 0;
        }

        return new double[]{ aFL, aFR, aBL, aBR, sFL, sFR, sBL, sBR };
    }

    // =========================================================================
    // VISION HELPERS
    // =========================================================================

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
        for (AprilTagDetection d : aprilTag.getDetections()) {
            if (d != null && d.id == id && d.ftcPose != null) return d;
        }
        return null;
    }

    private double getTag24RangeInches() {
        AprilTagDetection d = getFirstTagById(24);
        return d == null ? Double.NaN : d.ftcPose.range;
    }

    private double getTag24BearingDeg() {
        AprilTagDetection d = getFirstTagById(24);
        return d == null ? Double.NaN : d.ftcPose.bearing;
    }

    private void tryLockPattern21_22_23() {
        if (patternLocked || aprilTag == null) return;
        for (AprilTagDetection d : aprilTag.getDetections()) {
            if (d == null) continue;
            if (d.id == 21 || d.id == 22 || d.id == 23) {
                storedPatternId = d.id;
                patternLocked   = true;
                return;
            }
        }
    }

    private void updateLightServo(boolean tagVisible, double tagBearingDeg) {
        if (light == null) return;
        if (!tagVisible) {
            light.setPosition(LIGHT_NO_TAG);
        } else if (!Double.isNaN(tagBearingDeg) && Math.abs(tagBearingDeg) <= LIGHT_LOCK_DEG) {
            light.setPosition(LIGHT_TAG_LOCKED);
        } else {
            light.setPosition(LIGHT_TAG_SEEN);
        }
    }

    // =========================================================================
    // SMALL HELPERS
    // =========================================================================

    private void holdFireThenReset(LaunchState nextResetState, double flyCmd) {
        setFlyHold(flyCmd);
        stopIntakes();
        if (System.currentTimeMillis() - stateTimer >= LAUNCH_TRIGGER_HOLD_MS) {
            trigger.setPosition(TRIGGER_HOME);
            stateTimer  = System.currentTimeMillis();
            launchState = nextResetState;
        }
    }

    private boolean retryIfCenterStillHasBall(LaunchState fireStateToRetry, double flyCmd) {
        setFlyHold(flyCmd);
        stopIntakes();
        trigger.setPosition(TRIGGER_HOME);

        long elapsed = System.currentTimeMillis() - stateTimer;
        if (elapsed < TRIGGER_RESET_WAIT_MS) return true;

        if (centerHasBall) {
            if (elapsed < TRIGGER_RESET_WAIT_MS + RETRY_EXTRA_WAIT_MS) return true;
            if (shotRetryCount >= MAX_RETRIES_PER_SHOT) { abortLaunch(); return true; }
            shotRetryCount++;
            trigger.setPosition(TRIGGER_FIRE);
            stateTimer  = System.currentTimeMillis();
            launchState = fireStateToRetry;
            return true;
        }

        shotRetryCount = 0;
        return false;
    }

    private void setFlyHold(double flyCmd) {
        leftFly.setPower(flyCmd);
        rightFly.setPower(flyCmd);
    }

    private void stopIntakes() {
        frontIntake.setPower(0);
        backIntake.setPower(0);
    }

    private void setSideSortCentered() {
        sideSortPos = SIDE_SORT_CENTERED;
        if (sideSort != null) sideSort.setPosition(sideSortPos);
    }

    private void stowSideSort() {
        sideSortPos = SIDE_SORT_STOWED;
        if (sideSort != null) sideSort.setPosition(sideSortPos);
    }

    private void doShortOutwardBurp(double vbat) {
        frontIntake.setPower(intakeCommand(-SPINUP_OUTWARD_POWER, vbat));
        backIntake.setPower(intakeCommand(+SPINUP_OUTWARD_POWER, vbat));
        sleep(SHORT_BURP_MS);
        stopIntakes();
    }

    private void resetLaunchState() {
        launchState       = LaunchState.IDLE;
        spinupRemainingMs = 0;
        planCenterShot    = false;
        planFrontShot     = false;
        planBackShot      = false;
        sortedSteps       = new SortedStep[0];
        sortedStepIndex   = 0;
        sortedCase        = SortedCase.NONE;
    }

    private void abortLaunch() {
        stopIntakes();
        leftFly.setPower(0);
        rightFly.setPower(0);
        trigger.setPosition(TRIGGER_HOME);

        preSpinLatched = false;
        preSpinStartMs = 0;
        ltPrev         = false;
        fullStackLatched = false;
        shotRetryCount = 0;
        resetLaunchState();
        setSideSortCentered();
    }

    // =========================================================================
    // VOLTAGE COMPENSATION
    // =========================================================================

    private double flywheelCommand(double vbat) {
        if (!Double.isFinite(vbat) || vbat <= 0.5) return multiplier;
        double base = vbat < REF_VOLTAGE ? 1.0 : REF_VOLTAGE / vbat;
        return clamp(multiplier * base, 0.0, 1.0);
    }

    private double intakeVoltageScale(double vbat) {
        if (!Double.isFinite(vbat) || vbat <= 0.5) return 1.0;
        return clamp(INTAKE_REF_VOLTAGE / vbat, 0.0, 1.0);
    }

    private double intakeCommand(double desiredPower, double vbat) {
        return clamp(desiredPower * intakeVoltageScale(vbat), -1.0, 1.0);
    }

    // =========================================================================
    // SMART INTAKE
    // =========================================================================

    private void applySmartIntake(double vbat, double flyCmd) {
        DcMotor fastMotor = intakeModeOne ? frontIntake : backIntake;
        DcMotor slowMotor = intakeModeOne ? backIntake  : frontIntake;
        double  dir       = intakeModeOne ? +1.0 : -1.0;

        boolean fastBall = intakeModeOne ? frontHasBall : backHasBall;
        boolean slowBall = intakeModeOne ? backHasBall  : frontHasBall;

        double slowPower = slowBall ? 0.0 : dir * MANUAL_SLOW;
        double fastPower;

        if (!slowBall) {
            fastPower = dir * MANUAL_FAST;
        } else {
            fastPower = dir * (centerHasBall ? FAST_AFTER_SLOW_AND_CENTER : FAST_AFTER_SLOW);

            if (centerHasBall && fastBall) {
                fastPower = 0.0;
                slowPower = 0.0;
                isIntakeOn      = false;
                fullStackLatched = true;

                if (AUTO_PRESPIN_WHEN_FULL && launchState == LaunchState.IDLE) {
                    preSpinLatched = true;
                    preSpinStartMs = System.currentTimeMillis();
                    setFlyHold(flyCmd);
                }
            }
        }

        fastMotor.setPower(intakeCommand(fastPower, vbat));
        slowMotor.setPower(intakeCommand(slowPower, vbat));
    }

    // =========================================================================
    // COLOR CLASSIFICATION
    // =========================================================================

    private BallColor classifyBallColor(NormalizedColorSensor s, Slot slot, boolean hasBall) {
        if (!hasBall) return BallColor.EMPTY;
        if (s == null) return BallColor.UNKNOWN;

        NormalizedRGBA c = s.getNormalizedColors();
        double scoreGreen  = c.green + GREEN_B_WEIGHT  * c.blue;
        double scorePurple = c.red   + PURPLE_B_WEIGHT * c.blue;

        switch (slot) {
            case FRONT:  lastFrontGScore  = scoreGreen; lastFrontPScore  = scorePurple; break;
            case CENTER: lastCenterGScore = scoreGreen; lastCenterPScore = scorePurple; break;
            default:     lastBackGScore   = scoreGreen; lastBackPScore   = scorePurple; break;
        }

        if (scoreGreen  > scorePurple * GREEN_RATIO)  return BallColor.GREEN;
        if (scorePurple > scoreGreen  * PURPLE_RATIO) return BallColor.PURPLE;
        return BallColor.UNKNOWN;
    }

    // =========================================================================
    // SWERVE MODULE
    // =========================================================================

    private void runModule(DcMotor drive, CRServo steer, AnalogInput enc,
                           double offset, double speed, double target) {
        if (Double.isNaN(target)) { if (drive != null) drive.setPower(0); return; }

        double current    = wrapAngle(getRawAngle(enc) - offset);
        double delta      = wrapAngle(target - current);

        if (Math.abs(delta) > Math.PI / 2) {
            delta  = wrapAngle(delta + Math.PI);
            speed *= -1;
        }

        double steerPower = clamp(-STEER_KP * delta, -1, 1);
        if (Math.abs(steerPower) < STEER_DEADBAND) steerPower = 0;

        if (steer != null) steer.setPower(steerPower);
        if (drive != null) drive.setPower(speed);
    }

    // =========================================================================
    // DISTANCE / HYSTERESIS
    // =========================================================================

    private double safeDistanceCm(DistanceSensor s) {
        if (s == null) return 999.0;
        double cm = s.getDistance(DistanceUnit.CM);
        return Double.isFinite(cm) ? cm : 999.0;
    }

    private boolean hysteresisBall(boolean prev, double cm, double onCm, double offCm) {
        return prev ? (cm <= offCm) : (cm <= onCm);
    }

    // =========================================================================
    // MATH UTILITIES
    // =========================================================================

    private double getRawAngle(AnalogInput enc) {
        return enc == null ? 0.0 : enc.getVoltage() / 3.3 * (2 * Math.PI);
    }

    private double wrapAngle(double a) {
        while (a >  Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    // =========================================================================
    // TELEMETRY
    // =========================================================================

    private void updateTelemetry() {
        double tagRangeIn    = getTag24RangeInches();
        double tagBearingDeg = getTag24BearingDeg();
        boolean tagVisible   = !Double.isNaN(tagRangeIn);

        telemetry.addData("SortingMode",    sortingMode);
        telemetry.addData("PatternLocked",  "%s (%d)", patternLocked, storedPatternId);
        telemetry.addData("BallsPresent",   "F:%s C:%s B:%s", frontHasBall, centerHasBall, backHasBall);
        telemetry.addData("BallColors",     "F:%s C:%s B:%s", frontBallColor, centerBallColor, backBallColor);
        telemetry.addData("SortedCase",     sortedCase);
        telemetry.addData("SortedStep",     "%d/%d", sortedStepIndex, sortedSteps.length);
        telemetry.addData("Tag24",          tagVisible ? "VISIBLE" : "NO");
        telemetry.addData("Tag24 range(in)",    "%.2f", tagRangeIn);
        telemetry.addData("Tag24 bearing(deg)", "%.2f", tagBearingDeg);
        telemetry.addData("LightPos",       light != null ? "%.3f" : "null", light != null ? light.getPosition() : 0.0);
        telemetry.addData("TurretDeg",      "%.1f (0..270)", turretDeg);
        telemetry.addData("YawDeg",         "%.1f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("SideSortPos",    "%.3f", sideSortPos);
        telemetry.addData("F scores",       "G=%.3f P=%.3f", lastFrontGScore,  lastFrontPScore);
        telemetry.addData("C scores",       "G=%.3f P=%.3f", lastCenterGScore, lastCenterPScore);
        telemetry.addData("B scores",       "G=%.3f P=%.3f", lastBackGScore,   lastBackPScore);
        telemetry.addData("FullStackLatched", fullStackLatched);
        telemetry.addData("PreSpin",        preSpinLatched);
        telemetry.addData("LaunchState",    launchState);
        telemetry.update();
    }

    // =========================================================================
    // HARDWARE INIT
    // =========================================================================

    private void initHardware() {
        // Drive motors
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive  = hardwareMap.get(DcMotor.class, "backRightDrive");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Steer servos
        frontLeftSteer  = hardwareMap.get(CRServo.class, "frontLeftSteer");
        frontRightSteer = hardwareMap.get(CRServo.class, "frontRightSteer");
        backLeftSteer   = hardwareMap.get(CRServo.class, "backLeftSteer");
        backRightSteer  = hardwareMap.get(CRServo.class, "backRightSteer");

        // Encoders
        frontLeftEncoder  = hardwareMap.get(AnalogInput.class, "frontLeftEncoder");
        frontRightEncoder = hardwareMap.get(AnalogInput.class, "frontRightEncoder");
        backLeftEncoder   = hardwareMap.get(AnalogInput.class, "backLeftEncoder");
        backRightEncoder  = hardwareMap.get(AnalogInput.class, "backRightEncoder");

        // IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        // Voltage sensor
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Mechanism motors
        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
        backIntake  = hardwareMap.get(DcMotor.class, "backIntake");

        leftFly  = hardwareMap.get(DcMotor.class, "leftFly");
        rightFly = hardwareMap.get(DcMotor.class, "rightFly");
        leftFly.setDirection(DcMotor.Direction.REVERSE);
        rightFly.setDirection(DcMotor.Direction.FORWARD);

        // Servos
        turretRotation1 = hardwareMap.get(Servo.class, "turret_rotation_1");
        turretRotation2 = hardwareMap.get(Servo.class, "turret_rotation_2");
        turretRotation1.setDirection(Servo.Direction.FORWARD);
        turretRotation2.setDirection(Servo.Direction.REVERSE);

        trigger  = hardwareMap.get(Servo.class, "trigger");
        adjuster = hardwareMap.get(Servo.class, "adjuster");
        sideSort = hardwareMap.get(Servo.class, "side_sort");
        light    = hardwareMap.get(Servo.class, "lights");

        // Color / distance sensors
        try {
            RevColorSensorV3 f = hardwareMap.get(RevColorSensorV3.class, "frontColor");
            RevColorSensorV3 c = hardwareMap.get(RevColorSensorV3.class, "centerColor");
            RevColorSensorV3 b = hardwareMap.get(RevColorSensorV3.class, "backColor");
            frontColor  = f; centerColor  = c; backColor  = b;
            frontDist   = f; centerDist   = c; backDist   = b;
        } catch (Exception e) {
            frontColor  = hardwareMap.get(NormalizedColorSensor.class, "frontColor");
            centerColor = hardwareMap.get(NormalizedColorSensor.class, "centerColor");
            backColor   = hardwareMap.get(NormalizedColorSensor.class, "backColor");
            frontDist   = hardwareMap.get(DistanceSensor.class, "frontColor");
            centerDist  = hardwareMap.get(DistanceSensor.class, "centerColor");
            backDist    = hardwareMap.get(DistanceSensor.class, "backColor");
        }
    }
}
