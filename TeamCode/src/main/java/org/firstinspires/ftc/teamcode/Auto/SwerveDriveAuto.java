package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Auto.Waypoint.ActionTiming;

/**
 * SwerveDriveAuto
 * ───────────────────────────────────────────────────────────────────────────
 * Autonomous drive controller for the coaxial swerve drivetrain.
 * Uses the goBUILDA Pinpoint computer for localisation.
 *
 * Move sequence for every waypoint:
 *   1. SNAP    — wheels rotate to target angles, drive motors off
 *   2. ROTATE  — robot turns in place to target heading (if needed)
 *   3. DRIVE   — trapezoidal translation, heading held via PID correction
 *   4. ACTION  — optional mechanism callback fires on arrival
 */
public class SwerveDriveAuto {

    // ═════════════════════════════════════════════════════════════════════════
    //  TUNING — adjust these for your robot
    // ═════════════════════════════════════════════════════════════════════════

    // Heading PID — used for both rotate-in-place and drift correction while driving
    private static final double HEAD_KP = 0.5;
    private static final double HEAD_KI = 0.0;
    private static final double HEAD_KD = 0.04;

    // Translation PID — corrects minor overshoot at the end of a move
    private static final double TRANS_KP = 0.045;
    private static final double TRANS_KI = 0.0;
    private static final double TRANS_KD = 0.003;

    // Peak drive power during cruise phase (0…1)
    private static final double MAX_DRIVE_POWER = 0.70;

    // Distance tolerance — considered "arrived" when within this many inches
    private static final double POSITION_TOLERANCE_IN = 0.75;

    // Heading tolerance — considered "aligned" within this many radians (~1.7°)
    private static final double HEADING_TOLERANCE_RAD = 0.03;

    // Max rotation power during rotate-in-place phase
    private static final double MAX_ROTATE_POWER = 0.50;

    // Max heading-correction power blended in during translation
    private static final double MAX_HEADING_CORRECTION = 0.25;

    // How close each wheel must be to its target angle before drive starts (radians)
    // ~5.7° — tight enough for accuracy, loose enough not to wait forever
    private static final double SNAP_TOLERANCE_RAD = 0.10;

    // Maximum time to wait for wheels to snap into place (ms)
    // Safety valve — prevents hanging if a servo is stuck
    private static final long SNAP_TIMEOUT_MS = 600;

    // ═════════════════════════════════════════════════════════════════════════
    //  ROBOT GEOMETRY
    // ═════════════════════════════════════════════════════════════════════════
    private static final double TRACK_WIDTH = 17.258;
    private static final double WHEELBASE   = 13.544;
    private static final double R           = Math.hypot(TRACK_WIDTH, WHEELBASE);

    final double FRONT_LEFT_OFFSET  = 0.1200;
    final double FRONT_RIGHT_OFFSET = 1.3861;
    final double BACK_LEFT_OFFSET   = 1.6965;
    final double BACK_RIGHT_OFFSET  = 0.8225;
    // ═════════════════════════════════════════════════════════════════════════
    //  HARDWARE
    // ═════════════════════════════════════════════════════════════════════════
    private final LinearOpMode opMode;
    private SwerveModule frontLeft, frontRight, backLeft, backRight;
    private PinpointLocalizer localizer;

    // ═════════════════════════════════════════════════════════════════════════
    //  PID CONTROLLERS
    // ═════════════════════════════════════════════════════════════════════════
    private final PIDController headingPID     = new PIDController(HEAD_KP,  HEAD_KI,  HEAD_KD,  0.3);
    private final PIDController translationPID = new PIDController(TRANS_KP, TRANS_KI, TRANS_KD, 0.4);

    // ═════════════════════════════════════════════════════════════════════════
    //  POSE STATE
    // ═════════════════════════════════════════════════════════════════════════
    private double poseX       = 0;
    private double poseY       = 0;
    private double poseHeading = 0;

    // ─────────────────────────────────────────────────────────────────────────
    public SwerveDriveAuto(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  INITIALISATION
    // ═════════════════════════════════════════════════════════════════════════

    public void init(double startX, double startY, double startHeading) {
        HardwareMap hw = opMode.hardwareMap;

        frontLeft  = new SwerveModule(
                hw.get(DcMotor.class,     "frontLeftDrive"),
                hw.get(CRServo.class,     "frontLeftSteer"),
                hw.get(AnalogInput.class, "frontLeftEncoder"),
                FRONT_LEFT_OFFSET);

        frontRight = new SwerveModule(
                hw.get(DcMotor.class,     "frontRightDrive"),
                hw.get(CRServo.class,     "frontRightSteer"),
                hw.get(AnalogInput.class, "frontRightEncoder"),
                FRONT_RIGHT_OFFSET);

        backLeft   = new SwerveModule(
                hw.get(DcMotor.class,     "backLeftDrive"),
                hw.get(CRServo.class,     "backLeftSteer"),
                hw.get(AnalogInput.class, "backLeftEncoder"),
                BACK_LEFT_OFFSET);

        backRight  = new SwerveModule(
                hw.get(DcMotor.class,     "backRightDrive"),
                hw.get(CRServo.class,     "backRightSteer"),
                hw.get(AnalogInput.class, "backRightEncoder"),
                BACK_RIGHT_OFFSET);

        // Motor directions — match justSwerve
        hw.get(DcMotor.class, "frontLeftDrive") .setDirection(DcMotor.Direction.REVERSE);
        hw.get(DcMotor.class, "backLeftDrive")  .setDirection(DcMotor.Direction.REVERSE);
        hw.get(DcMotor.class, "frontRightDrive").setDirection(DcMotor.Direction.FORWARD);
        hw.get(DcMotor.class, "backRightDrive") .setDirection(DcMotor.Direction.FORWARD);

        hw.get(DcMotor.class, "frontLeftDrive") .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.get(DcMotor.class, "frontRightDrive").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.get(DcMotor.class, "backLeftDrive")  .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.get(DcMotor.class, "backRightDrive") .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        for (DcMotor m : new DcMotor[]{
                hw.get(DcMotor.class, "frontLeftDrive"),
                hw.get(DcMotor.class, "frontRightDrive"),
                hw.get(DcMotor.class, "backLeftDrive"),
                hw.get(DcMotor.class, "backRightDrive")}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        localizer = new PinpointLocalizer(opMode.hardwareMap);
        localizer.setPosition(startX, startY, startHeading);

        poseX       = startX;
        poseY       = startY;
        poseHeading = startHeading;
    }

    public void init() { init(0, 0, 0); }

    // ═════════════════════════════════════════════════════════════════════════
    //  PUBLIC API
    // ═════════════════════════════════════════════════════════════════════════

    /** Drive to an absolute field position and heading. Blocks until arrived. */
    public void driveTo(double x, double y, double heading) {
        executeMoveToTarget(x, y, heading, null, ActionTiming.ON_ARRIVE, 0);
    }

    /** Drive relative to current pose. */
    public void driveRelative(double dx, double dy, double dh) {
        updatePose();
        driveTo(poseX + dx, poseY + dy, poseHeading + dh);
    }

    /** Follow an ordered list of waypoints with optional per-waypoint actions. */
    public void followPath(Waypoint[] waypoints) {
        for (Waypoint wp : waypoints) {
            if (!opMode.opModeIsActive()) break;
            executeMoveToTarget(wp.x, wp.y, wp.heading,
                    wp.action, wp.timing, wp.approachDistance);
        }
    }

    /** Run a mechanism action inline and wait for it to finish. */
    public void runAction(Runnable action) {
        if (action != null) action.run();
    }

    /** Rotate in place to a heading without translating. */
    public void rotateTo(double heading) {
        updatePose();
        driveTo(poseX, poseY, heading);
    }

    /** Stop and X-lock all wheels. */
    public void stopAndLock() {
        frontLeft .lockAt(-Math.PI / 4);
        frontRight.lockAt( Math.PI / 4);
        backLeft  .lockAt( Math.PI / 4);
        backRight .lockAt(-Math.PI / 4);
    }

    public void updatePose() {
        localizer.update();
        poseX       = localizer.getX();
        poseY       = localizer.getY();
        poseHeading = localizer.getHeading();
    }

    public double getPoseX()       { return poseX; }
    public double getPoseY()       { return poseY; }
    public double getPoseHeading() { return poseHeading; }

    // ═════════════════════════════════════════════════════════════════════════
    //  CORE MOVE EXECUTION
    // ═════════════════════════════════════════════════════════════════════════

    private void executeMoveToTarget(double targetX, double targetY, double targetHeading,
                                     Runnable action, ActionTiming timing, double approachDist) {
        updatePose();

        double totalDist      = Math.hypot(targetX - poseX, targetY - poseY);
        boolean isPureRotation = totalDist < 0.5;

        // Direction wheels need to point for this move
        double travelAngle = isPureRotation ? 0
                : Math.atan2(targetY - poseY, targetX - poseX);

        // ── PHASE 1: SNAP ─────────────────────────────────────────────────────
        // Rotate all wheels to their target angles with drive motors off.
        // This ensures no unintended motion when drive power is first applied.
        if (!isPureRotation) {
            snapWheelsToAngle(travelAngle, targetHeading);
        } else {
            // For pure rotation, snap wheels to tangential rotation positions
            snapWheelsToRotation();
        }

        // ── PHASE 2: ROTATE ───────────────────────────────────────────────────
        // Turn robot to target heading in place before translating.
        double headingError = wrapAngle(targetHeading - poseHeading);
        if (Math.abs(headingError) > HEADING_TOLERANCE_RAD) {
            rotateToHeading(targetHeading);
        }

        if (isPureRotation) {
            // Pure rotation is complete — fire action and return
            if (action != null && timing == ActionTiming.ON_ARRIVE) action.run();
            return;
        }

        // ── PHASE 3: DRIVE ────────────────────────────────────────────────────
        // Re-read pose after rotation (heading has changed)
        updatePose();
        translationPID.reset();
        headingPID.reset();

        double startX    = poseX;
        double startY    = poseY;
        // Recompute total distance from actual post-rotation position
        totalDist        = Math.hypot(targetX - startX, targetY - startY);
        TrapezoidProfile profile = new TrapezoidProfile(totalDist, MAX_DRIVE_POWER);

        boolean approachActionFired = false;

        while (opMode.opModeIsActive()) {
            updatePose();

            double remainingDist = Math.hypot(targetX - poseX, targetY - poseY);
            double travelledDist = totalDist - remainingDist;

            // ON_APPROACH action
            if (!approachActionFired
                    && action != null
                    && timing == ActionTiming.ON_APPROACH
                    && remainingDist <= approachDist) {
                action.run();
                approachActionFired = true;
            }

            // Termination
            headingError         = wrapAngle(targetHeading - poseHeading);
            boolean atPosition   = remainingDist < POSITION_TOLERANCE_IN;
            boolean atHeading    = Math.abs(headingError) < HEADING_TOLERANCE_RAD;
            if (atPosition && atHeading) break;

            // Profile speed
            double profileSpeed  = atPosition ? 0 : profile.getVelocity(travelledDist);

            // Recompute travel angle each loop so we correct for any drift
            double currentTravelAngle = Math.atan2(targetY - poseY, targetX - poseX);
            double driveX = profileSpeed * Math.cos(currentTravelAngle);
            double driveY = profileSpeed * Math.sin(currentTravelAngle);

            // Heading drift correction (small correction only — wheels already aligned)
            double correction = headingPID.calculate(headingError, System.nanoTime());
            correction = Math.max(-MAX_HEADING_CORRECTION, Math.min(MAX_HEADING_CORRECTION, correction));

            setSwerveOutputs(driveX, driveY, correction, poseHeading);

            opMode.telemetry.addData("Phase",     "Driving");
            opMode.telemetry.addData("Target",    "(%.1f, %.1f, %.1f°)", targetX, targetY, Math.toDegrees(targetHeading));
            opMode.telemetry.addData("Pose",      "(%.1f, %.1f, %.1f°)", poseX, poseY, Math.toDegrees(poseHeading));
            opMode.telemetry.addData("Remaining", "%.2f in", remainingDist);
            opMode.telemetry.addData("Speed",     "%.2f", profileSpeed);
            opMode.telemetry.update();
        }

        // Stop
        frontLeft.stop(); frontRight.stop();
        backLeft.stop();  backRight.stop();

        // ON_ARRIVE action
        if (action != null && timing == ActionTiming.ON_ARRIVE) action.run();
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  PHASE HELPERS
    // ═════════════════════════════════════════════════════════════════════════

    /**
     * SNAP phase — command all wheels to the travel angle and wait until they
     * are physically within SNAP_TOLERANCE_RAD, then hold for one more loop
     * to confirm they're settled. Drive motors are zero throughout.
     */
    private void snapWheelsToAngle(double travelAngle, double targetHeading) {
        long startMs = System.currentTimeMillis();

        // Compute per-module angles accounting for robot-relative transform
        // (heading correction is 0 during snap, so we just need the travel direction)
        double[] angles = computeModuleAngles(
                Math.cos(travelAngle), Math.sin(travelAngle), 0, poseHeading);

        while (opMode.opModeIsActive()) {
            // Command servos to target angles, drive power = 0
            frontLeft .set(0, angles[0]);
            frontRight.set(0, angles[1]);
            backLeft  .set(0, angles[2]);
            backRight .set(0, angles[3]);

            // Check if all modules are within tolerance
            boolean flReady = Math.abs(wrapAngle(frontLeft .getCurrentAngle() - angles[0])) < SNAP_TOLERANCE_RAD;
            boolean frReady = Math.abs(wrapAngle(frontRight.getCurrentAngle() - angles[1])) < SNAP_TOLERANCE_RAD;
            boolean blReady = Math.abs(wrapAngle(backLeft  .getCurrentAngle() - angles[2])) < SNAP_TOLERANCE_RAD;
            boolean brReady = Math.abs(wrapAngle(backRight .getCurrentAngle() - angles[3])) < SNAP_TOLERANCE_RAD;

            opMode.telemetry.addData("Phase", "Snapping wheels");
            opMode.telemetry.addData("FL ready", flReady);
            opMode.telemetry.addData("FR ready", frReady);
            opMode.telemetry.addData("BL ready", blReady);
            opMode.telemetry.addData("BR ready", brReady);
            opMode.telemetry.update();

            if (flReady && frReady && blReady && brReady) break;

            // Safety timeout — don't hang forever if a servo is stuck
            if (System.currentTimeMillis() - startMs > SNAP_TIMEOUT_MS) break;
        }
    }

    /**
     * Snap wheels to tangential rotation positions (for pure rotate-in-place).
     * FL/BR point at +45°, FR/BL point at -45° (forms an X that spins the robot).
     */
    private void snapWheelsToRotation() {
        long startMs = System.currentTimeMillis();

        double angleFL = -Math.PI / 4;   //  -45°
        double angleFR =  Math.PI / 4;   //  +45°
        double angleBL =  Math.PI / 4;   //  +45°
        double angleBR = -Math.PI / 4;   //  -45°

        while (opMode.opModeIsActive()) {
            frontLeft .set(0, angleFL);
            frontRight.set(0, angleFR);
            backLeft  .set(0, angleBL);
            backRight .set(0, angleBR);

            boolean flReady = Math.abs(wrapAngle(frontLeft .getCurrentAngle() - angleFL)) < SNAP_TOLERANCE_RAD;
            boolean frReady = Math.abs(wrapAngle(frontRight.getCurrentAngle() - angleFR)) < SNAP_TOLERANCE_RAD;
            boolean blReady = Math.abs(wrapAngle(backLeft  .getCurrentAngle() - angleBL)) < SNAP_TOLERANCE_RAD;
            boolean brReady = Math.abs(wrapAngle(backRight .getCurrentAngle() - angleBR)) < SNAP_TOLERANCE_RAD;

            opMode.telemetry.addData("Phase", "Snapping to rotation");
            opMode.telemetry.update();

            if (flReady && frReady && blReady && brReady) break;
            if (System.currentTimeMillis() - startMs > SNAP_TIMEOUT_MS) break;
        }
    }

    /**
     * ROTATE phase — spin in place to targetHeading.
     * Wheels are already snapped to rotation positions from snapWheelsToRotation()
     * or will be driven tangentially by the kinematics (rot only, no translation).
     */
    private void rotateToHeading(double targetHeading) {
        headingPID.reset();

        while (opMode.opModeIsActive()) {
            updatePose();

            double headingError = wrapAngle(targetHeading - poseHeading);
            if (Math.abs(headingError) < HEADING_TOLERANCE_RAD) break;

            double rot = headingPID.calculate(headingError, System.nanoTime());
            rot = Math.max(-MAX_ROTATE_POWER, Math.min(MAX_ROTATE_POWER, rot));

            // Pure rotation — no translation component
            setSwerveOutputs(0, 0, rot, poseHeading);

            opMode.telemetry.addData("Phase",          "Rotating");
            opMode.telemetry.addData("Heading error",  "%.2f°", Math.toDegrees(headingError));
            opMode.telemetry.addData("Rotate power",   "%.3f",  rot);
            opMode.telemetry.update();
        }

        frontLeft.stop(); frontRight.stop();
        backLeft.stop();  backRight.stop();
        headingPID.reset();
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  SWERVE KINEMATICS
    // ═════════════════════════════════════════════════════════════════════════

    /**
     * Compute module target angles only (no power applied).
     * Returns [FL, FR, BL, BR] angles in radians.
     */
    private double[] computeModuleAngles(double fieldX, double fieldY,
                                         double rot, double heading) {
        double robotX = fieldX * Math.cos(-heading) - fieldY * Math.sin(-heading);
        double robotY = fieldX * Math.sin(-heading) + fieldY * Math.cos(-heading);

        double A = robotX - rot * (WHEELBASE / R);
        double B = robotX + rot * (WHEELBASE / R);
        double C = robotY - rot * (TRACK_WIDTH / R);
        double D = robotY + rot * (TRACK_WIDTH / R);

        return new double[]{
                Math.atan2(B, D),   // FL
                Math.atan2(B, C),   // FR
                Math.atan2(A, D),   // BL
                Math.atan2(A, C)    // BR
        };
    }

    /**
     * Convert field-relative drive inputs into per-module speed + angle commands.
     */
    private void setSwerveOutputs(double fieldX, double fieldY, double rot, double heading) {
        double robotX = fieldX * Math.cos(-heading) - fieldY * Math.sin(-heading);
        double robotY = fieldX * Math.sin(-heading) + fieldY * Math.cos(-heading);

        double A = robotX - rot * (WHEELBASE / R);
        double B = robotX + rot * (WHEELBASE / R);
        double C = robotY - rot * (TRACK_WIDTH / R);
        double D = robotY + rot * (TRACK_WIDTH / R);

        double speedFL = Math.hypot(B, D);
        double speedFR = Math.hypot(B, C);
        double speedBL = Math.hypot(A, D);
        double speedBR = Math.hypot(A, C);

        double maxSpd = Math.max(Math.max(speedFL, speedFR), Math.max(speedBL, speedBR));
        if (maxSpd > 1.0) {
            speedFL /= maxSpd; speedFR /= maxSpd;
            speedBL /= maxSpd; speedBR /= maxSpd;
        }

        frontLeft .set(speedFL, Math.atan2(B, D));
        frontRight.set(speedFR, Math.atan2(B, C));
        backLeft  .set(speedBL, Math.atan2(A, D));
        backRight .set(speedBR, Math.atan2(A, C));
    }

    // ─────────────────────────────────────────────────────────────────────────
    private double wrapAngle(double angle) {
        while (angle >  Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}