package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.Waypoint.ActionTiming;

/**
 * SwerveDriveAuto
 * ───────────────────────────────────────────────────────────────────────────
 * Autonomous drive controller for the coaxial swerve drivetrain.
 * Uses the goBUILDA Pinpoint computer for localisation.
 *
 * ── Quick-start ──────────────────────────────────────────────────────────────
 *
 *   SwerveDriveAuto drive = new SwerveDriveAuto(this);
 *   drive.init();
 *
 *   // Single moves
 *   drive.driveTo(24, 0, 0);                 // absolute field position (inches, radians)
 *   drive.driveRelative(12, 6, 0);           // relative to current pose
 *
 *   // Mechanism action inline
 *   drive.runAction(() -> lift.setTarget(800));
 *
 *   // Full path with actions
 *   drive.followPath(new Waypoint[]{
 *       new Waypoint(24, 0,  0),
 *       new Waypoint(24, 24, Math.PI/2, () -> claw.close()),
 *       new Waypoint(0,  24, Math.PI,   ActionTiming.ON_APPROACH, 6.0, () -> lift.setTarget(0)),
 *   });
 *
 * ── Tuning constants ─────────────────────────────────────────────────────────
 *   All PID and profile constants are at the top of this file.
 *   Start with defaults, then adjust for your robot weight / floor surface.
 */
public class SwerveDriveAuto {

    // ═════════════════════════════════════════════════════════════════════════
    //  TUNING — adjust these for your robot
    // ═════════════════════════════════════════════════════════════════════════

    // Translation PID (tracks velocity profile error)
    private static final double TRANS_KP = 0.045;
    private static final double TRANS_KI = 0.0;
    private static final double TRANS_KD = 0.003;

    // Heading PID (keeps robot pointed at target heading during a move)
    private static final double HEAD_KP = 0.5;
    private static final double HEAD_KI = 0.0;
    private static final double HEAD_KD = 0.04;

    // Peak drive power during cruise phase (0…1)
    private static final double MAX_DRIVE_POWER = 0.70;

    // Distance tolerance — considered "arrived" when within this many inches
    private static final double POSITION_TOLERANCE_IN = 0.75;

    // Heading tolerance — considered "aligned" within this many radians
    private static final double HEADING_TOLERANCE_RAD = 0.03;   // ~1.7°

    // Maximum heading-correction power added to drive output
    private static final double MAX_HEADING_CORRECTION = 0.35;

    // ═════════════════════════════════════════════════════════════════════════
    //  ROBOT GEOMETRY  (match justSwerve)
    // ═════════════════════════════════════════════════════════════════════════
    private static final double TRACK_WIDTH = 17.258;
    private static final double WHEELBASE   = 13.544;
    private static final double R           = Math.hypot(TRACK_WIDTH, WHEELBASE);

    private static final double FRONT_LEFT_OFFSET  = 1.34;
    private static final double FRONT_RIGHT_OFFSET = 3.161;
    private static final double BACK_LEFT_OFFSET   = 1.589;
    private static final double BACK_RIGHT_OFFSET  = 1.237;

    // ═════════════════════════════════════════════════════════════════════════
    //  HARDWARE
    // ═════════════════════════════════════════════════════════════════════════
    private final LinearOpMode opMode;

    private SwerveModule frontLeft, frontRight, backLeft, backRight;

    // goBUILDA Pinpoint — accessed via the FTC SDK device interface
    // Replace the type with whatever class your Pinpoint SDK exposes if different.
    // We use a thin wrapper (PinpointLocalizer) defined below in this file.
    private PinpointLocalizer localizer;

    // ═════════════════════════════════════════════════════════════════════════
    //  PID CONTROLLERS
    // ═════════════════════════════════════════════════════════════════════════
    private final PIDController translationPID = new PIDController(TRANS_KP, TRANS_KI, TRANS_KD, 0.4);
    private final PIDController headingPID     = new PIDController(HEAD_KP,  HEAD_KI,  HEAD_KD,  0.3);

    // ═════════════════════════════════════════════════════════════════════════
    //  POSE STATE
    // ═════════════════════════════════════════════════════════════════════════
    private double poseX       = 0;   // inches
    private double poseY       = 0;   // inches
    private double poseHeading = 0;   // radians

    // ─────────────────────────────────────────────────────────────────────────
    public SwerveDriveAuto(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  INITIALISATION
    // ═════════════════════════════════════════════════════════════════════════

    /**
     * Initialise hardware and localiser. Call once before waitForStart().
     * Optionally provide a starting pose if the robot does not start at (0,0,0).
     */
    public void init(double startX, double startY, double startHeading) {
        var hw = opMode.hardwareMap;

        // ── Drive modules ───────────────────────────────────────────────────
        frontLeft  = new SwerveModule(
                hw.get(DcMotor.class,    "frontLeftDrive"),
                hw.get(CRServo.class,    "frontLeftSteer"),
                hw.get(AnalogInput.class,"frontLeftEncoder"),
                FRONT_LEFT_OFFSET);

        frontRight = new SwerveModule(
                hw.get(DcMotor.class,    "frontRightDrive"),
                hw.get(CRServo.class,    "frontRightSteer"),
                hw.get(AnalogInput.class,"frontRightEncoder"),
                FRONT_RIGHT_OFFSET);

        backLeft   = new SwerveModule(
                hw.get(DcMotor.class,    "backLeftDrive"),
                hw.get(CRServo.class,    "backLeftSteer"),
                hw.get(AnalogInput.class,"backLeftEncoder"),
                BACK_LEFT_OFFSET);

        backRight  = new SwerveModule(
                hw.get(DcMotor.class,    "backRightDrive"),
                hw.get(CRServo.class,    "backRightSteer"),
                hw.get(AnalogInput.class,"backRightEncoder"),
                BACK_RIGHT_OFFSET);

        // ── Motor directions (match justSwerve) ─────────────────────────────
        hw.get(DcMotor.class, "frontLeftDrive") .setDirection(DcMotor.Direction.REVERSE);
        hw.get(DcMotor.class, "backLeftDrive")  .setDirection(DcMotor.Direction.REVERSE);
        hw.get(DcMotor.class, "frontRightDrive").setDirection(DcMotor.Direction.REVERSE);
        hw.get(DcMotor.class, "backRightDrive") .setDirection(DcMotor.Direction.REVERSE);

        hw.get(DcMotor.class, "frontLeftDrive") .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.get(DcMotor.class, "frontRightDrive").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.get(DcMotor.class, "backLeftDrive")  .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.get(DcMotor.class, "backRightDrive") .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        for (DcMotor m : new DcMotor[]{
                hw.get(DcMotor.class,"frontLeftDrive"),  hw.get(DcMotor.class,"frontRightDrive"),
                hw.get(DcMotor.class,"backLeftDrive"),   hw.get(DcMotor.class,"backRightDrive")}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // ── Pinpoint localiser ───────────────────────────────────────────────
        localizer = new PinpointLocalizer(opMode.hardwareMap);
        localizer.setPosition(startX, startY, startHeading);

        poseX       = startX;
        poseY       = startY;
        poseHeading = startHeading;
    }

    /** Convenience overload — robot starts at field origin, facing 0 rad. */
    public void init() { init(0, 0, 0); }

    // ═════════════════════════════════════════════════════════════════════════
    //  PUBLIC API
    // ═════════════════════════════════════════════════════════════════════════

    /**
     * Drive to an absolute field position and heading. Blocks until arrived.
     *
     * @param x       target X in inches (field-relative)
     * @param y       target Y in inches (field-relative)
     * @param heading target robot heading in radians
     */
    public void driveTo(double x, double y, double heading) {
        executeMoveToTarget(x, y, heading, null, ActionTiming.ON_ARRIVE, 0);
    }

    /**
     * Drive a distance relative to the current pose. Blocks until arrived.
     *
     * @param dx      inches in the current field-X direction
     * @param dy      inches in the current field-Y direction
     * @param dh      heading change in radians
     */
    public void driveRelative(double dx, double dy, double dh) {
        updatePose();
        driveTo(poseX + dx, poseY + dy, poseHeading + dh);
    }

    /**
     * Follow an ordered list of waypoints.
     * Actions attached to each waypoint fire automatically.
     */
    public void followPath(Waypoint[] waypoints) {
        for (Waypoint wp : waypoints) {
            if (!opMode.opModeIsActive()) break;
            executeMoveToTarget(wp.x, wp.y, wp.heading,
                                wp.action, wp.timing, wp.approachDistance);
        }
    }

    /**
     * Run a mechanism action right now and wait for it to complete.
     * Use between driveTo() calls to trigger mechanisms without a position change.
     */
    public void runAction(Runnable action) {
        if (action != null) action.run();
    }

    /**
     * Rotate in place to a new heading without translating.
     */
    public void rotateTo(double heading) {
        driveTo(poseX, poseY, heading);
    }

    /**
     * Stop all drive motors immediately and X-lock the wheels.
     */
    public void stopAndLock() {
        frontLeft .lockAt(-Math.PI / 4);
        frontRight.lockAt( Math.PI / 4);
        backLeft  .lockAt( Math.PI / 4);
        backRight .lockAt(-Math.PI / 4);
    }

    /** Read the latest pose from the Pinpoint. */
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

    /**
     * Internal blocking move from current pose to (targetX, targetY, targetHeading).
     * Handles trapezoidal profiling, heading correction, and action callbacks.
     */
    private void executeMoveToTarget(double targetX, double targetY, double targetHeading,
                                     Runnable action, ActionTiming timing, double approachDist) {
        translationPID.reset();
        headingPID.reset();
        updatePose();

        // Total distance for this move (used by the trapezoid profile)
        double startX   = poseX;
        double startY   = poseY;
        double totalDist = Math.hypot(targetX - startX, targetY - startY);

        TrapezoidProfile profile = new TrapezoidProfile(totalDist, MAX_DRIVE_POWER);

        boolean approachActionFired = false;
        boolean isPureRotation      = totalDist < 0.5;  // heading-only move

        while (opMode.opModeIsActive()) {
            updatePose();

            double remainingDist = Math.hypot(targetX - poseX, targetY - poseY);
            double travelledDist = totalDist - remainingDist;

            // ── Check ON_APPROACH action ─────────────────────────────────────
            if (!approachActionFired
                    && action != null
                    && timing == ActionTiming.ON_APPROACH
                    && remainingDist <= approachDist) {
                action.run();
                approachActionFired = true;
            }

            // ── Termination check ────────────────────────────────────────────
            double headingError = wrapAngle(targetHeading - poseHeading);
            boolean atPosition  = remainingDist   < POSITION_TOLERANCE_IN;
            boolean atHeading   = Math.abs(headingError) < HEADING_TOLERANCE_RAD;

            if (atPosition && atHeading) break;

            // ── Translation ──────────────────────────────────────────────────
            double profileSpeed = isPureRotation ? 0
                    : profile.getVelocity(travelledDist);

            // Angle of travel in field frame
            double travelAngle = Math.atan2(targetY - poseY, targetX - poseX);

            // Decompose into field X/Y drive components
            double driveX = profileSpeed * Math.cos(travelAngle);
            double driveY = profileSpeed * Math.sin(travelAngle);

            // Slow down as we approach (PID on remaining distance)
            if (atPosition) { driveX = 0; driveY = 0; }

            // ── Heading correction ───────────────────────────────────────────
            double headingCorrection = headingPID.calculate(headingError, System.nanoTime());
            headingCorrection = Math.max(-MAX_HEADING_CORRECTION,
                                Math.min( MAX_HEADING_CORRECTION, headingCorrection));

            // ── Swerve kinematics (field-relative → module vectors) ──────────
            setSwerveOutputs(driveX, driveY, headingCorrection, poseHeading);

            // ── Telemetry ────────────────────────────────────────────────────
            opMode.telemetry.addData("Target",    "(%.1f, %.1f, %.2f°)", targetX, targetY, Math.toDegrees(targetHeading));
            opMode.telemetry.addData("Pose",      "(%.1f, %.1f, %.2f°)", poseX,   poseY,   Math.toDegrees(poseHeading));
            opMode.telemetry.addData("Remaining", "%.2f in", remainingDist);
            opMode.telemetry.addData("Speed",     "%.2f", profileSpeed);
            opMode.telemetry.update();
        }

        // Arrived — stop drive
        frontLeft.stop();  frontRight.stop();
        backLeft.stop();   backRight.stop();

        // ── ON_ARRIVE action ─────────────────────────────────────────────────
        if (action != null && timing == ActionTiming.ON_ARRIVE) {
            action.run();
        }
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  SWERVE KINEMATICS
    // ═════════════════════════════════════════════════════════════════════════

    /**
     * Convert field-relative drive inputs into per-module speed + angle commands.
     * Uses the same kinematics as justSwerve, with a field→robot frame rotation.
     *
     * @param fieldX   field-X drive component (-1…1)
     * @param fieldY   field-Y drive component (-1…1)
     * @param rot      rotation demand (-1…1, positive = CCW)
     * @param heading  current robot heading (radians) for field-relative transform
     */
    private void setSwerveOutputs(double fieldX, double fieldY, double rot, double heading) {
        // Rotate field-relative inputs into robot-relative frame
        double robotX =  fieldX * Math.cos(-heading) - fieldY * Math.sin(-heading);
        double robotY =  fieldX * Math.sin(-heading) + fieldY * Math.cos(-heading);

        // Standard swerve kinematics (identical to justSwerve)
        double A = robotX - rot * (WHEELBASE / R);
        double B = robotX + rot * (WHEELBASE / R);
        double C = robotY - rot * (TRACK_WIDTH / R);
        double D = robotY + rot * (TRACK_WIDTH / R);

        double speedFL = Math.hypot(B, D);
        double speedFR = Math.hypot(B, C);
        double speedBL = Math.hypot(A, D);
        double speedBR = Math.hypot(A, C);

        // Normalise if any speed exceeds 1
        double maxSpd = Math.max(Math.max(speedFL, speedFR), Math.max(speedBL, speedBR));
        if (maxSpd > 1.0) { speedFL /= maxSpd; speedFR /= maxSpd; speedBL /= maxSpd; speedBR /= maxSpd; }

        double angleFL = Math.atan2(B, D);
        double angleFR = Math.atan2(B, C);
        double angleBL = Math.atan2(A, D);
        double angleBR = Math.atan2(A, C);

        frontLeft .set(speedFL, angleFL);
        frontRight.set(speedFR, angleFR);
        backLeft  .set(speedBL, angleBL);
        backRight .set(speedBR, angleBR);
    }

    // ─────────────────────────────────────────────────────────────────────────
    private double wrapAngle(double angle) {
        while (angle >  Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
