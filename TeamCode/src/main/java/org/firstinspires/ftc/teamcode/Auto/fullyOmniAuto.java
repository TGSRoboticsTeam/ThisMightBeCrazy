package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * fullyOmniAuto — adds diagonal driving to just2DAuto's capabilities.
 *
 * Five reusable DRIVE methods:
 *   driveX(inches)           - forward/back (wheels at 0°, L/R heading correction)
 *   driveY(inches)           - strafe       (wheels at 90°, F/B heading correction)
 *   driveXY(dx, dy)          - RELATIVE diagonal: travel dx,dy from current position
 *   driveToXY(x, y)          - ABSOLUTE diagonal: travel to field position x,y
 *   correctHeading()         - active rotate back to auto-start heading
 *
 * MECHANISM methods (added — mirror ApolloDrive TeleOp wiring):
 *   intake(seconds)          - run intake forward, blocking, with stall auto-shutoff
 *   intakeReverse(seconds)   - run intake reverse @ 50%, blocking (no stall check)
 *   stopIntake()             - hard off
 *   blockerLaunch()          - blocker to launch position (0.45)
 *   blockerBlocked()         - blocker to blocked position (0.15)
 *   launcher(seconds)        - spin flywheels up, hold, then graceful ramp-down (blocking)
 *   startLauncher()          - spin flywheels up and return (for overlap with driving)
 *   stopLauncher()           - graceful ramp-down to stop (blocking, ~0.5s)
 *
 * For diagonal moves:
 *   targetAngle  = atan2(dy, dx)        (all wheels lock here)
 *   totalDist    = hypot(dx, dy)        (drive distance along that direction)
 *   progress     = (Δx)cos(θ) + (Δy)sin(θ)   (projection of travel onto path)
 *   No in-drive heading correction — uses correctHeading() between moves instead.
 *
 * Sign conventions:
 *   driveX(+) = forward (Pinpoint X increases)
 *   driveY(+) = LEFT    (assumed) — flip Y_DIRECTION_SIGN if it strafes right instead
 *   driveXY(dx, dy) uses the same Pinpoint axes
 */
@Disabled
@Autonomous(name = "fullyOmniAuto", group = "Swerve")
public class fullyOmniAuto extends LinearOpMode {

    // ============================================================
    // HARDWARE
    // ============================================================
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private CRServo frontLeftSteer, frontRightSteer, backLeftSteer, backRightSteer;
    private AnalogInput frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder;
    private GoBildaPinpointDriver odo;
    private VoltageSensor voltageSensor;

    // --- MECHANISM HARDWARE (mirrors ApolloDrive TeleOp) ---
    private DcMotorEx topIntake, bottomIntake;   // DcMotorEx for current sensing
    private DcMotor leftFly, rightFly;           // launcher flywheels
    private Servo blocker;

    // ============================================================
    // ROBOT GEOMETRY (copied from justSwerve)
    // ============================================================
    final double TRACK_WIDTH = 17.258;
    final double WHEELBASE   = 13.544;
    final double R = Math.hypot(TRACK_WIDTH, WHEELBASE);

    // ============================================================
    // SWERVE MODULE ENCODER OFFSETS (copied from justSwerve)
    // ============================================================
    final double FRONT_LEFT_OFFSET  = 0.1200;
    final double FRONT_RIGHT_OFFSET = 1.3861;
    final double BACK_LEFT_OFFSET   = 1.6965;
    final double BACK_RIGHT_OFFSET  = 0.8225;

    // ============================================================
    // STEERING TUNING (copied from justSwerve)
    // ============================================================
    final double STEER_KP = 0.6;
    final double STEER_DEADBAND = 0.05;

    // ============================================================
    // PINPOINT CONFIGURATION
    // ============================================================
    final GoBildaPinpointDriver.EncoderDirection X_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    final GoBildaPinpointDriver.EncoderDirection Y_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;

    final double X_POD_OFFSET_MM =  41.9999922;   // X pod, left of center
    final double Y_POD_OFFSET_MM = -148.3535768;  // Y pod, behind center

    // Y_DIRECTION_SIGN: if positive Y commands drive the wrong physical direction,
    // flip this to -1.0. Currently assumed +Y = LEFT.
    final double Y_DIRECTION_SIGN = +1.0;

    // ============================================================
    // DISTANCE PID — shared across all drive methods.
    // ============================================================
    final double DRIVE_KP = 0.15;
    final double DRIVE_KI = 0.0;
    final double DRIVE_KD = 0.03;
    final double DRIVE_KS = 0.05;
    final double DRIVE_TOLERANCE_IN = 0.5;
    final double DRIVE_MAX_POWER = 0.75;
    final double DRIVE_MIN_POWER = 0.20;
    final double DRIVE_P_MIN_POWER = 0.20;
    final long   DRIVE_TIMEOUT_MS = 1500;
    final int    DRIVE_SETTLE_FRAMES = 5;
    final double DRIVE_INTEGRAL_RANGE_IN = 4.0;
    final double DRIVE_OVERSHOOT_BRAKE_POWER = 0.25;

    // ============================================================
    // IN-DRIVE HEADING CORRECTION (only used by driveX and driveY)
    // ============================================================
    final double HEADING_KP = 0.025;
    final double HEADING_MAX_CORRECTION = 0.3;

    // ============================================================
    // STANDALONE HEADING CORRECTION (correctHeading() method)
    // ============================================================
    final double HEADING_CORRECT_KP = 0.040;
    final double HEADING_CORRECT_KD = 0.002;
    final double HEADING_CORRECT_KS = 0.05;
    final double HEADING_CORRECT_TOLERANCE_DEG = 2;
    final double HEADING_CORRECT_MAX_POWER = 0.5;
    final double HEADING_CORRECT_MIN_POWER = 0.22;
    final double HEADING_CORRECT_P_MIN_POWER = 0.22;
    final long   HEADING_CORRECT_TIMEOUT_MS = 750;
    final int    HEADING_CORRECT_SETTLE_FRAMES = 5;

    // ============================================================
    // WHEEL ALIGNMENT
    // ============================================================
    final double STEER_ALIGN_TOLERANCE_RAD = Math.toRadians(1.75);
    final long   STEER_ALIGN_TIMEOUT_MS = 1500;

    // ============================================================
    // WHEEL TARGET ANGLES (for driveX and driveY)
    // ============================================================
    final double WHEELS_FORWARD_RAD = 0.0;
    final double WHEELS_SIDEWAYS_RAD = Math.PI / 2;

    // ============================================================
    // UNITS
    // ============================================================
    final double MM_PER_INCH = 25.4;

    // ============================================================
    // MECHANISM CONSTANTS (mirror ApolloDrive TeleOp values)
    // ============================================================
    // Blocker servo positions
    final double BLOCKER_BLOCKED_POSITION = 0.15;
    final double BLOCKER_LAUNCH_POSITION  = 0.45;

    // Intake
    final double INTAKE_FORWARD_POWER       = 0.90;  // matches INTAKE_SPEED_HIGH
    final double INTAKE_REVERSE_HOLD_POWER  = 0.50;  // matches TeleOp reverse-hold

    // Launcher (flywheels)
    final double LAUNCHER_POWER             = 0.90;  // auto launcher speed
    final double MOTOR_COAST_RAMP_SECONDS   = 0.5;   // graceful ramp-down duration

    // Intake stall detection — 5000 Series motor stall current @12V = 9.2A.
    // 7.0A sustained on either motor for >1s indicates a genuine jam.
    final double INTAKE_STALL_CURRENT_AMPS  = 7.0;
    final double INTAKE_STALL_TIME_SECONDS  = 1.0;

    // Captured once at the start of auto. correctHeading() targets this.
    private double autoStartHeadingDeg = 0;

    @Override
    public void runOpMode() {
        initializeHardware();

        // Start with the blocker closed (blocked) so nothing escapes pre-launch.
        blockerBlocked();

        telemetry.addLine("fullyOmniAuto initialized. Waiting for start.");
        telemetry.addData("Pinpoint status", odo.getDeviceStatus());
        telemetry.addData("Battery", "%.2f V", voltageSensor.getVoltage());
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        odo.update();
        autoStartHeadingDeg = odo.getHeading(AngleUnit.DEGREES);

        // ====================================================
        // HOURGLASS / BOWTIE SAMPLE SEQUENCE
        // Assumes +Y = LEFT. Path returns to origin if signs are right.
        //
        //   (24,24) <-- forward 24 --- (0,24)
        //       \                       ^
        //        \  diagonal           / diagonal
        //         \  back-right       /  back-left
        //          v                 /
        //   (0, 0) ---- forward 24 -> (24, 0)
        //
        // Order: forward, diagonal back-left, forward, diagonal back-right.
        // ====================================================
        driveX(24);                 // (0,0) → (24,0)
        sleep(300);
        correctHeading();
        sleep(300);

        driveXY(-24, 24);           // (24,0) → (0,24) — back-left diagonal
        sleep(300);
        correctHeading();
        sleep(300);

        driveX(24);                 // (0,24) → (24,24)
        sleep(300);
        correctHeading();
        sleep(300);

        driveXY(-24, -24);          // (24,24) → (0,0) — back-right diagonal
        sleep(300);
        correctHeading();
        // ====================================================

        // Example mechanism usage — uncomment / adapt as needed:
        // intake(2.0);             // collect for 2s (stops early if it jams)
        // blockerLaunch();         // open the blocker
        // launcher(3.0);           // spin up, hold 3s, ramp down
        // blockerBlocked();        // close the blocker again

        telemetry.addLine("Auto complete.");
        telemetry.update();
    }

    // ============================================================
    // HARDWARE INITIALIZATION
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

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetMotors(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);

        // --- MECHANISM HARDWARE (mirrors ApolloDrive TeleOp) ---
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

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(X_POD_OFFSET_MM, Y_POD_OFFSET_MM, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(X_POD_DIRECTION, Y_POD_DIRECTION);
        odo.resetPosAndIMU();
    }

    // ============================================================
    // ============================================================
    //   MECHANISM METHODS  (intake / blocker / launcher)
    // ============================================================
    // ============================================================

    // ------------------------------------------------------------
    // intake — run the intake FORWARD for a set duration (blocking).
    //
    // Hard on/off (no ramp). While running, motor current is polled
    // every loop; if max(topIntake, bottomIntake) current stays above
    // INTAKE_STALL_CURRENT_AMPS for more than INTAKE_STALL_TIME_SECONDS
    // continuously, the intake is shut off and the method returns early.
    //
    // NOTE: topIntake/bottomIntake spin FORWARD on NEGATIVE power
    // (same wiring as ApolloDrive TeleOp).
    // ------------------------------------------------------------
    public void intake(double seconds) {
        setIntakePower(INTAKE_FORWARD_POWER);   // forward

        ElapsedTime runTimer   = new ElapsedTime();
        ElapsedTime stallTimer = new ElapsedTime();
        boolean stallTiming = false;
        boolean stalled     = false;

        while (opModeIsActive() && runTimer.seconds() < seconds) {
            double topC = topIntake.getCurrent(CurrentUnit.AMPS);
            double botC = bottomIntake.getCurrent(CurrentUnit.AMPS);
            double maxC = Math.max(topC, botC);

            if (maxC > INTAKE_STALL_CURRENT_AMPS) {
                if (!stallTiming) {
                    stallTiming = true;
                    stallTimer.reset();
                } else if (stallTimer.seconds() >= INTAKE_STALL_TIME_SECONDS) {
                    stalled = true;
                    break;   // sustained stall — bail out
                }
            } else {
                stallTiming = false;   // dropped below threshold — reset timer
            }

            telemetry.addData("Phase", "INTAKE");
            telemetry.addData("Elapsed (s)", "%.2f / %.2f", runTimer.seconds(), seconds);
            telemetry.addData("Current", "top %.2fA  bot %.2fA", topC, botC);
            telemetry.addData("Stall watch", stallTiming
                    ? String.format("OVER %.2fs / %.1fs", stallTimer.seconds(), INTAKE_STALL_TIME_SECONDS)
                    : "ok");
            telemetry.update();
        }

        stopIntake();

        if (stalled) {
            telemetry.addData("Phase", "INTAKE");
            telemetry.addLine("STALL DETECTED — intake stopped early.");
            telemetry.update();
        }
    }

    // ------------------------------------------------------------
    // intakeReverse — run the intake in REVERSE at 50% for a set
    // duration (blocking). No stall detection (reverse is excluded,
    // same as ApolloDrive TeleOp). Useful for clearing a jam.
    // ------------------------------------------------------------
    public void intakeReverse(double seconds) {
        setIntakePower(-INTAKE_REVERSE_HOLD_POWER);   // reverse

        ElapsedTime runTimer = new ElapsedTime();
        while (opModeIsActive() && runTimer.seconds() < seconds) {
            telemetry.addData("Phase", "INTAKE REVERSE");
            telemetry.addData("Elapsed (s)", "%.2f / %.2f", runTimer.seconds(), seconds);
            telemetry.update();
        }

        stopIntake();
    }

    // ------------------------------------------------------------
    // stopIntake — hard off.
    // ------------------------------------------------------------
    public void stopIntake() {
        setIntakePower(0.0);
    }

    /**
     * Sets intake power in a "forward-positive" convention and applies
     * the negation needed by this robot's wiring. forwardPower > 0
     * collects; forwardPower < 0 ejects.
     */
    private void setIntakePower(double forwardPower) {
        topIntake.setPower(-forwardPower);
        bottomIntake.setPower(-forwardPower);
    }

    // ------------------------------------------------------------
    // blockerLaunch — move the blocker to the launch (open) position.
    // ------------------------------------------------------------
    public void blockerLaunch() {
        blocker.setPosition(BLOCKER_LAUNCH_POSITION);
    }

    // ------------------------------------------------------------
    // blockerBlocked — move the blocker to the blocked (closed) position.
    // ------------------------------------------------------------
    public void blockerBlocked() {
        blocker.setPosition(BLOCKER_BLOCKED_POSITION);
    }

    // ------------------------------------------------------------
    // launcher — spin the flywheels up, hold for the given duration,
    // then ramp down gracefully (blocking, total time is roughly
    // seconds + MOTOR_COAST_RAMP_SECONDS).
    // ------------------------------------------------------------
    public void launcher(double seconds) {
        startLauncher();

        ElapsedTime holdTimer = new ElapsedTime();
        while (opModeIsActive() && holdTimer.seconds() < seconds) {
            telemetry.addData("Phase", "LAUNCHER (hold)");
            telemetry.addData("Elapsed (s)", "%.2f / %.2f", holdTimer.seconds(), seconds);
            telemetry.addData("Power", "%.3f", launcherCompensatedPower());
            telemetry.update();
        }

        stopLauncher();
    }

    // ------------------------------------------------------------
    // startLauncher — spin the flywheels up to speed and return
    // immediately. Use this to overlap launcher spin-up with driving,
    // then call stopLauncher() when done.
    // ------------------------------------------------------------
    public void startLauncher() {
        double power = launcherCompensatedPower();
        leftFly.setPower(power);
        rightFly.setPower(power);
    }

    // ------------------------------------------------------------
    // stopLauncher — graceful ramp-down to a stop (blocking, ~0.5s).
    // Mirrors the TeleOp coast-to-stop behavior.
    // ------------------------------------------------------------
    public void stopLauncher() {
        double startPower = launcherCompensatedPower();
        ElapsedTime rampTimer = new ElapsedTime();

        while (opModeIsActive()) {
            double fraction = rampTimer.seconds() / MOTOR_COAST_RAMP_SECONDS;
            if (fraction >= 1.0) break;

            double power = startPower * (1.0 - fraction);
            leftFly.setPower(power);
            rightFly.setPower(power);

            telemetry.addData("Phase", "LAUNCHER (ramp down)");
            telemetry.addData("Power", "%.3f", power);
            telemetry.update();
        }

        leftFly.setPower(0.0);
        rightFly.setPower(0.0);
    }

    /** Launcher power scaled by the voltage factor (matches TeleOp). */
    private double launcherCompensatedPower() {
        double voltage = voltageSensor.getVoltage();
        double vf = (voltage > 0) ? Math.min(12.0 / voltage, 1.0) : 1.0;
        return LAUNCHER_POWER * vf;
    }

    // ============================================================
    // driveX — drive forward/backward.
    // ============================================================
    public void driveX(double inches) {
        odo.update();
        double startPos = readPosXInches();
        double targetPos = startPos + inches;

        alignWheelsTo(WHEELS_FORWARD_RAD, "ALIGNING FORWARD");

        runDistancePID(inches, targetPos, true, WHEELS_FORWARD_RAD, true);
    }

    // ============================================================
    // driveY — strafe.
    // ============================================================
    public void driveY(double inches) {
        odo.update();
        double startPos = readPosYInches();
        double targetPos = startPos + inches;

        alignWheelsTo(WHEELS_SIDEWAYS_RAD, "ALIGNING SIDEWAYS");

        runDistancePID(inches, targetPos, false, WHEELS_SIDEWAYS_RAD, false);
    }

    // ============================================================
    // driveXY — RELATIVE diagonal drive.
    // Travel a (dx, dy) displacement from current position.
    //
    // dx/dy are interpreted in the Pinpoint frame:
    //   dx > 0 = forward,  dx < 0 = back
    //   dy > 0 = left,     dy < 0 = right  (assuming +Y = LEFT)
    // ============================================================
    public void driveXY(double dxInches, double dyInches) {
        odo.update();
        double startX = readPosXInches();
        double startY = readPosYInches();
        double targetX = startX + dxInches;
        double targetY = startY + dyInches;

        runDiagonalPID(dxInches, dyInches, startX, startY, targetX, targetY);
    }

    // ============================================================
    // driveToXY — ABSOLUTE diagonal drive.
    // Travel to absolute field position (targetX, targetY) in inches.
    // Field origin is wherever Pinpoint was last reset (start of auto).
    // ============================================================
    public void driveToXY(double targetX, double targetY) {
        odo.update();
        double startX = readPosXInches();
        double startY = readPosYInches();
        double dx = targetX - startX;
        double dy = targetY - startY;

        runDiagonalPID(dx, dy, startX, startY, targetX, targetY);
    }

    // ============================================================
    // Shared diagonal PID. Used by driveXY and driveToXY.
    //   dx, dy:           displacement from start (inches)
    //   startX, startY:   starting position (inches)
    //   targetX, targetY: target position (inches)
    // ============================================================
    private void runDiagonalPID(double dx, double dy,
                                double startX, double startY,
                                double targetX, double targetY) {
        double totalDistance = Math.hypot(dx, dy);

        // No-op for zero-distance commands
        if (totalDistance < DRIVE_TOLERANCE_IN) {
            return;
        }

        // Travel angle in the Pinpoint frame: atan2(dy, dx)
        // This is also the wheel target angle (all four wheels point this direction).
        double travelAngle = Math.atan2(dy, dx);
        double cosA = Math.cos(travelAngle);
        double sinA = Math.sin(travelAngle);

        // Align all four wheels to the travel angle
        alignWheelsTo(travelAngle, String.format("ALIGNING %.1f deg", Math.toDegrees(travelAngle)));

        // PID state — we control progress (scalar distance along the path)
        double prevError = totalDistance;
        double integral = 0.0;
        long startTime = System.currentTimeMillis();
        long lastTime = startTime;
        int settleCounter = 0;

        boolean hasOvershot = false;

        while (opModeIsActive()) {
            odo.update();

            long now = System.currentTimeMillis();
            double dt = (now - lastTime) / 1000.0;
            if (dt <= 0) dt = 0.001;
            lastTime = now;

            // Progress = projection of current displacement onto the travel direction
            double currentX = readPosXInches();
            double currentY = readPosYInches();
            double traveled = (currentX - startX) * cosA + (currentY - startY) * sinA;
            double error = totalDistance - traveled;

            // Settle / exit
            if (Math.abs(error) < DRIVE_TOLERANCE_IN) {
                settleCounter++;
                if (settleCounter >= DRIVE_SETTLE_FRAMES) break;
            } else {
                settleCounter = 0;
            }
            if (now - startTime > DRIVE_TIMEOUT_MS) break;

            // PID with anti-windup
            if (Math.abs(error) < DRIVE_INTEGRAL_RANGE_IN) {
                integral += error * dt;
            } else {
                integral = 0;
            }
            double derivative = (error - prevError) / dt;

            // P term with floor
            double pTerm = DRIVE_KP * error;
            if (Math.abs(pTerm) < DRIVE_P_MIN_POWER && Math.abs(error) > DRIVE_TOLERANCE_IN) {
                pTerm = DRIVE_P_MIN_POWER * Math.signum(error);
            }

            double basePower = pTerm
                    + DRIVE_KI * integral
                    + DRIVE_KD * derivative
                    + DRIVE_KS * Math.signum(error);

            basePower = clamp(basePower, -DRIVE_MAX_POWER, DRIVE_MAX_POWER);

            if (Math.abs(basePower) < DRIVE_MIN_POWER && Math.abs(error) > DRIVE_TOLERANCE_IN) {
                basePower = DRIVE_MIN_POWER * Math.signum(error);
            }

            // Overshoot brake: progress went past total distance (or behind for retreats,
            // but for diagonal moves we always go forward along the travel angle)
            if (error < -DRIVE_TOLERANCE_IN) {
                hasOvershot = true;
            }
            if (hasOvershot && Math.abs(error) > DRIVE_TOLERANCE_IN) {
                basePower = DRIVE_OVERSHOOT_BRAKE_POWER * Math.signum(error);
            }

            // No in-drive heading correction during diagonal moves —
            // correctHeading() between moves handles drift.
            // All four wheels get the same power and angle.
            setDrivePowersAll(basePower, travelAngle);

            // Heading is monitored for telemetry only (not corrected here)
            double currentHeading = readHeadingDegrees();
            double headingError = wrapAngleDegrees(autoStartHeadingDeg - currentHeading);

            telemetry.addData("Phase", "DIAGONAL");
            telemetry.addData("Travel angle (deg)", "%.2f", Math.toDegrees(travelAngle));
            telemetry.addData("Total dist (in)",    "%.2f", totalDistance);
            telemetry.addData("Traveled (in)",      "%.2f", traveled);
            telemetry.addData("Error (in)",         "%.2f", error);
            telemetry.addData("Current (x,y)",      "(%.2f, %.2f)", currentX, currentY);
            telemetry.addData("Target  (x,y)",      "(%.2f, %.2f)", targetX, targetY);
            telemetry.addData("Base Power",         "%.3f", basePower);
            telemetry.addData("Overshot?",          hasOvershot);
            telemetry.addData("Heading drift (deg)","%.2f", headingError);
            telemetry.addData("Elapsed (ms)",       now - startTime);
            telemetry.update();

            prevError = error;
        }

        // Stop
        long stopUntil = System.currentTimeMillis() + 100;
        while (opModeIsActive() && System.currentTimeMillis() < stopUntil) {
            odo.update();
            setDrivePowersAll(0, travelAngle);
        }
        setDrivePowersAll(0, travelAngle);
    }

    // ============================================================
    // Shared linear distance PID. Used by driveX and driveY.
    // ============================================================
    private void runDistancePID(double inches, double targetPos, boolean readX,
                                double wheelAngle, boolean useLeftRight) {
        double prevError = inches;
        double integral = 0.0;
        long startTime = System.currentTimeMillis();
        long lastTime = startTime;
        int settleCounter = 0;

        double initialErrorSign = Math.signum(inches);
        boolean hasOvershot = false;

        while (opModeIsActive()) {
            odo.update();

            long now = System.currentTimeMillis();
            double dt = (now - lastTime) / 1000.0;
            if (dt <= 0) dt = 0.001;
            lastTime = now;

            double currentPos = readX ? readPosXInches() : readPosYInches();
            double error = targetPos - currentPos;

            if (Math.abs(error) < DRIVE_TOLERANCE_IN) {
                settleCounter++;
                if (settleCounter >= DRIVE_SETTLE_FRAMES) break;
            } else {
                settleCounter = 0;
            }
            if (now - startTime > DRIVE_TIMEOUT_MS) break;

            if (Math.abs(error) < DRIVE_INTEGRAL_RANGE_IN) {
                integral += error * dt;
            } else {
                integral = 0;
            }
            double derivative = (error - prevError) / dt;

            double pTerm = DRIVE_KP * error;
            if (Math.abs(pTerm) < DRIVE_P_MIN_POWER && Math.abs(error) > DRIVE_TOLERANCE_IN) {
                pTerm = DRIVE_P_MIN_POWER * Math.signum(error);
            }

            double basePower = pTerm
                    + DRIVE_KI * integral
                    + DRIVE_KD * derivative
                    + DRIVE_KS * Math.signum(error);

            basePower = clamp(basePower, -DRIVE_MAX_POWER, DRIVE_MAX_POWER);

            if (Math.abs(basePower) < DRIVE_MIN_POWER && Math.abs(error) > DRIVE_TOLERANCE_IN) {
                basePower = DRIVE_MIN_POWER * Math.signum(error);
            }

            if (initialErrorSign != 0 && Math.signum(error) != initialErrorSign
                    && Math.abs(error) > DRIVE_TOLERANCE_IN) {
                hasOvershot = true;
            }
            if (hasOvershot && Math.abs(error) > DRIVE_TOLERANCE_IN) {
                basePower = DRIVE_OVERSHOOT_BRAKE_POWER * Math.signum(error);
            }

            double currentHeading = readHeadingDegrees();
            double headingError = wrapAngleDegrees(autoStartHeadingDeg - currentHeading);
            double correction = clamp(HEADING_KP * headingError,
                    -HEADING_MAX_CORRECTION,
                    HEADING_MAX_CORRECTION);

            double powerA = clamp(basePower - correction, -1.0, 1.0);
            double powerB = clamp(basePower + correction, -1.0, 1.0);

            if (useLeftRight) {
                setDrivePowersLeftRight(powerA, powerB, wheelAngle);
            } else {
                setDrivePowersFrontBack(powerA, powerB, wheelAngle);
            }

            telemetry.addData("Phase", readX ? "DRIVING X" : "DRIVING Y");
            telemetry.addData("Target (in)",  "%.2f", targetPos);
            telemetry.addData("Current (in)", "%.2f", currentPos);
            telemetry.addData("Error (in)",   "%.2f", error);
            telemetry.addData("Base Power",   "%.3f", basePower);
            telemetry.addData("Overshot?",    hasOvershot);
            telemetry.addData("Heading err (deg)", "%.2f", headingError);
            telemetry.addData("Correction",   "%.3f", correction);
            telemetry.addData(useLeftRight ? "Left / Right" : "Front / Back",
                    "%.3f / %.3f", powerA, powerB);
            telemetry.addData("Elapsed (ms)", now - startTime);
            telemetry.update();

            prevError = error;
        }

        long stopUntil = System.currentTimeMillis() + 100;
        while (opModeIsActive() && System.currentTimeMillis() < stopUntil) {
            odo.update();
            if (useLeftRight) {
                setDrivePowersLeftRight(0, 0, wheelAngle);
            } else {
                setDrivePowersFrontBack(0, 0, wheelAngle);
            }
        }
        if (useLeftRight) {
            setDrivePowersLeftRight(0, 0, wheelAngle);
        } else {
            setDrivePowersFrontBack(0, 0, wheelAngle);
        }
    }

    // ============================================================
    // correctHeading — actively rotate the robot back to autoStartHeadingDeg.
    // ============================================================
    public void correctHeading() {
        long startTime = System.currentTimeMillis();
        long lastTime = startTime;
        double prevError;
        int settleCounter = 0;

        odo.update();
        prevError = wrapAngleDegrees(autoStartHeadingDeg - readHeadingDegrees());

        while (opModeIsActive()) {
            odo.update();

            long now = System.currentTimeMillis();
            double dt = (now - lastTime) / 1000.0;
            if (dt <= 0) dt = 0.001;
            lastTime = now;

            double currentHeading = readHeadingDegrees();
            double error = wrapAngleDegrees(autoStartHeadingDeg - currentHeading);

            if (Math.abs(error) < HEADING_CORRECT_TOLERANCE_DEG) {
                settleCounter++;
                if (settleCounter >= HEADING_CORRECT_SETTLE_FRAMES) break;
            } else {
                settleCounter = 0;
            }
            if (now - startTime > HEADING_CORRECT_TIMEOUT_MS) break;

            double derivative = (error - prevError) / dt;

            double pTerm = HEADING_CORRECT_KP * error;
            if (Math.abs(pTerm) < HEADING_CORRECT_P_MIN_POWER
                    && Math.abs(error) > HEADING_CORRECT_TOLERANCE_DEG) {
                pTerm = HEADING_CORRECT_P_MIN_POWER * Math.signum(error);
            }

            double rotPower = pTerm
                    + HEADING_CORRECT_KD * derivative
                    + HEADING_CORRECT_KS * Math.signum(error);

            rotPower = clamp(rotPower, -HEADING_CORRECT_MAX_POWER, HEADING_CORRECT_MAX_POWER);
            if (Math.abs(rotPower) < HEADING_CORRECT_MIN_POWER
                    && Math.abs(error) > HEADING_CORRECT_TOLERANCE_DEG) {
                rotPower = HEADING_CORRECT_MIN_POWER * Math.signum(error);
            }

            // SIGN: justSwerve's rot convention is OPPOSITE of Pinpoint heading convention.
            // If robot still turns wrong way on the bench, remove the negation here.
            double rot = -rotPower;
            double A = -rot * (WHEELBASE / R);
            double B =  rot * (WHEELBASE / R);
            double C = -rot * (TRACK_WIDTH / R);
            double D =  rot * (TRACK_WIDTH / R);

            double speedFL = Math.hypot(B, D);
            double speedFR = Math.hypot(B, C);
            double speedBL = Math.hypot(A, D);
            double speedBR = Math.hypot(A, C);
            double maxSpeed = Math.max(Math.max(speedFL, speedFR), Math.max(speedBL, speedBR));
            if (maxSpeed > 1.0) {
                speedFL /= maxSpeed; speedFR /= maxSpeed;
                speedBL /= maxSpeed; speedBR /= maxSpeed;
            }

            double angleFL = Math.atan2(B, D);
            double angleFR = Math.atan2(B, C);
            double angleBL = Math.atan2(A, D);
            double angleBR = Math.atan2(A, C);

            runModule(frontLeftDrive,  frontLeftSteer,  frontLeftEncoder,  FRONT_LEFT_OFFSET,  speedFL, angleFL);
            runModule(frontRightDrive, frontRightSteer, frontRightEncoder, FRONT_RIGHT_OFFSET, speedFR, angleFR);
            runModule(backLeftDrive,   backLeftSteer,   backLeftEncoder,   BACK_LEFT_OFFSET,   speedBL, angleBL);
            runModule(backRightDrive,  backRightSteer,  backRightEncoder,  BACK_RIGHT_OFFSET,  speedBR, angleBR);

            telemetry.addData("Phase", "CORRECTING HEADING");
            telemetry.addData("Target heading (deg)",  "%.3f", autoStartHeadingDeg);
            telemetry.addData("Current heading (deg)", "%.3f", currentHeading);
            telemetry.addData("Error (deg)",           "%.3f", error);
            telemetry.addData("Rot power",             "%.3f", rotPower);
            telemetry.addData("Elapsed (ms)",          now - startTime);
            telemetry.update();

            prevError = error;
        }

        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        frontLeftSteer.setPower(0);
        frontRightSteer.setPower(0);
        backLeftSteer.setPower(0);
        backRightSteer.setPower(0);
    }

    // ============================================================
    // Align all four modules to a target angle.
    // ============================================================
    private void alignWheelsTo(double targetAngle, String phaseLabel) {
        long startTime = System.currentTimeMillis();

        while (opModeIsActive()) {
            odo.update();

            double errFL = wrapAngle(targetAngle - (getRawAngle(frontLeftEncoder)  - FRONT_LEFT_OFFSET));
            double errFR = wrapAngle(targetAngle - (getRawAngle(frontRightEncoder) - FRONT_RIGHT_OFFSET));
            double errBL = wrapAngle(targetAngle - (getRawAngle(backLeftEncoder)   - BACK_LEFT_OFFSET));
            double errBR = wrapAngle(targetAngle - (getRawAngle(backRightEncoder)  - BACK_RIGHT_OFFSET));

            runModule(frontLeftDrive,  frontLeftSteer,  frontLeftEncoder,  FRONT_LEFT_OFFSET,  0, targetAngle);
            runModule(frontRightDrive, frontRightSteer, frontRightEncoder, FRONT_RIGHT_OFFSET, 0, targetAngle);
            runModule(backLeftDrive,   backLeftSteer,   backLeftEncoder,   BACK_LEFT_OFFSET,   0, targetAngle);
            runModule(backRightDrive,  backRightSteer,  backRightEncoder,  BACK_RIGHT_OFFSET,  0, targetAngle);

            boolean aligned = isAlignedTo(errFL) && isAlignedTo(errFR)
                    && isAlignedTo(errBL) && isAlignedTo(errBR);

            long elapsed = System.currentTimeMillis() - startTime;

            telemetry.addData("Phase", phaseLabel);
            telemetry.addData("FL err (deg)", "%.2f", Math.toDegrees(errFL));
            telemetry.addData("FR err (deg)", "%.2f", Math.toDegrees(errFR));
            telemetry.addData("BL err (deg)", "%.2f", Math.toDegrees(errBL));
            telemetry.addData("BR err (deg)", "%.2f", Math.toDegrees(errBR));
            telemetry.addData("Aligned?", aligned);
            telemetry.addData("Elapsed (ms)", elapsed);
            telemetry.update();

            if (aligned) break;
            if (elapsed > STEER_ALIGN_TIMEOUT_MS) break;
        }

        frontLeftSteer.setPower(0);
        frontRightSteer.setPower(0);
        backLeftSteer.setPower(0);
        backRightSteer.setPower(0);
    }

    private boolean isAlignedTo(double angleError) {
        double e1 = Math.abs(angleError);
        double e2 = Math.abs(wrapAngle(angleError + Math.PI));
        return Math.min(e1, e2) < STEER_ALIGN_TOLERANCE_RAD;
    }

    // ============================================================
    // Drive power application
    // ============================================================
    private void setDrivePowersLeftRight(double leftPower, double rightPower, double wheelAngle) {
        runModule(frontLeftDrive,  frontLeftSteer,  frontLeftEncoder,  FRONT_LEFT_OFFSET,  leftPower,  wheelAngle);
        runModule(backLeftDrive,   backLeftSteer,   backLeftEncoder,   BACK_LEFT_OFFSET,   leftPower,  wheelAngle);
        runModule(frontRightDrive, frontRightSteer, frontRightEncoder, FRONT_RIGHT_OFFSET, rightPower, wheelAngle);
        runModule(backRightDrive,  backRightSteer,  backRightEncoder,  BACK_RIGHT_OFFSET,  rightPower, wheelAngle);
    }

    private void setDrivePowersFrontBack(double frontPower, double backPower, double wheelAngle) {
        runModule(frontLeftDrive,  frontLeftSteer,  frontLeftEncoder,  FRONT_LEFT_OFFSET,  frontPower, wheelAngle);
        runModule(frontRightDrive, frontRightSteer, frontRightEncoder, FRONT_RIGHT_OFFSET, frontPower, wheelAngle);
        runModule(backLeftDrive,   backLeftSteer,   backLeftEncoder,   BACK_LEFT_OFFSET,   backPower,  wheelAngle);
        runModule(backRightDrive,  backRightSteer,  backRightEncoder,  BACK_RIGHT_OFFSET,  backPower,  wheelAngle);
    }

    /** All four wheels get the same power and angle (used for diagonal drives). */
    private void setDrivePowersAll(double power, double wheelAngle) {
        runModule(frontLeftDrive,  frontLeftSteer,  frontLeftEncoder,  FRONT_LEFT_OFFSET,  power, wheelAngle);
        runModule(frontRightDrive, frontRightSteer, frontRightEncoder, FRONT_RIGHT_OFFSET, power, wheelAngle);
        runModule(backLeftDrive,   backLeftSteer,   backLeftEncoder,   BACK_LEFT_OFFSET,   power, wheelAngle);
        runModule(backRightDrive,  backRightSteer,  backRightEncoder,  BACK_RIGHT_OFFSET,  power, wheelAngle);
    }

    // ============================================================
    // Swerve module control (copied verbatim from justSwerve)
    // ============================================================
    private void runModule(DcMotor driveMotor, CRServo steerServo, AnalogInput encoder,
                           double encoderOffset, double speed, double targetAngle) {
        double rawAngle = getRawAngle(encoder);
        double currentAngle = rawAngle - encoderOffset;
        currentAngle = wrapAngle(currentAngle);

        double delta = wrapAngle(targetAngle - currentAngle);

        if (Math.abs(delta) > Math.PI / 2) {
            delta = wrapAngle(delta + Math.PI);
            speed *= -1;
        }

        double servoPower = STEER_KP * delta;
        servoPower *= -1;

        if (Math.abs(servoPower) < STEER_DEADBAND) servoPower = 0;
        servoPower = Math.max(-1, Math.min(1, servoPower));

        steerServo.setPower(servoPower);
        driveMotor.setPower(speed);
    }

    // ============================================================
    // Pinpoint readers
    // ============================================================
    private double readPosXInches() {
        return odo.getPosX(DistanceUnit.INCH);
    }

    private double readPosYInches() {
        return odo.getPosY(DistanceUnit.INCH) * Y_DIRECTION_SIGN;
    }

    private double readHeadingDegrees() {
        return odo.getHeading(AngleUnit.DEGREES);
    }

    // ============================================================
    // Angle / utility helpers
    // ============================================================
    private double wrapAngle(double angle) {
        while (angle > Math.PI)  angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private double wrapAngleDegrees(double angle) {
        while (angle > 180)  angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    private double getRawAngle(AnalogInput encoder) {
        return encoder.getVoltage() / 3.3 * (2 * Math.PI);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private void resetMotors(DcMotor... motors) {
        for (DcMotor m : motors) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
}