package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * fullyOmniAuto — adds diagonal driving to just2DAuto's capabilities.
 *
 * Five reusable methods:
 *   driveX(inches)           - forward/back (wheels at 0°, L/R heading correction)
 *   driveY(inches)           - strafe       (wheels at 90°, F/B heading correction)
 *   driveXY(dx, dy)          - RELATIVE diagonal: travel dx,dy from current position
 *   driveToXY(x, y)          - ABSOLUTE diagonal: travel to field position x,y
 *   correctHeading()         - active rotate back to auto-start heading
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
    final double DRIVE_KP = 0.10;
    final double DRIVE_KI = 0.0;
    final double DRIVE_KD = 0.05;
    final double DRIVE_KS = 0.05;
    final double DRIVE_TOLERANCE_IN = 0.5;
    final double DRIVE_MAX_POWER = 0.75;
    final double DRIVE_MIN_POWER = 0.20;
    final double DRIVE_P_MIN_POWER = 0.20;
    final long   DRIVE_TIMEOUT_MS = 6000;
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
    final double HEADING_CORRECT_TOLERANCE_DEG = 0.75;
    final double HEADING_CORRECT_MAX_POWER = 0.5;
    final double HEADING_CORRECT_MIN_POWER = 0.22;
    final double HEADING_CORRECT_P_MIN_POWER = 0.22;
    final long   HEADING_CORRECT_TIMEOUT_MS = 2500;
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

    // Captured once at the start of auto. correctHeading() targets this.
    private double autoStartHeadingDeg = 0;

    @Override
    public void runOpMode() {
        initializeHardware();

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

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(X_POD_OFFSET_MM, Y_POD_OFFSET_MM, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(X_POD_DIRECTION, Y_POD_DIRECTION);
        odo.resetPosAndIMU();
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