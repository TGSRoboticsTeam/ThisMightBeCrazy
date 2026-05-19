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
 * justAutoStrafe — drives sideways using the Y pod for position feedback.
 *
 * Wheels lock at +90 degrees so the robot strafes. Position feedback comes from
 * the Y pod (which is verified working). Heading correction uses front-pair vs
 * back-pair drive power asymmetry instead of left/right.
 *
 * Positive driveDistance() = strafe in the direction that makes Y increase.
 * If that's the wrong physical direction, flip Y_DIRECTION_SIGN below.
 */
@Autonomous(name = "justAutoStrafe", group = "Swerve")
public class JustAutoStrafe extends LinearOpMode {

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

    // ============================================================
    // DISTANCE PID (using Y pod, in inches)
    // Same gains as forward auto — adjust if strafe behaves differently.
    // ============================================================
    final double DRIVE_KP = 0.04;
    final double DRIVE_KI = 0.0;
    final double DRIVE_KD = 0.05;
    final double DRIVE_KS = 0.03;
    final double DRIVE_TOLERANCE_IN = 0.5;
    final double DRIVE_MAX_POWER = 0.45;
    final double DRIVE_MIN_POWER = 0.08;
    final long   DRIVE_TIMEOUT_MS = 4000;
    final int    DRIVE_SETTLE_FRAMES = 5;
    final double DRIVE_INTEGRAL_RANGE_IN = 4.0;
    final double DRIVE_OVERSHOOT_BRAKE_POWER = 0.25;

    // ============================================================
    // HEADING CORRECTION — strafe mode uses front/back pair asymmetry
    // ============================================================
    final double HEADING_KP = 0.025;
    final double HEADING_MAX_CORRECTION = 0.3;

    // ============================================================
    // WHEEL ALIGNMENT
    // ============================================================
    final double STEER_ALIGN_TOLERANCE_RAD = Math.toRadians(1.75);
    final long   STEER_ALIGN_TIMEOUT_MS = 1500;

    // ============================================================
    // STRAFE MODE
    // All four wheels lock at +90 degrees so the robot strafes.
    // Position feedback comes from the Y pod.
    //
    // Y_DIRECTION_SIGN: if pushing the robot in the intended "positive" strafe
    // direction makes the Y reading DECREASE on pinpointTest, flip this to -1.
    // This way driveDistance(positive) always moves the robot in your chosen
    // direction regardless of pod orientation.
    // ============================================================
    final double STRAFE_WHEEL_ANGLE_RAD = Math.PI / 2; // +90 degrees
    final double Y_DIRECTION_SIGN = +1.0;

    // ============================================================
    // UNITS
    // ============================================================
    final double MM_PER_INCH = 25.4;

    // ============================================================
    // HEADING SETTLE
    // ============================================================
    final double HEADING_SETTLE_TOLERANCE_DEG = 0.75;
    final long   HEADING_SETTLE_TIMEOUT_MS = 1500;

    private double autoStartHeadingDeg = 0;

    @Override
    public void runOpMode() {
        initializeHardware();

        telemetry.addLine("Initialized. Waiting for start.");
        telemetry.addLine("STRAFE MODE: wheels lock at +90, drives along Y axis.");
        telemetry.addData("Pinpoint status", odo.getDeviceStatus());
        telemetry.addData("Battery", "%.2f V", voltageSensor.getVoltage());
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        odo.update();
        autoStartHeadingDeg = odo.getHeading(AngleUnit.DEGREES);

        // ====================================================
        // EDIT THIS SEQUENCE PER RUN
        // Positive = strafe in +Y direction (after sign correction).
        // ====================================================
        driveDistance(24);
        sleep(1000);
        driveDistance(-24);
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
    // MAIN REUSABLE METHOD: strafe a signed distance in inches.
    // Positive = +Y direction (after Y_DIRECTION_SIGN correction).
    // ============================================================
    public void driveDistance(double inches) {
        odo.update();
        double startY = readPosYInches();
        double targetY = startY + inches;

        alignWheelsToStrafe();
        waitForHeadingSettle();

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

            double currentY = readPosYInches();
            double error = targetY - currentY;

            if (Math.abs(error) < DRIVE_TOLERANCE_IN) {
                settleCounter++;
                if (settleCounter >= DRIVE_SETTLE_FRAMES) {
                    break;
                }
            } else {
                settleCounter = 0;
            }
            if (now - startTime > DRIVE_TIMEOUT_MS) {
                break;
            }

            if (Math.abs(error) < DRIVE_INTEGRAL_RANGE_IN) {
                integral += error * dt;
            } else {
                integral = 0;
            }
            double derivative = (error - prevError) / dt;
            double basePower = DRIVE_KP * error
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

            // Heading correction — FRONT/BACK pair asymmetry (strafe mode).
            // VERIFY SIGN on the bench: with wheels at +90, if robot rotates the wrong
            // way during a strafe, flip the sign of `correction` below.
            // Reasoning: with wheels at +π/2 pointing left, positive drive power moves
            // the robot in one Y direction. If you push the front wheels harder than
            // the back wheels, the front of the robot strafes faster than the back,
            // which rotates the robot. Whether that rotation is CW or CCW depends on
            // which way the wheels physically point at +π/2.
            double currentHeading = readHeadingDegrees();
            double headingError = wrapAngleDegrees(autoStartHeadingDeg - currentHeading);
            double correction = clamp(HEADING_KP * headingError,
                    -HEADING_MAX_CORRECTION,
                    HEADING_MAX_CORRECTION);

            double frontPower = basePower - correction;
            double backPower  = basePower + correction;

            frontPower = clamp(frontPower, -1.0, 1.0);
            backPower  = clamp(backPower,  -1.0, 1.0);

            setDrivePowersFrontBack(frontPower, backPower);

            telemetry.addData("Phase", "STRAFING");
            telemetry.addData("Target Y (in)",  "%.2f", targetY);
            telemetry.addData("Current Y (in)", "%.2f", currentY);
            telemetry.addData("Error (in)",     "%.2f", error);
            telemetry.addData("Base Power",     "%.3f", basePower);
            telemetry.addData("Overshot?",      hasOvershot);
            telemetry.addData("Heading err (deg)", "%.2f", headingError);
            telemetry.addData("Correction",     "%.3f", correction);
            telemetry.addData("Front / Back",   "%.3f / %.3f", frontPower, backPower);
            telemetry.addData("Elapsed (ms)",   now - startTime);
            telemetry.update();

            prevError = error;
        }

        long stopUntil = System.currentTimeMillis() + 100;
        while (opModeIsActive() && System.currentTimeMillis() < stopUntil) {
            odo.update();
            setDrivePowersFrontBack(0, 0);
        }
        setDrivePowersFrontBack(0, 0);
    }

    // ============================================================
    // Align all four modules to STRAFE_WHEEL_ANGLE_RAD (+90 degrees).
    // ============================================================
    private void alignWheelsToStrafe() {
        long startTime = System.currentTimeMillis();

        while (opModeIsActive()) {
            odo.update();

            double errFL = wrapAngle(STRAFE_WHEEL_ANGLE_RAD - (getRawAngle(frontLeftEncoder)  - FRONT_LEFT_OFFSET));
            double errFR = wrapAngle(STRAFE_WHEEL_ANGLE_RAD - (getRawAngle(frontRightEncoder) - FRONT_RIGHT_OFFSET));
            double errBL = wrapAngle(STRAFE_WHEEL_ANGLE_RAD - (getRawAngle(backLeftEncoder)   - BACK_LEFT_OFFSET));
            double errBR = wrapAngle(STRAFE_WHEEL_ANGLE_RAD - (getRawAngle(backRightEncoder)  - BACK_RIGHT_OFFSET));

            runModule(frontLeftDrive,  frontLeftSteer,  frontLeftEncoder,  FRONT_LEFT_OFFSET,  0, STRAFE_WHEEL_ANGLE_RAD);
            runModule(frontRightDrive, frontRightSteer, frontRightEncoder, FRONT_RIGHT_OFFSET, 0, STRAFE_WHEEL_ANGLE_RAD);
            runModule(backLeftDrive,   backLeftSteer,   backLeftEncoder,   BACK_LEFT_OFFSET,   0, STRAFE_WHEEL_ANGLE_RAD);
            runModule(backRightDrive,  backRightSteer,  backRightEncoder,  BACK_RIGHT_OFFSET,  0, STRAFE_WHEEL_ANGLE_RAD);

            boolean aligned = Math.abs(errFL) < STEER_ALIGN_TOLERANCE_RAD
                    && Math.abs(errFR) < STEER_ALIGN_TOLERANCE_RAD
                    && Math.abs(errBL) < STEER_ALIGN_TOLERANCE_RAD
                    && Math.abs(errBR) < STEER_ALIGN_TOLERANCE_RAD;

            long elapsed = System.currentTimeMillis() - startTime;

            telemetry.addData("Phase", "ALIGNING STRAFE (+90 deg)");
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

    // ============================================================
    // Wait until heading is within HEADING_SETTLE_TOLERANCE_DEG of the
    // auto-start heading. Holds wheels at strafe angle with no drive power.
    // ============================================================
    private void waitForHeadingSettle() {
        long startTime = System.currentTimeMillis();

        while (opModeIsActive()) {
            odo.update();

            double currentHeading = readHeadingDegrees();
            double headingError = wrapAngleDegrees(autoStartHeadingDeg - currentHeading);

            setDrivePowersFrontBack(0, 0);

            long elapsed = System.currentTimeMillis() - startTime;

            telemetry.addData("Phase", "HEADING SETTLE");
            telemetry.addData("Target heading (deg)", "%.3f", autoStartHeadingDeg);
            telemetry.addData("Current heading (deg)", "%.3f", currentHeading);
            telemetry.addData("Heading error (deg)", "%.3f", headingError);
            telemetry.addData("Tolerance (deg)", "%.3f", HEADING_SETTLE_TOLERANCE_DEG);
            telemetry.addData("Elapsed (ms)", elapsed);
            telemetry.update();

            if (Math.abs(headingError) < HEADING_SETTLE_TOLERANCE_DEG) break;
            if (elapsed > HEADING_SETTLE_TIMEOUT_MS) break;
        }
    }

    // ============================================================
    // Apply drive powers grouped by front pair / back pair.
    // All modules hold target angle STRAFE_WHEEL_ANGLE_RAD.
    // ============================================================
    private void setDrivePowersFrontBack(double frontPower, double backPower) {
        runModule(frontLeftDrive,  frontLeftSteer,  frontLeftEncoder,  FRONT_LEFT_OFFSET,  frontPower, STRAFE_WHEEL_ANGLE_RAD);
        runModule(frontRightDrive, frontRightSteer, frontRightEncoder, FRONT_RIGHT_OFFSET, frontPower, STRAFE_WHEEL_ANGLE_RAD);
        runModule(backLeftDrive,   backLeftSteer,   backLeftEncoder,   BACK_LEFT_OFFSET,   backPower,  STRAFE_WHEEL_ANGLE_RAD);
        runModule(backRightDrive,  backRightSteer,  backRightEncoder,  BACK_RIGHT_OFFSET,  backPower,  STRAFE_WHEEL_ANGLE_RAD);
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
        servoPower *= -1; // Steering Fix: invert servo power to match physical rotation

        if (Math.abs(servoPower) < STEER_DEADBAND) servoPower = 0;

        servoPower = Math.max(-1, Math.min(1, servoPower));

        steerServo.setPower(servoPower);
        driveMotor.setPower(speed);
    }

    // ============================================================
    // Pinpoint readers
    // ============================================================
    private double readPosYInches() {
        // Y_DIRECTION_SIGN lets us flip the sense of "positive Y" without touching pod direction config.
        return odo.getPosY(DistanceUnit.INCH) * Y_DIRECTION_SIGN;
    }

    private double readHeadingDegrees() {
        return odo.getHeading(AngleUnit.DEGREES);
    }

    // ============================================================
    // Angle helpers
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