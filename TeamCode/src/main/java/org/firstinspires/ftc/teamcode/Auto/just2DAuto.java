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
 * just2DAuto — unified 2D auto for the swerve drive.
 *
 * Three reusable methods:
 *   driveX(inches)        - forward/back driving (wheels at 0°, L/R heading correction)
 *   driveY(inches)        - strafe driving       (wheels at 90°, F/B heading correction)
 *   correctHeading()      - active rotate back to auto-start heading (in-place spin)
 *
 * All methods lock the wheels to the correct angle before applying drive power
 * and wait for them to be aligned within STEER_ALIGN_TOLERANCE_RAD first.
 *
 * Sign conventions:
 *   driveX(+) = +X direction (Pinpoint X increases)
 *   driveY(+) = +Y direction after Y_DIRECTION_SIGN correction
 */
@Autonomous(name = "just2DAuto", group = "Swerve")
public class just2DAuto extends LinearOpMode {

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
    // Confirmed: X pod FORWARD, Y pod REVERSED.
    // ============================================================
    final GoBildaPinpointDriver.EncoderDirection X_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    final GoBildaPinpointDriver.EncoderDirection Y_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;

    final double X_POD_OFFSET_MM =  41.9999922;   // X pod, left of center
    final double Y_POD_OFFSET_MM = -148.3535768;  // Y pod, behind center

    // Y_DIRECTION_SIGN: extra flip if positive driveY moves wrong physical direction.
    final double Y_DIRECTION_SIGN = +1.0;

    // ============================================================
    // DISTANCE PID — shared between driveX and driveY.
    // Tuned values from justAuto. If X and Y need different gains later,
    // duplicate these into DRIVE_X_KP / DRIVE_Y_KP etc.
    // ============================================================
    final double DRIVE_KP = 0.10;
    final double DRIVE_KI = 0.0;
    final double DRIVE_KD = 0.05;
    final double DRIVE_KS = 0.05;
    final double DRIVE_TOLERANCE_IN = 0.5;
    final double DRIVE_MAX_POWER = 0.75;
    final double DRIVE_MIN_POWER = 0.15;
    final long   DRIVE_TIMEOUT_MS = 6000;
    final int    DRIVE_SETTLE_FRAMES = 5;
    final double DRIVE_INTEGRAL_RANGE_IN = 4.0;
    final double DRIVE_OVERSHOOT_BRAKE_POWER = 0.25;

    // ============================================================
    // IN-DRIVE HEADING CORRECTION (small left/right or front/back asymmetry
    // applied DURING driveX or driveY to fight drift)
    // ============================================================
    final double HEADING_KP = 0.025;             // power difference per degree of heading error
    final double HEADING_MAX_CORRECTION = 0.3;   // cap on asymmetry

    // ============================================================
    // STANDALONE HEADING CORRECTION (correctHeading() method)
    // Used between moves to actively rotate back to the auto-start heading.
    // Separate PID knobs so it can be tuned independently of in-drive correction.
    // ============================================================
    final double HEADING_CORRECT_KP = 0.020;        // rotation power per degree of error
    final double HEADING_CORRECT_KD = 0.002;        // damping
    final double HEADING_CORRECT_KS = 0.05;         // static feedforward to overcome stiction
    final double HEADING_CORRECT_TOLERANCE_DEG = 0.75;
    final double HEADING_CORRECT_MAX_POWER = 0.4;   // cap on rotation power
    final double HEADING_CORRECT_MIN_POWER = 0.10;  // floor to break stiction
    final long   HEADING_CORRECT_TIMEOUT_MS = 2500;
    final int    HEADING_CORRECT_SETTLE_FRAMES = 5;

    // ============================================================
    // WHEEL ALIGNMENT BEFORE DRIVING
    // ============================================================
    final double STEER_ALIGN_TOLERANCE_RAD = Math.toRadians(1.75);
    final long   STEER_ALIGN_TIMEOUT_MS = 1500;

    // ============================================================
    // WHEEL TARGET ANGLES
    // ============================================================
    final double WHEELS_FORWARD_RAD = 0.0;       // driveX
    final double WHEELS_SIDEWAYS_RAD = Math.PI / 2; // driveY (+90 degrees)

    // ============================================================
    // UNITS
    // ============================================================
    final double MM_PER_INCH = 25.4;

    // Captured once at the start of auto. correctHeading() targets this.
    private double autoStartHeadingDeg = 0;

    @Override
    public void runOpMode() {
        initializeHardware();

        telemetry.addLine("Initialized. Waiting for start.");
        telemetry.addData("Pinpoint status", odo.getDeviceStatus());
        telemetry.addData("Battery", "%.2f V", voltageSensor.getVoltage());
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        // Capture the absolute heading reference for this entire auto run.
        odo.update();
        autoStartHeadingDeg = odo.getHeading(AngleUnit.DEGREES);

        // ====================================================
        // EDIT THIS SEQUENCE PER RUN
        // driveX(+) = forward,  driveX(-) = backward
        // driveY(+) = strafe in +Y,  driveY(-) = strafe in -Y
        // correctHeading() = rotate back to auto-start heading
        // ====================================================
        driveX(24);
        sleep(500);
        correctHeading();
        sleep(500);
        driveY(24);
        sleep(500);
        correctHeading();
        sleep(500);
        driveX(-24);
        sleep(500);
        correctHeading();
        sleep(500);
        driveY(-24);
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
    // Wheels at 0°. Position feedback from Pinpoint X (inches).
    // Heading correction via left-pair vs right-pair drive power asymmetry.
    // ============================================================
    public void driveX(double inches) {
        odo.update();
        double startPos = readPosXInches();
        double targetPos = startPos + inches;

        alignWheelsTo(WHEELS_FORWARD_RAD, "ALIGNING FORWARD");

        runDistancePID(
                inches, targetPos,
                /* readPos        */ true,   // true = read X, false = read Y
                WHEELS_FORWARD_RAD,
                /* useLeftRightHeading */ true
        );
    }

    // ============================================================
    // driveY — strafe.
    // Wheels at +90°. Position feedback from Pinpoint Y (inches).
    // Heading correction via front-pair vs back-pair drive power asymmetry.
    // ============================================================
    public void driveY(double inches) {
        odo.update();
        double startPos = readPosYInches();
        double targetPos = startPos + inches;

        alignWheelsTo(WHEELS_SIDEWAYS_RAD, "ALIGNING SIDEWAYS");

        runDistancePID(
                inches, targetPos,
                /* readPos        */ false,  // read Y
                WHEELS_SIDEWAYS_RAD,
                /* useLeftRightHeading */ false
        );
    }

    // ============================================================
    // Shared distance PID loop. Used by both driveX and driveY.
    //   inches:       requested distance (for direction sign / overshoot detection)
    //   targetPos:    absolute target position (inches)
    //   readX:        true = read Pinpoint X, false = read Pinpoint Y
    //   wheelAngle:   target wheel angle (0 or +π/2)
    //   useLeftRight: true = L/R asymmetry, false = F/B asymmetry
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

            // Settle / exit checks
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

            // PID with anti-windup
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

            // Overshoot brake
            if (initialErrorSign != 0 && Math.signum(error) != initialErrorSign
                    && Math.abs(error) > DRIVE_TOLERANCE_IN) {
                hasOvershot = true;
            }
            if (hasOvershot && Math.abs(error) > DRIVE_TOLERANCE_IN) {
                basePower = DRIVE_OVERSHOOT_BRAKE_POWER * Math.signum(error);
            }

            // Heading correction asymmetry
            double currentHeading = readHeadingDegrees();
            double headingError = wrapAngleDegrees(autoStartHeadingDeg - currentHeading);
            double correction = clamp(HEADING_KP * headingError,
                    -HEADING_MAX_CORRECTION,
                    HEADING_MAX_CORRECTION);

            // Apply asymmetry in the appropriate pair grouping
            double powerA = clamp(basePower - correction, -1.0, 1.0); // left pair OR front pair
            double powerB = clamp(basePower + correction, -1.0, 1.0); // right pair OR back pair

            if (useLeftRight) {
                setDrivePowersLeftRight(powerA, powerB, wheelAngle);
            } else {
                setDrivePowersFrontBack(powerA, powerB, wheelAngle);
            }

            // Telemetry
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

        // Brief stop / hold
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
    // Uses the same in-place rotation as justSwerve: zero translation, pure rotation.
    // Each wheel goes to its own kinematics angle and spins to rotate the chassis.
    // PID with its own tuning knobs (HEADING_CORRECT_*).
    //
    // VERIFY SIGN: if the robot rotates the WRONG way (away from target),
    // flip the sign of `rot` below (negate it).
    // ============================================================
    public void correctHeading() {
        long startTime = System.currentTimeMillis();
        long lastTime = startTime;
        double prevError = 0;
        int settleCounter = 0;

        // Prime prevError so first derivative is zero
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

            // Settle / exit checks
            if (Math.abs(error) < HEADING_CORRECT_TOLERANCE_DEG) {
                settleCounter++;
                if (settleCounter >= HEADING_CORRECT_SETTLE_FRAMES) {
                    break;
                }
            } else {
                settleCounter = 0;
            }
            if (now - startTime > HEADING_CORRECT_TIMEOUT_MS) {
                break;
            }

            // PD + static FF
            double derivative = (error - prevError) / dt;
            double rotPower = HEADING_CORRECT_KP * error
                    + HEADING_CORRECT_KD * derivative
                    + HEADING_CORRECT_KS * Math.signum(error);

            // Clamp + floor
            rotPower = clamp(rotPower, -HEADING_CORRECT_MAX_POWER, HEADING_CORRECT_MAX_POWER);
            if (Math.abs(rotPower) < HEADING_CORRECT_MIN_POWER
                    && Math.abs(error) > HEADING_CORRECT_TOLERANCE_DEG) {
                rotPower = HEADING_CORRECT_MIN_POWER * Math.signum(error);
            }

            // Apply pure rotation using swerve kinematics.
            // robotX = 0, robotY = 0, rot = rotPower
            // A = -rot * (WHEELBASE / R)
            // B = +rot * (WHEELBASE / R)
            // C = -rot * (TRACK_WIDTH / R)
            // D = +rot * (TRACK_WIDTH / R)
            // VERIFY SIGN: if robot turns away from target, negate rotPower here.
            double rot = rotPower;
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

        // Stop all drive motors after settling
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
    // Align all four modules to a target angle, telemetry-labeled.
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

            // For wheels in normal forward, the alignment check uses delta directly.
            // For wheels at ±π/2, runModule may flip speed sign and target by π if shorter;
            // but for steering check we want raw mechanical alignment to targetAngle OR targetAngle+π.
            // Either is fine since the drive direction is sign-handled inside runModule.
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

    /** A module is "aligned" if either error or (error + π) wrapped is within tolerance,
     *  because the drive motor can spin either direction so the wheel itself only needs
     *  to be on the right axis, not the right "facing." */
    private boolean isAlignedTo(double angleError) {
        double e1 = Math.abs(angleError);
        double e2 = Math.abs(wrapAngle(angleError + Math.PI));
        return Math.min(e1, e2) < STEER_ALIGN_TOLERANCE_RAD;
    }

    // ============================================================
    // Drive power application — left-pair vs right-pair grouping
    // ============================================================
    private void setDrivePowersLeftRight(double leftPower, double rightPower, double wheelAngle) {
        runModule(frontLeftDrive,  frontLeftSteer,  frontLeftEncoder,  FRONT_LEFT_OFFSET,  leftPower,  wheelAngle);
        runModule(backLeftDrive,   backLeftSteer,   backLeftEncoder,   BACK_LEFT_OFFSET,   leftPower,  wheelAngle);
        runModule(frontRightDrive, frontRightSteer, frontRightEncoder, FRONT_RIGHT_OFFSET, rightPower, wheelAngle);
        runModule(backRightDrive,  backRightSteer,  backRightEncoder,  BACK_RIGHT_OFFSET,  rightPower, wheelAngle);
    }

    // ============================================================
    // Drive power application — front-pair vs back-pair grouping
    // ============================================================
    private void setDrivePowersFrontBack(double frontPower, double backPower, double wheelAngle) {
        runModule(frontLeftDrive,  frontLeftSteer,  frontLeftEncoder,  FRONT_LEFT_OFFSET,  frontPower, wheelAngle);
        runModule(frontRightDrive, frontRightSteer, frontRightEncoder, FRONT_RIGHT_OFFSET, frontPower, wheelAngle);
        runModule(backLeftDrive,   backLeftSteer,   backLeftEncoder,   BACK_LEFT_OFFSET,   backPower,  wheelAngle);
        runModule(backRightDrive,  backRightSteer,  backRightEncoder,  BACK_RIGHT_OFFSET,  backPower,  wheelAngle);
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