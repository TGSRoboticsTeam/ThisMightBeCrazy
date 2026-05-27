package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Disabled
@Autonomous(name = "justAuto", group = "Swerve")
public class justAuto extends LinearOpMode {

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
    // Flip FORWARD <-> REVERSED below to invert a pod's direction.
    // User-confirmed: when robot drives forward, the X pod's wheel spins.
    // The X pod sits 41.99mm LEFT and 146.81mm BACK of robot center.
    // The Y pod sits 38.97mm RIGHT and 148.35mm BACK of robot center.
    //
    // VERIFY: setOffsets(xOffset, yOffset) in the goBILDA driver expects
    // the perpendicular distance of each pod from the tracking center, in mm.
    //   xOffset = X pod's lateral distance from center (LEFT = positive)
    //   yOffset = Y pod's forward distance from center (FORWARD = positive)
    // X pod is 41.99mm LEFT  -> +41.9999922
    // Y pod is 148.35mm BACK -> -148.3535768
    // If forward driving causes Pinpoint X to DECREASE, flip X_POD_DIRECTION.
    // If the robot thinks it's strafing when it isn't, flip Y_POD_DIRECTION
    // or check the sign of Y_POD_OFFSET_MM.
    // ============================================================
    final GoBildaPinpointDriver.EncoderDirection X_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    final GoBildaPinpointDriver.EncoderDirection Y_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    final double X_POD_OFFSET_MM =  41.9999922;   // X pod, left of center
    final double Y_POD_OFFSET_MM = -148.3535768;  // Y pod, behind center

    // ============================================================
    // DISTANCE PID (tuned for up to ~55" travel, units in inches)
    // ============================================================
    final double DRIVE_KP = 0.10;        // power per inch of error (raised from 0.04 — was too slow)
    final double DRIVE_KI = 0.0;         // off by default
    final double DRIVE_KD = 0.05;        // damping
    final double DRIVE_KS = 0.05;        // static feedforward to overcome stiction (raised from 0.03)
    final double DRIVE_TOLERANCE_IN = 0.5;
    final double DRIVE_MAX_POWER = 0.75; // cap on commanded power
    final double DRIVE_MIN_POWER = 0.15; // floor when error > tolerance (raised from 0.08 to escape low-speed crawl)
    final long   DRIVE_TIMEOUT_MS = 6000; // raised from 4000 to give a 55" move headroom
    final int    DRIVE_SETTLE_FRAMES = 5; // must be within tolerance for N consecutive loops
    final double DRIVE_INTEGRAL_RANGE_IN = 4.0; // only accumulate integral when |error| < this (anti-windup)

    // Active braking: when we overshoot (error flips sign past target),
    // apply this much extra reverse power to actively stop the robot.
    // Set to 0 to disable; raise if overshoot is still bad.
    final double DRIVE_OVERSHOOT_BRAKE_POWER = 0.25;

    // ============================================================
    // HEADING CORRECTION (tank-style left/right asymmetry)
    // ============================================================
    final double HEADING_KP = 0.025;             // power difference per degree of heading error
    final double HEADING_MAX_CORRECTION = 0.3;   // cap on left/right asymmetry

    // ============================================================
    // WHEEL ALIGNMENT BEFORE DRIVING
    // ============================================================
    final double STEER_ALIGN_TOLERANCE_RAD = Math.toRadians(1.75); // 1.75 degrees
    final long   STEER_ALIGN_TIMEOUT_MS = 1500;

    // ============================================================
    // UNITS
    // ============================================================
    final double MM_PER_INCH = 25.4;

    // ============================================================
    // HEADING SETTLE BEFORE EACH DRIVE
    // Wait for heading to return within this tolerance of the auto-start heading
    // before beginning each driveDistance. Prevents compounding errors between moves.
    // ============================================================
    final double HEADING_SETTLE_TOLERANCE_DEG = 0.75;
    final long   HEADING_SETTLE_TIMEOUT_MS = 1500;

    // Captured once at the start of auto. All driveDistance calls correct toward this.
    private double autoStartHeadingDeg = 0;

    // ============================================================
    // VERIFY block — Pinpoint API methods used in this file:
    //   odo.setOffsets(double xMM, double yMM)
    //   odo.setEncoderResolution(GoBildaOdometryPods)
    //   odo.setEncoderDirections(EncoderDirection, EncoderDirection)
    //   odo.resetPosAndIMU()
    //   odo.update()
    //   odo.getPosX()                  -> millimeters
    //   odo.getHeading()               -> radians (CCW positive)
    // If your installed driver version differs (e.g. unit-aware getters
    // like getPosition().getX(DistanceUnit.INCH)), adjust readPosXInches()
    // and readHeadingDegrees() below — those are the only two touch points.
    // ============================================================

    @Override
    public void runOpMode() {
        initializeHardware();

        telemetry.addLine("Initialized. Waiting for start.");
        telemetry.addData("Pinpoint status", odo.getDeviceStatus());
        telemetry.addData("Battery", "%.2f V", voltageSensor.getVoltage());
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        // Capture the heading at the moment auto begins. All subsequent
        // driveDistance calls will correct toward this absolute reference.
        odo.update();
        autoStartHeadingDeg = odo.getHeading(AngleUnit.DEGREES);

        // ====================================================
        // EDIT THIS SEQUENCE PER RUN
        // Positive = forward, Negative = backward. Units: inches.
        // ====================================================
        driveDistance(24);   // forward 24 inches
        sleep(1000);         // pause between steps
        driveDistance(-24);  // back to start
        // ====================================================
        // END EDITABLE SEQUENCE
        // ====================================================

        telemetry.addLine("Auto complete.");
        telemetry.update();
    }

    // ============================================================
    // HARDWARE INITIALIZATION
    // ============================================================
    private void initializeHardware() {
        // Swerve drive hardware
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

        // Drive motor directions (copied from justSwerve)
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Zero-power behavior: BRAKE for crisp stops
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetMotors(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);

        // Pinpoint initialization
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(X_POD_OFFSET_MM, Y_POD_OFFSET_MM, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(X_POD_DIRECTION, Y_POD_DIRECTION);
        odo.resetPosAndIMU();
    }

    // ============================================================
    // MAIN REUSABLE METHOD: drive a signed distance in inches.
    // Positive = forward, Negative = backward.
    // Holds heading via tank-style left/right asymmetry. Wheels stay at 0.
    // ============================================================
    public void driveDistance(double inches) {
        // Capture starting state
        odo.update();
        double startX = readPosXInches();
        // Heading target is the absolute auto-start heading, NOT this call's heading.
        // This prevents heading drift from compounding across multiple driveDistance calls.
        double targetX = startX + inches;

        // Ensure all four modules are pointing forward (angle 0) before applying drive power.
        alignWheelsForward();

        // Ensure heading has settled back to the auto-start reference before driving.
        waitForHeadingSettle();

        // PID state
        double prevError = inches;
        double integral = 0.0;
        long startTime = System.currentTimeMillis();
        long lastTime = startTime;
        int settleCounter = 0;

        // Track initial direction of travel so we know what "overshoot" looks like.
        // If we started needing to go forward (inches > 0), an overshoot means error < 0.
        // If we started needing to go backward (inches < 0), an overshoot means error > 0.
        double initialErrorSign = Math.signum(inches);
        boolean hasOvershot = false;

        while (opModeIsActive()) {
            // Refresh Pinpoint
            odo.update();

            // Time step
            long now = System.currentTimeMillis();
            double dt = (now - lastTime) / 1000.0;
            if (dt <= 0) dt = 0.001;
            lastTime = now;

            // Distance error
            double currentX = readPosXInches();
            double error = targetX - currentX;

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

            // PID — only accumulate integral when close to target (anti-windup)
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

            // Clamp
            basePower = clamp(basePower, -DRIVE_MAX_POWER, DRIVE_MAX_POWER);

            // Floor: don't let it stall short of the target
            if (Math.abs(basePower) < DRIVE_MIN_POWER && Math.abs(error) > DRIVE_TOLERANCE_IN) {
                basePower = DRIVE_MIN_POWER * Math.signum(error);
            }

            // Overshoot brake: if error has flipped sign from our initial direction,
            // we've passed the target. Override basePower with a firm reverse pulse
            // to actively stop the chassis instead of relying on small reverse PID + brake mode.
            // Once we've overshot once, stay in brake mode until tolerance is reached.
            if (initialErrorSign != 0 && Math.signum(error) != initialErrorSign
                    && Math.abs(error) > DRIVE_TOLERANCE_IN) {
                hasOvershot = true;
            }
            if (hasOvershot && Math.abs(error) > DRIVE_TOLERANCE_IN) {
                basePower = DRIVE_OVERSHOOT_BRAKE_POWER * Math.signum(error);
            }

            // Heading correction
            // VERIFY SIGN: Pinpoint heading is CCW-positive. If robot drifts
            // counter-clockwise (heading increases) we want LEFT side to push
            // forward MORE than right -> rightPower decreases.
            //   headingError = autoStartHeadingDeg - currentHeading
            //   If robot drifted CCW: currentHeading > startHeading -> headingError < 0
            //   -> correction < 0 -> leftPower = base - corr (bigger), rightPower = base + corr (smaller) -> turns CW back. Good.
            // If on the bench the correction is BACKWARDS (robot spirals out),
            // flip the sign of `correction` below.
            double currentHeading = readHeadingDegrees();
            double headingError = wrapAngleDegrees(autoStartHeadingDeg - currentHeading);
            double correction = clamp(HEADING_KP * headingError,
                    -HEADING_MAX_CORRECTION,
                    HEADING_MAX_CORRECTION);

            double leftPower  = basePower - correction;
            double rightPower = basePower + correction;

            // Clamp final outputs
            leftPower  = clamp(leftPower,  -1.0, 1.0);
            rightPower = clamp(rightPower, -1.0, 1.0);

            // Apply: all wheels target angle 0, left pair gets leftPower, right pair gets rightPower
            setDrivePowers(leftPower, rightPower);

            // Telemetry
            telemetry.addData("Phase", "DRIVING");
            telemetry.addData("Target (in)",  "%.2f", targetX);
            telemetry.addData("Current (in)", "%.2f", currentX);
            telemetry.addData("Error (in)",   "%.2f", error);
            telemetry.addData("Base Power",   "%.3f", basePower);
            telemetry.addData("Overshot?",    hasOvershot);
            telemetry.addData("Heading err (deg)", "%.2f", headingError);
            telemetry.addData("Correction",   "%.3f", correction);
            telemetry.addData("Left / Right", "%.3f / %.3f", leftPower, rightPower);
            telemetry.addData("Elapsed (ms)", now - startTime);
            telemetry.update();

            prevError = error;
        }

        // Brief stop / hold at 0
        long stopUntil = System.currentTimeMillis() + 100;
        while (opModeIsActive() && System.currentTimeMillis() < stopUntil) {
            odo.update();
            setDrivePowers(0, 0);
        }
        // Final stop
        setDrivePowers(0, 0);
    }

    // ============================================================
    // Align all four modules to angle 0 (wheels pointing forward).
    // Exits when all modules are within STEER_ALIGN_TOLERANCE_RAD of 0,
    // or STEER_ALIGN_TIMEOUT_MS has elapsed.
    // ============================================================
    private void alignWheelsForward() {
        long startTime = System.currentTimeMillis();

        while (opModeIsActive()) {
            odo.update();

            double errFL = wrapAngle(0 - (getRawAngle(frontLeftEncoder)  - FRONT_LEFT_OFFSET));
            double errFR = wrapAngle(0 - (getRawAngle(frontRightEncoder) - FRONT_RIGHT_OFFSET));
            double errBL = wrapAngle(0 - (getRawAngle(backLeftEncoder)   - BACK_LEFT_OFFSET));
            double errBR = wrapAngle(0 - (getRawAngle(backRightEncoder)  - BACK_RIGHT_OFFSET));

            // Drive modules toward angle 0 with zero drive power
            runModule(frontLeftDrive,  frontLeftSteer,  frontLeftEncoder,  FRONT_LEFT_OFFSET,  0, 0);
            runModule(frontRightDrive, frontRightSteer, frontRightEncoder, FRONT_RIGHT_OFFSET, 0, 0);
            runModule(backLeftDrive,   backLeftSteer,   backLeftEncoder,   BACK_LEFT_OFFSET,   0, 0);
            runModule(backRightDrive,  backRightSteer,  backRightEncoder,  BACK_RIGHT_OFFSET,  0, 0);

            boolean aligned = Math.abs(errFL) < STEER_ALIGN_TOLERANCE_RAD
                    && Math.abs(errFR) < STEER_ALIGN_TOLERANCE_RAD
                    && Math.abs(errBL) < STEER_ALIGN_TOLERANCE_RAD
                    && Math.abs(errBR) < STEER_ALIGN_TOLERANCE_RAD;

            long elapsed = System.currentTimeMillis() - startTime;

            telemetry.addData("Phase", "ALIGNING");
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

        // Stop steer servos after alignment exit
        frontLeftSteer.setPower(0);
        frontRightSteer.setPower(0);
        backLeftSteer.setPower(0);
        backRightSteer.setPower(0);
    }

    // ============================================================
    // Wait until heading is within HEADING_SETTLE_TOLERANCE_DEG of the
    // auto-start heading, or until HEADING_SETTLE_TIMEOUT_MS elapses.
    // Holds wheels at 0 and motors off while waiting.
    // ============================================================
    private void waitForHeadingSettle() {
        long startTime = System.currentTimeMillis();

        while (opModeIsActive()) {
            odo.update();

            double currentHeading = readHeadingDegrees();
            double headingError = wrapAngleDegrees(autoStartHeadingDeg - currentHeading);

            // Hold wheels at 0 with no drive power while waiting
            setDrivePowers(0, 0);

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
    // Apply drive powers: left pair = leftPower, right pair = rightPower.
    // All modules continue to hold target angle 0.
    // ============================================================
    private void setDrivePowers(double leftPower, double rightPower) {
        runModule(frontLeftDrive,  frontLeftSteer,  frontLeftEncoder,  FRONT_LEFT_OFFSET,  leftPower,  0);
        runModule(backLeftDrive,   backLeftSteer,   backLeftEncoder,   BACK_LEFT_OFFSET,   leftPower,  0);
        runModule(frontRightDrive, frontRightSteer, frontRightEncoder, FRONT_RIGHT_OFFSET, rightPower, 0);
        runModule(backRightDrive,  backRightSteer,  backRightEncoder,  BACK_RIGHT_OFFSET,  rightPower, 0);
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
    // Pinpoint readers — single touch-points for unit/method changes
    // ============================================================
    private double readPosXInches() {
        // Newer goBILDA driver: unit-aware getter. Returns inches directly.
        // If your driver only has getPosX() (returns mm), swap to:
        //   return odo.getPosX() / MM_PER_INCH;
        return odo.getPosX(DistanceUnit.INCH);
    }

    private double readHeadingDegrees() {
        // Newer goBILDA driver: unit-aware getter. Returns degrees directly.
        // If your driver only has getHeading() (returns radians), swap to:
        //   return Math.toDegrees(odo.getHeading());
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