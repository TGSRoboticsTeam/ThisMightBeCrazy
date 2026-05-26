package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name = "omnipotentBackAuto", group = "Swerve")
public class omnipotentBackAuto extends LinearOpMode {

    // ============================================================
    // HARDWARE
    // ============================================================
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private CRServo frontLeftSteer, frontRightSteer, backLeftSteer, backRightSteer;
    private AnalogInput frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder;
    private GoBildaPinpointDriver odo;
    private VoltageSensor voltageSensor;

    private DcMotorEx topIntake, bottomIntake;
    private DcMotor leftFly, rightFly;
    private Servo blocker;

    // ============================================================
    // ROBOT GEOMETRY
    // ============================================================
    final double TRACK_WIDTH = 17.258;
    final double WHEELBASE   = 13.544;
    final double R = Math.hypot(TRACK_WIDTH, WHEELBASE);

    // ============================================================
    // SWERVE MODULE ENCODER OFFSETS
    // ============================================================
    final double FRONT_LEFT_OFFSET  = 0.1200;
    final double FRONT_RIGHT_OFFSET = 1.3861;
    final double BACK_LEFT_OFFSET   = 1.6965;
    final double BACK_RIGHT_OFFSET  = 0.8225;

    // ============================================================
    // STEERING TUNING
    // ============================================================
    final double STEER_KP       = 0.6;
    final double STEER_DEADBAND = 0.05;

    // ============================================================
    // PINPOINT CONFIGURATION
    // ============================================================
    final GoBildaPinpointDriver.EncoderDirection X_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    final GoBildaPinpointDriver.EncoderDirection Y_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;

    final double X_POD_OFFSET_MM =  41.9999922;
    final double Y_POD_OFFSET_MM = -148.3535768;
    final double Y_DIRECTION_SIGN = +1.0;

    // ============================================================
    // DISTANCE PID
    // ============================================================
    final double DRIVE_KP                 = 0.15;
    final double DRIVE_KI                 = 0.0;
    final double DRIVE_KD                 = 0.03;
    final double DRIVE_KS                 = 0.05;
    final double DRIVE_TOLERANCE_IN       = 0.5;
    final double DRIVE_MAX_POWER          = 0.75;
    final double DRIVE_MIN_POWER          = 0.20;
    final double DRIVE_P_MIN_POWER        = 0.20;
    final long   DRIVE_TIMEOUT_MS         = 1500;
    final int    DRIVE_SETTLE_FRAMES      = 5;
    final double DRIVE_INTEGRAL_RANGE_IN  = 4.0;
    final double DRIVE_OVERSHOOT_BRAKE_POWER = 0.25;

    // ============================================================
    // IN-DRIVE HEADING CORRECTION
    // ============================================================
    final double HEADING_KP             = 0.025;
    final double HEADING_MAX_CORRECTION = 0.3;

    // ============================================================
    // STANDALONE HEADING CORRECTION
    // ============================================================
    final double HEADING_CORRECT_KP              = 0.040;
    final double HEADING_CORRECT_KD              = 0.002;
    final double HEADING_CORRECT_KS              = 0.05;
    final double HEADING_CORRECT_TOLERANCE_DEG   = 2;
    final double HEADING_CORRECT_MAX_POWER       = 0.5;
    final double HEADING_CORRECT_MIN_POWER       = 0.22;
    final double HEADING_CORRECT_P_MIN_POWER     = 0.22;
    final long   HEADING_CORRECT_TIMEOUT_MS      = 750;
    final int    HEADING_CORRECT_SETTLE_FRAMES   = 5;

    // ============================================================
    // WHEEL ALIGNMENT
    // ============================================================
    final double STEER_ALIGN_TOLERANCE_RAD = Math.toRadians(1.75);
    final long   STEER_ALIGN_TIMEOUT_MS    = 1500;

    final double WHEELS_FORWARD_RAD  = 0.0;
    final double WHEELS_SIDEWAYS_RAD = Math.PI / 2;

    final double MM_PER_INCH = 25.4;

    // ============================================================
    // MECHANISM CONSTANTS
    // ============================================================
    final double BLOCKER_BLOCKED_POSITION        = 0.15;
    final double BLOCKER_LAUNCH_POSITION         = 0.45;

    final double INTAKE_FORWARD_POWER            = 0.90;
    final double INTAKE_REVERSE_HOLD_POWER       = 0.50;

    final double LAUNCHER_POWER                  = 0.90;
    final double MOTOR_COAST_RAMP_SECONDS        = 0.5;

    final double INTAKE_STALL_CURRENT_AMPS       = 7.0;
    final double INTAKE_STALL_TIME_SECONDS       = 1.0;

    // ApolloDrive-mirrored launch sequence constants
    final double BLOCKER_FLYWHEEL_SPINUP_SECONDS = 1.2;
    final double BLOCKER_LAUNCH_INTAKE_DELAY     = 0.4;
    final double BLOCKER_AUTO_RETURN_SECONDS     = 2.0;

    private double autoStartHeadingDeg = 0;

    // ============================================================
    // MATCH TIMER / SEQUENCE PARAMETERS
    // ============================================================
    private final ElapsedTime matchTimer = new ElapsedTime();
    final double MATCH_CUTOFF_SECONDS = 28.0;

    final double FORWARD_COLLECT_FEET               = 3.75;
    final double COLLECT_PUSH_FEET                  = 0.25;
    final double CORNER_FORWARD_FEET                = 4.00;
    final double COLLECT_POST_DISTANCE_GRACE_SECONDS = 1.0;

    private boolean matchTimeUp() {
        return matchTimer.seconds() >= MATCH_CUTOFF_SECONDS;
    }

    // ============================================================
    // runOpMode
    // ============================================================
    @Override
    public void runOpMode() {
        initializeHardware();

        blockerBlocked();

        telemetry.addLine("omnipotentBackAuto initialized. Waiting for start.");
        telemetry.addData("Pinpoint status", odo.getDeviceStatus());
        telemetry.addData("Battery", "%.2f V", voltageSensor.getVoltage());
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        odo.update();
        autoStartHeadingDeg = odo.getHeading(AngleUnit.DEGREES);
        double startX = readPosXInches();
        matchTimer.reset();

        // ====================================================
        // INITIAL LAUNCH — fire preloaded element
        // ====================================================
        launchSequence();

        // ====================================================
        // COLLECT-AND-LAUNCH CYCLES
        // ====================================================
        while (opModeIsActive() && !matchTimeUp()) {

            // 1. Drive forward to collect zone
            driveX(FORWARD_COLLECT_FEET * 12.0);
            if (matchTimeUp()) break;

            // 2. Intake on, push forward, run until stall
            collectDrive(COLLECT_PUSH_FEET * 12.0);
            stopIntake();
            if (matchTimeUp()) break;

            // 3. Drive back to start; flywheels spin up at halfway
            driveBackToStartWithFlywheelAtHalfway(startX);
            if (matchTimeUp()) break;

            // 4. Launch sequence (mirrors ApolloDrive A-press behavior)
            launchSequence();
        }

        // ====================================================
        // 28s CUTOFF
        // ====================================================
        stopIntake();
        stopLauncherImmediate();
        blockerBlocked();

        driveToCornerForward(startX);

        stopIntake();
        stopLauncherImmediate();

        telemetry.addLine("Auto complete.");
        telemetry.update();
    }

    // ============================================================
    // launchSequence — mirrors ApolloDrive's A-press behavior:
    //   1. Spin flywheels up for BLOCKER_FLYWHEEL_SPINUP_SECONDS
    //   2. Open blocker
    //   3. After BLOCKER_LAUNCH_INTAKE_DELAY, run intake at full power
    //   4. After BLOCKER_AUTO_RETURN_SECONDS, close blocker + cut all
    // Bails immediately on matchTimeUp().
    // ============================================================
    private void launchSequence() {
        if (matchTimeUp()) return;

        // 1. Spin up
        ElapsedTime spinup = new ElapsedTime();
        while (opModeIsActive() && !matchTimeUp()
                && spinup.seconds() < BLOCKER_FLYWHEEL_SPINUP_SECONDS) {
            startLauncher();
            telemetry.addData("Phase", "LAUNCH spin-up");
            telemetry.addData("Match (s)", "%.1f", matchTimer.seconds());
            telemetry.update();
        }
        if (matchTimeUp()) {
            stopLauncherImmediate();
            return;
        }

        // 2. Open blocker
        blockerLaunch();
        ElapsedTime dwell = new ElapsedTime();
        boolean intakeFired = false;

        // 3. Dwell; delayed intake start
        while (opModeIsActive() && !matchTimeUp()
                && dwell.seconds() < BLOCKER_AUTO_RETURN_SECONDS) {
            startLauncher();
            if (!intakeFired && dwell.seconds() >= BLOCKER_LAUNCH_INTAKE_DELAY) {
                setIntakePower(INTAKE_FORWARD_POWER);
                intakeFired = true;
            }
            telemetry.addData("Phase", "LAUNCH dwell (blocker open)");
            telemetry.addData("Intake", intakeFired ? "ON" : "waiting");
            telemetry.addData("Match (s)", "%.1f", matchTimer.seconds());
            telemetry.update();
        }

        // 4. Close blocker, cut everything
        blockerBlocked();
        stopIntake();
        stopLauncherImmediate();
    }

    // ============================================================
    // collectDrive
    // ============================================================
    private void collectDrive(double inches) {
        odo.update();
        double startPos  = readPosXInches();
        double targetPos = startPos + inches;

        alignWheelsTo(WHEELS_FORWARD_RAD, "ALIGNING FORWARD (collect)");

        setIntakePower(INTAKE_FORWARD_POWER);

        ElapsedTime stallTimer    = new ElapsedTime();
        boolean stallTiming       = false;
        ElapsedTime graceTimer    = new ElapsedTime();
        boolean distanceReached   = false;

        long startTime = System.currentTimeMillis();
        long lastTime  = startTime;
        double prevError = inches;
        double integral  = 0.0;

        while (opModeIsActive() && !matchTimeUp()) {
            odo.update();

            long now = System.currentTimeMillis();
            double dt = (now - lastTime) / 1000.0;
            if (dt <= 0) dt = 0.001;
            lastTime = now;

            double currentPos = readPosXInches();
            double error = targetPos - currentPos;

            double topC = topIntake.getCurrent(CurrentUnit.AMPS);
            double botC = bottomIntake.getCurrent(CurrentUnit.AMPS);
            double maxC = Math.max(topC, botC);
            if (maxC > INTAKE_STALL_CURRENT_AMPS) {
                if (!stallTiming) {
                    stallTiming = true;
                    stallTimer.reset();
                } else if (stallTimer.seconds() >= INTAKE_STALL_TIME_SECONDS) {
                    break;
                }
            } else {
                stallTiming = false;
            }

            if (!distanceReached && Math.abs(error) < DRIVE_TOLERANCE_IN) {
                distanceReached = true;
                graceTimer.reset();
            }
            if (distanceReached
                    && graceTimer.seconds() >= COLLECT_POST_DISTANCE_GRACE_SECONDS) {
                break;
            }

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
            double basePower = pTerm + DRIVE_KI * integral + DRIVE_KD * derivative
                    + DRIVE_KS * Math.signum(error);
            basePower = clamp(basePower, -DRIVE_MAX_POWER, DRIVE_MAX_POWER);
            if (Math.abs(basePower) < DRIVE_MIN_POWER && Math.abs(error) > DRIVE_TOLERANCE_IN) {
                basePower = DRIVE_MIN_POWER * Math.signum(error);
            }
            if (distanceReached) basePower = 0.0;

            double currentHeading = readHeadingDegrees();
            double headingError   = wrapAngleDegrees(autoStartHeadingDeg - currentHeading);
            double correction     = clamp(HEADING_KP * headingError,
                    -HEADING_MAX_CORRECTION, HEADING_MAX_CORRECTION);

            double powerLeft  = clamp(basePower - correction, -1.0, 1.0);
            double powerRight = clamp(basePower + correction, -1.0, 1.0);
            setDrivePowersLeftRight(powerLeft, powerRight, WHEELS_FORWARD_RAD);

            telemetry.addData("Phase", "COLLECT DRIVE");
            telemetry.addData("Error (in)", "%.2f", error);
            telemetry.addData("Intake A", "top %.2f  bot %.2f", topC, botC);
            telemetry.addData("Stall watch", stallTiming
                    ? String.format("%.2fs / %.1fs", stallTimer.seconds(), INTAKE_STALL_TIME_SECONDS)
                    : "ok");
            telemetry.addData("Distance reached", distanceReached);
            telemetry.addData("Match (s)", "%.1f", matchTimer.seconds());
            telemetry.update();

            prevError = error;
        }

        setDrivePowersLeftRight(0, 0, WHEELS_FORWARD_RAD);
    }

    // ============================================================
    // driveBackToStartWithFlywheelAtHalfway
    // ============================================================
    private void driveBackToStartWithFlywheelAtHalfway(double startX) {
        odo.update();
        double currentX  = readPosXInches();
        double targetPos = startX;
        double halfwayX  = (currentX + targetPos) / 2.0;

        alignWheelsTo(WHEELS_FORWARD_RAD, "ALIGNING FORWARD (return)");

        boolean flywheelStarted = false;

        long startTime = System.currentTimeMillis();
        long lastTime  = startTime;
        double prevError = targetPos - currentX;
        double integral  = 0.0;
        int settleCounter = 0;
        double initialErrorSign = Math.signum(prevError);
        boolean hasOvershot = false;

        while (opModeIsActive() && !matchTimeUp()) {
            odo.update();

            long now = System.currentTimeMillis();
            double dt = (now - lastTime) / 1000.0;
            if (dt <= 0) dt = 0.001;
            lastTime = now;

            double pos   = readPosXInches();
            double error = targetPos - pos;

            if (!flywheelStarted) {
                boolean pastHalfway = (currentX > targetPos)
                        ? (pos <= halfwayX)
                        : (pos >= halfwayX);
                if (pastHalfway) {
                    startLauncher();
                    flywheelStarted = true;
                }
            }

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
            double basePower = pTerm + DRIVE_KI * integral + DRIVE_KD * derivative
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
            double headingError   = wrapAngleDegrees(autoStartHeadingDeg - currentHeading);
            double correction     = clamp(HEADING_KP * headingError,
                    -HEADING_MAX_CORRECTION, HEADING_MAX_CORRECTION);

            double powerLeft  = clamp(basePower - correction, -1.0, 1.0);
            double powerRight = clamp(basePower + correction, -1.0, 1.0);
            setDrivePowersLeftRight(powerLeft, powerRight, WHEELS_FORWARD_RAD);

            telemetry.addData("Phase", "RETURN TO START");
            telemetry.addData("Error (in)", "%.2f", error);
            telemetry.addData("Flywheel", flywheelStarted ? "SPINNING UP" : "off (pre-halfway)");
            telemetry.addData("Match (s)", "%.1f", matchTimer.seconds());
            telemetry.update();

            prevError = error;
        }

        setDrivePowersLeftRight(0, 0, WHEELS_FORWARD_RAD);
    }

    // ============================================================
    // driveToCornerForward
    // ============================================================
    private void driveToCornerForward(double startX) {
        odo.update();
        double targetPos = startX + CORNER_FORWARD_FEET * 12.0;

        alignWheelsTo(WHEELS_FORWARD_RAD, "ALIGNING FORWARD (corner)");

        long startTime = System.currentTimeMillis();
        long lastTime  = startTime;
        double prevError = targetPos - readPosXInches();
        double integral  = 0.0;
        int settleCounter = 0;
        double initialErrorSign = Math.signum(prevError);
        boolean hasOvershot = false;

        while (opModeIsActive()) {
            odo.update();

            long now = System.currentTimeMillis();
            double dt = (now - lastTime) / 1000.0;
            if (dt <= 0) dt = 0.001;
            lastTime = now;

            double pos   = readPosXInches();
            double error = targetPos - pos;

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
            double basePower = pTerm + DRIVE_KI * integral + DRIVE_KD * derivative
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
            double headingError   = wrapAngleDegrees(autoStartHeadingDeg - currentHeading);
            double correction     = clamp(HEADING_KP * headingError,
                    -HEADING_MAX_CORRECTION, HEADING_MAX_CORRECTION);

            double powerLeft  = clamp(basePower - correction, -1.0, 1.0);
            double powerRight = clamp(basePower + correction, -1.0, 1.0);
            setDrivePowersLeftRight(powerLeft, powerRight, WHEELS_FORWARD_RAD);

            telemetry.addData("Phase", "DRIVE TO CORNER");
            telemetry.addData("Error (in)", "%.2f", error);
            telemetry.addData("Match (s)", "%.1f", matchTimer.seconds());
            telemetry.update();

            prevError = error;
        }

        setDrivePowersLeftRight(0, 0, WHEELS_FORWARD_RAD);
    }

    // ============================================================
    // stopLauncherImmediate
    // ============================================================
    private void stopLauncherImmediate() {
        leftFly.setPower(0.0);
        rightFly.setPower(0.0);
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

        topIntake    = hardwareMap.get(DcMotorEx.class, "topIntake");
        bottomIntake = hardwareMap.get(DcMotorEx.class, "bottomIntake");
        leftFly      = hardwareMap.get(DcMotor.class,   "leftFly");
        rightFly     = hardwareMap.get(DcMotor.class,   "rightFly");

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
    // MECHANISM METHODS
    // ============================================================

    public void intake(double seconds) {
        setIntakePower(INTAKE_FORWARD_POWER);

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
                    break;
                }
            } else {
                stallTiming = false;
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

    public void intakeReverse(double seconds) {
        setIntakePower(-INTAKE_REVERSE_HOLD_POWER);

        ElapsedTime runTimer = new ElapsedTime();
        while (opModeIsActive() && runTimer.seconds() < seconds) {
            telemetry.addData("Phase", "INTAKE REVERSE");
            telemetry.addData("Elapsed (s)", "%.2f / %.2f", runTimer.seconds(), seconds);
            telemetry.update();
        }

        stopIntake();
    }

    public void stopIntake() {
        setIntakePower(0.0);
    }

    private void setIntakePower(double forwardPower) {
        topIntake.setPower(-forwardPower);
        bottomIntake.setPower(-forwardPower);
    }

    public void blockerLaunch() {
        blocker.setPosition(BLOCKER_LAUNCH_POSITION);
    }

    public void blockerBlocked() {
        blocker.setPosition(BLOCKER_BLOCKED_POSITION);
    }

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

    public void startLauncher() {
        double power = launcherCompensatedPower();
        leftFly.setPower(power);
        rightFly.setPower(power);
    }

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

    private double launcherCompensatedPower() {
        double voltage = voltageSensor.getVoltage();
        double vf = (voltage > 0) ? Math.min(12.0 / voltage, 1.0) : 1.0;
        return LAUNCHER_POWER * vf;
    }

    // ============================================================
    // DRIVE METHODS
    // ============================================================

    public void driveX(double inches) {
        odo.update();
        double startPos  = readPosXInches();
        double targetPos = startPos + inches;

        alignWheelsTo(WHEELS_FORWARD_RAD, "ALIGNING FORWARD");

        runDistancePID(inches, targetPos, true, WHEELS_FORWARD_RAD, true);
    }

    public void driveY(double inches) {
        odo.update();
        double startPos  = readPosYInches();
        double targetPos = startPos + inches;

        alignWheelsTo(WHEELS_SIDEWAYS_RAD, "ALIGNING SIDEWAYS");

        runDistancePID(inches, targetPos, false, WHEELS_SIDEWAYS_RAD, false);
    }

    public void driveXY(double dxInches, double dyInches) {
        odo.update();
        double startX  = readPosXInches();
        double startY  = readPosYInches();
        double targetX = startX + dxInches;
        double targetY = startY + dyInches;

        runDiagonalPID(dxInches, dyInches, startX, startY, targetX, targetY);
    }

    public void driveToXY(double targetX, double targetY) {
        odo.update();
        double startX = readPosXInches();
        double startY = readPosYInches();
        double dx = targetX - startX;
        double dy = targetY - startY;

        runDiagonalPID(dx, dy, startX, startY, targetX, targetY);
    }

    private void runDiagonalPID(double dx, double dy,
                                double startX, double startY,
                                double targetX, double targetY) {
        double totalDistance = Math.hypot(dx, dy);
        if (totalDistance < DRIVE_TOLERANCE_IN) return;

        double travelAngle = Math.atan2(dy, dx);
        double cosA = Math.cos(travelAngle);
        double sinA = Math.sin(travelAngle);

        alignWheelsTo(travelAngle,
                String.format("ALIGNING %.1f deg", Math.toDegrees(travelAngle)));

        double prevError = totalDistance;
        double integral  = 0.0;
        long startTime   = System.currentTimeMillis();
        long lastTime    = startTime;
        int settleCounter = 0;
        boolean hasOvershot = false;

        while (opModeIsActive()) {
            odo.update();

            long now = System.currentTimeMillis();
            double dt = (now - lastTime) / 1000.0;
            if (dt <= 0) dt = 0.001;
            lastTime = now;

            double currentX = readPosXInches();
            double currentY = readPosYInches();
            double traveled = (currentX - startX) * cosA + (currentY - startY) * sinA;
            double error    = totalDistance - traveled;

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

            double basePower = pTerm + DRIVE_KI * integral + DRIVE_KD * derivative
                    + DRIVE_KS * Math.signum(error);
            basePower = clamp(basePower, -DRIVE_MAX_POWER, DRIVE_MAX_POWER);
            if (Math.abs(basePower) < DRIVE_MIN_POWER && Math.abs(error) > DRIVE_TOLERANCE_IN) {
                basePower = DRIVE_MIN_POWER * Math.signum(error);
            }

            if (error < -DRIVE_TOLERANCE_IN) hasOvershot = true;
            if (hasOvershot && Math.abs(error) > DRIVE_TOLERANCE_IN) {
                basePower = DRIVE_OVERSHOOT_BRAKE_POWER * Math.signum(error);
            }

            setDrivePowersAll(basePower, travelAngle);

            double currentHeading = readHeadingDegrees();
            double headingError   = wrapAngleDegrees(autoStartHeadingDeg - currentHeading);

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

        long stopUntil = System.currentTimeMillis() + 100;
        while (opModeIsActive() && System.currentTimeMillis() < stopUntil) {
            odo.update();
            setDrivePowersAll(0, travelAngle);
        }
        setDrivePowersAll(0, travelAngle);
    }

    private void runDistancePID(double inches, double targetPos, boolean readX,
                                double wheelAngle, boolean useLeftRight) {
        double prevError = inches;
        double integral  = 0.0;
        long startTime   = System.currentTimeMillis();
        long lastTime    = startTime;
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

            double basePower = pTerm + DRIVE_KI * integral + DRIVE_KD * derivative
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
            double headingError   = wrapAngleDegrees(autoStartHeadingDeg - currentHeading);
            double correction     = clamp(HEADING_KP * headingError,
                    -HEADING_MAX_CORRECTION, HEADING_MAX_CORRECTION);

            double powerA = clamp(basePower - correction, -1.0, 1.0);
            double powerB = clamp(basePower + correction, -1.0, 1.0);

            if (useLeftRight) {
                setDrivePowersLeftRight(powerA, powerB, wheelAngle);
            } else {
                setDrivePowersFrontBack(powerA, powerB, wheelAngle);
            }

            telemetry.addData("Phase",   readX ? "DRIVING X" : "DRIVING Y");
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
            if (useLeftRight) setDrivePowersLeftRight(0, 0, wheelAngle);
            else              setDrivePowersFrontBack(0, 0, wheelAngle);
        }
        if (useLeftRight) setDrivePowersLeftRight(0, 0, wheelAngle);
        else              setDrivePowersFrontBack(0, 0, wheelAngle);
    }

    // ============================================================
    // correctHeading
    // ============================================================
    public void correctHeading() {
        long startTime = System.currentTimeMillis();
        long lastTime  = startTime;
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

            double rotPower = pTerm + HEADING_CORRECT_KD * derivative
                    + HEADING_CORRECT_KS * Math.signum(error);
            rotPower = clamp(rotPower, -HEADING_CORRECT_MAX_POWER, HEADING_CORRECT_MAX_POWER);
            if (Math.abs(rotPower) < HEADING_CORRECT_MIN_POWER
                    && Math.abs(error) > HEADING_CORRECT_TOLERANCE_DEG) {
                rotPower = HEADING_CORRECT_MIN_POWER * Math.signum(error);
            }

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
    // alignWheelsTo
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

    private void setDrivePowersAll(double power, double wheelAngle) {
        runModule(frontLeftDrive,  frontLeftSteer,  frontLeftEncoder,  FRONT_LEFT_OFFSET,  power, wheelAngle);
        runModule(frontRightDrive, frontRightSteer, frontRightEncoder, FRONT_RIGHT_OFFSET, power, wheelAngle);
        runModule(backLeftDrive,   backLeftSteer,   backLeftEncoder,   BACK_LEFT_OFFSET,   power, wheelAngle);
        runModule(backRightDrive,  backRightSteer,  backRightEncoder,  BACK_RIGHT_OFFSET,  power, wheelAngle);
    }

    // ============================================================
    // Swerve module control
    // ============================================================
    private void runModule(DcMotor driveMotor, CRServo steerServo, AnalogInput encoder,
                           double encoderOffset, double speed, double targetAngle) {
        double rawAngle     = getRawAngle(encoder);
        double currentAngle = rawAngle - encoderOffset;
        currentAngle = wrapAngle(currentAngle);

        double delta = wrapAngle(targetAngle - currentAngle);

        if (Math.abs(delta) > Math.PI / 2) {
            delta = wrapAngle(delta + Math.PI);
            speed *= -1;
        }

        double servoPower = STEER_KP * delta * -1;
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
    // Utility helpers
    // ============================================================
    private double wrapAngle(double angle) {
        while (angle >  Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private double wrapAngleDegrees(double angle) {
        while (angle >  180) angle -= 360;
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
        for (DcMotor m : motors) m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}