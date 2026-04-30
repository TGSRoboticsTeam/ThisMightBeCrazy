package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "AndromedaDrive", group = "Swerve")
public class AndromedaDrive extends LinearOpMode {

    // --- 1. HARDWARE DECLARATIONS ---
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private CRServo frontLeftSteer, frontRightSteer, backLeftSteer, backRightSteer;
    private AnalogInput frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder;
    private IMU imu;
    private VoltageSensor voltageSensor;

    // --- MECHANISM MOTORS ---
    private DcMotor topIntake, bottomIntake;
    private DcMotor leftFly, rightFly;

    // --- BLOCKER SERVO ---
    private Servo blocker;

    // --- LIFT SERVO ---
    private Servo lift;
    private boolean liftEngaged = false;
    private boolean xButtonPreviouslyPressed = false;
    private boolean yButtonPreviouslyPressed = false;

    // ============================================================
    //   SERVO POSITIONS  <-- SET YOUR VALUES HERE
    // ============================================================
    final double BLOCKER_BLOCKED_POSITION   = 0.0;  // Blocker: blocking launch
    final double BLOCKER_LAUNCH_POSITION    = 1.0;  // Blocker: allowing launch
    final double BLOCKER_AUTO_CLOSE_SECONDS = 3.0;  // Blocker: auto-return delay (seconds)

    final double LIFT_START_POSITION    = 0.0;  // Lift: retracted / start
    final double LIFT_ENGAGED_POSITION  = 1.0;  // Lift: extended / engaged
    // ============================================================

    // --- 2. ROBOT GEOMETRY ---
    final double TRACK_WIDTH = 17.258;
    final double WHEELBASE   = 13.544;
    final double R = Math.hypot(TRACK_WIDTH, WHEELBASE);

    // --- 3. OFFSETS (Your measured values) ---
    final double FRONT_LEFT_OFFSET  = 1.34;
    final double FRONT_RIGHT_OFFSET = 3.161;
    final double BACK_LEFT_OFFSET   = 1.589;
    final double BACK_RIGHT_OFFSET  = 1.237;

    // --- 4. TUNING PARAMETERS ---
    final double STEER_KP = 0.6;
    final double DRIVE_DEADBAND = 0.05;
    final double STEER_DEADBAND = 0.05;

    // --- 5. SPEED CONTROL CONSTANTS ---
    final double MAX_SPEED_GLOBAL    = 0.8;
    final double MAX_SPEED_SLOW_MODE = 0.2;

    // --- 6. FLYWHEEL / INTAKE CONSTANTS ---
    // How long flywheels must be spinning before launch is allowed (seconds)
    final double FLYWHEEL_SPINUP_TIME_REQUIRED  = 0.7;
    // How long to wait for intakes to spin up before opening blocker (seconds)
    final double INTAKE_LAUNCH_SPINUP_SECONDS   = 0.25;
    // Ramp-down time in seconds for flywheels and intakes when toggled off
    final double MOTOR_COAST_RAMP_SECONDS = 0.5;

    // --- 7. TOGGLES / STATES ---
    private boolean rightStickButtonPreviouslyPressed = false;

    // Intake toggle
    private boolean intakeRunning = false;
    private boolean rightTriggerPreviouslyPressed = false;

    // Flywheel toggle
    private boolean flywheelRunning = false;
    private boolean leftTriggerPreviouslyPressed = false;

    // Flywheel spin-up timer (tracks how long flywheels have been running)
    private ElapsedTime flywheelTimer = new ElapsedTime();
    private boolean flywheelTimerRunning = false;

    // Blocker state
    private boolean aButtonPreviouslyPressed = false;
    private boolean bButtonPreviouslyPressed = false;
    private boolean blockerOpen = false;
    private ElapsedTime blockerLaunchTimer = new ElapsedTime();

    // Pending launch state (A pressed — waiting for intake spinup before opening blocker)
    private boolean launchPending = false;
    private ElapsedTime launchIntakeSpinupTimer = new ElapsedTime();

    // Ramp-down state for graceful coast-to-stop
    private boolean intakeRampingDown  = false;
    private boolean flywheelRampingDown = false;
    private ElapsedTime intakeRampTimer    = new ElapsedTime();
    private ElapsedTime flywheelRampTimer  = new ElapsedTime();
    private double intakeRampStartPower    = 0.0;
    private double flywheelRampStartPower  = 0.0;

    // Wheel 'planting'
    final int FRAMES_TO_PLANT_WHEELS = 5;
    private int framesSinceLastMoved = 0;

    // Heading debug / optional offset
    private double headingOffset = 0.0;
    private boolean dpadUpPrev = false;

    @Override
    public void runOpMode() {
        initializeHardware();

        // Start blocker in blocked position
        blocker.setPosition(BLOCKER_BLOCKED_POSITION);
        // Start lift in retracted position
        lift.setPosition(LIFT_START_POSITION);

        telemetry.addLine("AndromedaDrive ready.");
        telemetry.addLine("G1 Right Trigger: toggle intakes (topIntake + bottomIntake)");
        telemetry.addLine("G1 Left Trigger:  toggle flywheels (leftFly + rightFly)");
        telemetry.addLine("G1 A:             launch (requires flywheels spun >=0.7s)");
        telemetry.addLine("G1 B:             close blocker early");
        telemetry.addLine("G1 X + Y:         toggle lift (stationary only)");
        telemetry.addLine("G1 dpad_up:       reset field heading");
        telemetry.addLine("G1 RB:            slow mode");
        telemetry.addLine("G1 L3:            lock wheels (X)");
        telemetry.update();

        waitForStart();

        flywheelTimer.reset();

        double targetAngleFL = 0, targetAngleFR = 0, targetAngleBL = 0, targetAngleBR = 0;

        while (opModeIsActive()) {

            // ============================================================
            //   INTAKE TOGGLE  (Right Trigger — press = on, press again = off)
            // ============================================================
            boolean rightTriggerCurrentlyPressed = gamepad1.right_trigger > 0.5;
            if (rightTriggerCurrentlyPressed && !rightTriggerPreviouslyPressed) {
                if (intakeRunning) {
                    // Begin ramp-down instead of hard stop
                    intakeRampingDown = true;
                    intakeRampTimer.reset();
                    // Capture the current target power as the ramp start
                    double voltage = voltageSensor.getVoltage();
                    intakeRampStartPower = (voltage > 0) ? Math.min(12.0 / voltage, 1.0) : 1.0;
                    intakeRunning = false;
                } else {
                    intakeRampingDown = false; // cancel any in-progress ramp
                    intakeRunning = true;
                }
            }
            rightTriggerPreviouslyPressed = rightTriggerCurrentlyPressed;

            // Compute intake power
            double intakePower = 0.0;
            if (intakeRunning) {
                double voltage = voltageSensor.getVoltage();
                intakePower = (voltage > 0) ? Math.min(12.0 / voltage, 1.0) : 1.0;
            } else if (intakeRampingDown) {
                double elapsed = intakeRampTimer.seconds();
                double fraction = elapsed / MOTOR_COAST_RAMP_SECONDS;
                if (fraction >= 1.0) {
                    intakeRampingDown = false;
                    intakePower = 0.0;
                } else {
                    intakePower = intakeRampStartPower * (1.0 - fraction);
                }
            }
            topIntake.setPower(intakePower);
            bottomIntake.setPower(intakePower);

            // ============================================================
            //   FLYWHEEL TOGGLE  (Left Trigger — press = on, press again = off)
            // ============================================================
            boolean leftTriggerCurrentlyPressed = gamepad1.left_trigger > 0.5;
            if (leftTriggerCurrentlyPressed && !leftTriggerPreviouslyPressed) {
                if (flywheelRunning) {
                    // Begin ramp-down
                    flywheelRampingDown = true;
                    flywheelRampTimer.reset();
                    double voltage = voltageSensor.getVoltage();
                    flywheelRampStartPower = (voltage > 0) ? Math.min(12.0 / voltage, 1.0) : 1.0;
                    flywheelRunning = false;
                    flywheelTimerRunning = false;
                } else {
                    flywheelRampingDown = false;
                    flywheelRunning = true;
                    flywheelTimer.reset();   // restart spin-up timer
                    flywheelTimerRunning = true;
                }
            }
            leftTriggerPreviouslyPressed = leftTriggerCurrentlyPressed;

            // Compute flywheel power
            double flywheelPower = 0.0;
            if (flywheelRunning) {
                double voltage = voltageSensor.getVoltage();
                flywheelPower = (voltage > 0) ? Math.min(12.0 / voltage, 1.0) : 1.0;
            } else if (flywheelRampingDown) {
                double elapsed = flywheelRampTimer.seconds();
                double fraction = elapsed / MOTOR_COAST_RAMP_SECONDS;
                if (fraction >= 1.0) {
                    flywheelRampingDown = false;
                    flywheelPower = 0.0;
                } else {
                    flywheelPower = flywheelRampStartPower * (1.0 - fraction);
                }
            }
            leftFly.setPower(flywheelPower);
            rightFly.setPower(flywheelPower);

            // ============================================================
            //   BLOCKER / LAUNCH  (A to open, B or timer to close)
            //   Sequence on A press:
            //     1. Ensure intakes are running (force on if not).
            //     2. Wait INTAKE_LAUNCH_SPINUP_SECONDS for them to spin up.
            //     3. If flywheels have also been running >= FLYWHEEL_SPINUP_TIME_REQUIRED,
            //        open the blocker. Otherwise rumble and abort.
            // ============================================================
            boolean aButtonCurrentlyPressed = gamepad1.a;
            boolean bButtonCurrentlyPressed = gamepad1.b;

            // A pressed — kick off launch sequence
            if (aButtonCurrentlyPressed && !aButtonPreviouslyPressed) {
                // Force intakes on if they weren't already
                if (!intakeRunning) {
                    intakeRampingDown = false;
                    intakeRunning = true;
                }
                // Start (or restart) the intake spinup wait
                launchPending = true;
                launchIntakeSpinupTimer.reset();
            }

            // While launch is pending, wait for intake spinup delay to elapse
            if (launchPending) {
                if (launchIntakeSpinupTimer.seconds() >= INTAKE_LAUNCH_SPINUP_SECONDS) {
                    launchPending = false;
                    boolean flywheelsReady =
                            flywheelRunning && flywheelTimer.seconds() >= FLYWHEEL_SPINUP_TIME_REQUIRED;
                    if (flywheelsReady) {
                        blocker.setPosition(BLOCKER_LAUNCH_POSITION);
                        blockerOpen = true;
                        blockerLaunchTimer.reset();
                    } else {
                        // Flywheels not ready — rumble and leave blocker closed
                        gamepad1.rumble(0.5, 0.5, 200);
                    }
                }
            }

            // Close blocker: B pressed OR auto-close timer expired
            if (blockerOpen) {
                boolean bPressed     = bButtonCurrentlyPressed && !bButtonPreviouslyPressed;
                boolean timerExpired = blockerLaunchTimer.seconds() >= BLOCKER_AUTO_CLOSE_SECONDS;
                if (bPressed || timerExpired) {
                    blocker.setPosition(BLOCKER_BLOCKED_POSITION);
                    blockerOpen = false;
                }
            }

            aButtonPreviouslyPressed = aButtonCurrentlyPressed;
            bButtonPreviouslyPressed = bButtonCurrentlyPressed;

            // ============================================================
            //   LIFT SERVO  (X + Y simultaneously, only while stationary)
            //   Toggles between LIFT_START_POSITION and LIFT_ENGAGED_POSITION
            // ============================================================
            boolean xButtonCurrentlyPressed = gamepad1.x;
            boolean yButtonCurrentlyPressed = gamepad1.y;
            boolean xPressed = xButtonCurrentlyPressed && !xButtonPreviouslyPressed;
            boolean yPressed = yButtonCurrentlyPressed && !yButtonPreviouslyPressed;

            // Trigger on either X or Y edge, but only if both are held at that moment
            if ((xPressed && yButtonCurrentlyPressed) || (yPressed && xButtonCurrentlyPressed)) {
                boolean robotStationary = !driverActive;
                if (robotStationary) {
                    liftEngaged = !liftEngaged;
                    lift.setPosition(liftEngaged ? LIFT_ENGAGED_POSITION : LIFT_START_POSITION);
                } else {
                    // Robot is moving — rumble to warn driver
                    gamepad1.rumble(0.5, 0.5, 200);
                }
            }
            xButtonPreviouslyPressed = xButtonCurrentlyPressed;
            yButtonPreviouslyPressed = yButtonCurrentlyPressed;

            // ============================================================
            //   CALIBRATION MODE TOGGLE  (R3)
            // ============================================================
            boolean rightStickButtonCurrentlyPressed = gamepad1.right_stick_button;
            if (rightStickButtonCurrentlyPressed && !rightStickButtonPreviouslyPressed) {
                // reserved — calibration is a standalone OpMode
            }
            rightStickButtonPreviouslyPressed = rightStickButtonCurrentlyPressed;

            // --- Speed Limiter ---
            double speedMultiplier = MAX_SPEED_GLOBAL;
            if (gamepad1.right_bumper) speedMultiplier = MAX_SPEED_SLOW_MODE;

            // --- Field-Centric Reset (dpad_up) ---
            boolean dpadUpNow = gamepad1.dpad_up;
            if (dpadUpNow && !dpadUpPrev) {
                imu.resetYaw();
                headingOffset = 0.0;
            }
            dpadUpPrev = dpadUpNow;

            // ============================================================
            //   FIELD-CENTRIC SWERVE DRIVE
            // ============================================================
            double fieldY = -gamepad1.left_stick_y * speedMultiplier;
            double fieldX =  gamepad1.left_stick_x * speedMultiplier;
            double rot    =  gamepad1.right_stick_x * speedMultiplier;

            double rawHeading  = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rawPitch    = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.RADIANS);
            double rawRoll     = imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.RADIANS);
            double botHeading  = wrapAngle(rawHeading - headingOffset);

            double robotX = fieldX * Math.cos(-botHeading) - fieldY * Math.sin(-botHeading);
            double robotY = fieldX * Math.sin(-botHeading) + fieldY * Math.cos(-botHeading);

            boolean invertRobotX = false;
            if (invertRobotX) robotX = -robotX;

            double A = robotX - rot * (WHEELBASE / R);
            double B = robotX + rot * (WHEELBASE / R);
            double C = robotY - rot * (TRACK_WIDTH / R);
            double D = robotY + rot * (TRACK_WIDTH / R);

            double speedFrontLeft  = Math.hypot(B, D);
            double speedFrontRight = Math.hypot(B, C);
            double speedBackLeft   = Math.hypot(A, D);
            double speedBackRight  = Math.hypot(A, C);

            double maxSpeed = Math.max(
                    Math.max(speedFrontLeft, speedFrontRight),
                    Math.max(speedBackLeft, speedBackRight)
            );
            if (maxSpeed > 1.0) {
                speedFrontLeft  /= maxSpeed;
                speedFrontRight /= maxSpeed;
                speedBackLeft   /= maxSpeed;
                speedBackRight  /= maxSpeed;
            }

            boolean driverActive =
                    Math.abs(fieldX) > DRIVE_DEADBAND ||
                    Math.abs(fieldY) > DRIVE_DEADBAND ||
                    Math.abs(rot)    > DRIVE_DEADBAND;

            if (driverActive) {
                targetAngleFL = Math.atan2(B, D);
                targetAngleFR = Math.atan2(B, C);
                targetAngleBL = Math.atan2(A, D);
                targetAngleBR = Math.atan2(A, C);
                framesSinceLastMoved = 0;
            } else {
                speedFrontLeft = 0; speedFrontRight = 0; speedBackLeft = 0; speedBackRight = 0;
                framesSinceLastMoved += 1;
            }

            boolean lockWheels = gamepad1.left_stick_button || framesSinceLastMoved >= FRAMES_TO_PLANT_WHEELS;
            if (lockWheels) {
                targetAngleFL = -Math.PI / 4; targetAngleFR =  Math.PI / 4;
                targetAngleBL =  Math.PI / 4; targetAngleBR = -Math.PI / 4;
                speedFrontLeft = 0; speedFrontRight = 0; speedBackLeft = 0; speedBackRight = 0;
            }

            ModuleDebug fl = runModule(frontLeftDrive,  frontLeftSteer,  frontLeftEncoder,  FRONT_LEFT_OFFSET,  speedFrontLeft,  targetAngleFL, "FL");
            ModuleDebug fr = runModule(frontRightDrive, frontRightSteer, frontRightEncoder, FRONT_RIGHT_OFFSET, speedFrontRight, targetAngleFR, "FR");
            ModuleDebug bl = runModule(backLeftDrive,   backLeftSteer,   backLeftEncoder,   BACK_LEFT_OFFSET,   speedBackLeft,   targetAngleBL, "BL");
            ModuleDebug br = runModule(backRightDrive,  backRightSteer,  backRightEncoder,  BACK_RIGHT_OFFSET,  speedBackRight,  targetAngleBR, "BR");

            // ============================================================
            //   TELEMETRY
            // ============================================================
            telemetry.addLine("=== ANDROMEDA MECHANISMS ===");
            telemetry.addData("Intake",      intakeRunning ? "RUNNING" : (intakeRampingDown ? "RAMPING DOWN" : "OFF"));
            telemetry.addData("IntakePwr",   "%.2f", intakePower);
            telemetry.addData("Flywheel",    flywheelRunning ? "RUNNING" : (flywheelRampingDown ? "RAMPING DOWN" : "OFF"));
            telemetry.addData("FlywheelPwr", "%.2f", flywheelPower);
            telemetry.addData("FlySpinTime", "%.2f s (need %.1f)", flywheelTimerRunning ? flywheelTimer.seconds() : 0.0, FLYWHEEL_SPINUP_TIME_REQUIRED);
            telemetry.addData("FlyReady",    flywheelRunning && flywheelTimer.seconds() >= FLYWHEEL_SPINUP_TIME_REQUIRED ? "YES" : "NO");
            telemetry.addData("Blocker",     blockerOpen
                    ? String.format("OPEN (auto-close in %.1fs)", BLOCKER_AUTO_CLOSE_SECONDS - blockerLaunchTimer.seconds())
                    : "BLOCKED");
            telemetry.addData("Lift",        liftEngaged ? "ENGAGED" : "START");

            telemetry.addLine("=== FIELD CENTRIC INPUTS ===");
            telemetry.addData("fieldX/fieldY/rot", "%.2f  %.2f  %.2f", fieldX, fieldY, rot);
            telemetry.addData("rawHeading(rad)", "%.3f", rawHeading);
            telemetry.addData("botHeading(rad)", "%.3f", botHeading);
            telemetry.addData("rawPitch(rad)",   "%.3f", rawPitch);
            telemetry.addData("rawRoll(rad)",    "%.3f", rawRoll);
            telemetry.addData("robotX/robotY",   "%.2f  %.2f", robotX, robotY);

            telemetry.addLine("=== KINEMATICS ===");
            telemetry.addData("A B C D",           "%.2f  %.2f  %.2f  %.2f", A, B, C, D);
            telemetry.addData("spd FL FR BL BR",   "%.2f  %.2f  %.2f  %.2f", speedFrontLeft, speedFrontRight, speedBackLeft, speedBackRight);
            telemetry.addData("maxSpeed(norm)",    "%.2f", maxSpeed);
            telemetry.addData("driverActive",      driverActive);
            telemetry.addData("framesSinceMoved",  framesSinceLastMoved);
            telemetry.addData("lockWheels",        lockWheels);

            telemetry.addLine("=== MODULES (raw/adj/target/delta/servo/speed) ===");
            telemetry.addData("FL", fl.toShortString());
            telemetry.addData("FR", fr.toShortString());
            telemetry.addData("BL", bl.toShortString());
            telemetry.addData("BR", br.toShortString());

            telemetry.addLine("=== SYSTEM ===");
            telemetry.addData("Voltage", "%.2f V", voltageSensor.getVoltage());
            telemetry.update();
        }
    }

    // ============================================================
    //   HARDWARE INIT
    // ============================================================
    private void initializeHardware() {
        // Swerve drive motors
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive  = hardwareMap.get(DcMotor.class, "backRightDrive");

        // Swerve steer servos
        frontLeftSteer  = hardwareMap.get(CRServo.class, "frontLeftSteer");
        frontRightSteer = hardwareMap.get(CRServo.class, "frontRightSteer");
        backLeftSteer   = hardwareMap.get(CRServo.class, "backLeftSteer");
        backRightSteer  = hardwareMap.get(CRServo.class, "backRightSteer");

        // Steer encoders
        frontLeftEncoder  = hardwareMap.get(AnalogInput.class, "frontLeftEncoder");
        frontRightEncoder = hardwareMap.get(AnalogInput.class, "frontRightEncoder");
        backLeftEncoder   = hardwareMap.get(AnalogInput.class, "backLeftEncoder");
        backRightEncoder  = hardwareMap.get(AnalogInput.class, "backRightEncoder");

        // Mechanism motors — float on zero power so they coast to a stop
        topIntake    = hardwareMap.get(DcMotor.class, "topIntake");
        bottomIntake = hardwareMap.get(DcMotor.class, "bottomIntake");
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

        // Blocker servo
        blocker = hardwareMap.get(Servo.class, "blocker");

        // Lift servo
        lift = hardwareMap.get(Servo.class, "lift");

        // Sensors
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );
        imu.initialize(parameters);
        imu.resetYaw();

        // Drive motor directions
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetMotors(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);

        headingOffset = 0.0;
    }

    // ============================================================
    //   MODULE HELPER
    // ============================================================
    private static class ModuleDebug {
        String name;
        double rawAngle, currentAngle, targetAngle, delta, servoPower, drivePower;
        boolean flipped;

        String toShortString() {
            return String.format(
                    "raw=%.2f adj=%.2f tgt=%.2f d=%.2f sp=%.2f drv=%.2f flip=%s",
                    rawAngle, currentAngle, targetAngle, delta, servoPower, drivePower, flipped
            );
        }
    }

    private ModuleDebug runModule(
            DcMotor driveMotor, CRServo steerServo, AnalogInput encoder,
            double encoderOffset, double speed, double targetAngle, String name
    ) {
        ModuleDebug dbg = new ModuleDebug();
        dbg.name = name;

        double rawAngle     = getRawAngle(encoder);
        double currentAngle = wrapAngle(rawAngle - encoderOffset);
        double delta        = wrapAngle(targetAngle - currentAngle);

        boolean flipped = false;
        if (Math.abs(delta) > Math.PI / 2) {
            delta = wrapAngle(delta + Math.PI);
            speed *= -1;
            flipped = true;
        }

        double servoPower = STEER_KP * delta * -1;
        if (Math.abs(servoPower) < STEER_DEADBAND) servoPower = 0;
        servoPower = Math.max(-1, Math.min(1, servoPower));

        steerServo.setPower(servoPower);
        driveMotor.setPower(speed);

        dbg.rawAngle    = rawAngle;
        dbg.currentAngle = currentAngle;
        dbg.targetAngle = targetAngle;
        dbg.delta       = delta;
        dbg.servoPower  = servoPower;
        dbg.drivePower  = speed;
        dbg.flipped     = flipped;
        return dbg;
    }

    // ============================================================
    //   UTILITY
    // ============================================================
    private double wrapAngle(double angle) {
        while (angle >  Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private void resetMotors(DcMotor... motors) {
        for (DcMotor m : motors) m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private double getRawAngle(AnalogInput encoder) {
        return encoder.getVoltage() / 3.3 * (2 * Math.PI);
    }
}
