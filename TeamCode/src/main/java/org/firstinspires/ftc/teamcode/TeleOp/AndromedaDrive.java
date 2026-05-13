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

    // --- PTO SERVO ---
    private Servo pto;
    private boolean ptoEngaged = false;
    private boolean ptoDpadUpPreviouslyPressed   = false;
    private boolean ptoDpadDownPreviouslyPressed = false;

    // --- TURRET SERVOS ---
    private Servo leftTurret, rightTurret;
    private double turretPosition = 0.5;
    final double TURRET_SPEED    = 0.008; // position units per loop tick
    final double TURRET_DEADBAND = 0.05;

    // ============================================================
    //   SERVO POSITIONS  <-- SET YOUR VALUES HERE
    // ============================================================
    final double BLOCKER_BLOCKED_POSITION = 0.15;
    final double BLOCKER_LAUNCH_POSITION  = 0.45;

    final double LIFT_START_POSITION   = 0.0;
    final double LIFT_ENGAGED_POSITION = 1.0;

    final double PTO_DISENGAGED = 0.7;
    final double PTO_ENGAGED    = 0.5;
    // ============================================================

    // --- 2. ROBOT GEOMETRY ---
    final double TRACK_WIDTH = 17.258;
    final double WHEELBASE   = 13.544;
    final double R = Math.hypot(TRACK_WIDTH, WHEELBASE);

    final double FRONT_LEFT_OFFSET  = 0.1200;
    final double FRONT_RIGHT_OFFSET = 1.3861;
    final double BACK_LEFT_OFFSET   = 1.6965;
    final double BACK_RIGHT_OFFSET  = 0.8225;

    // --- 4. TUNING PARAMETERS ---
    final double STEER_KP        = 0.6;
    final double DRIVE_DEADBAND  = 0.05;
    final double STEER_DEADBAND  = 0.05;
    final double STEER_MIN_POWER = 0.08;

    // --- 5. SPEED CONTROL CONSTANTS ---
    final double MAX_SPEED_GLOBAL    = 0.8;
    final double MAX_SPEED_SLOW_MODE = 0.2;

    // --- 6. FLYWHEEL / INTAKE CONSTANTS ---
    final double MOTOR_COAST_RAMP_SECONDS = 0.5;

    // --- 6b. INTAKE SPEED PRESETS ---
    final double INTAKE_SPEED_HIGH = 0.90; // dpad_up
    final double INTAKE_SPEED_LOW  = 0.60; // dpad_down
    private double intakeSpeedTarget = INTAKE_SPEED_HIGH; // default

    // --- 7. TOGGLES / STATES ---
    private boolean rightStickButtonPreviouslyPressed = false;

    // Intake toggle
    private boolean intakeRunning = false;
    private boolean rightTriggerPreviouslyPressed = false;

    // Flywheel toggle
    private boolean flywheelRunning = false;
    private boolean leftTriggerPreviouslyPressed = false;

    // Blocker — A = launch, B = blocked
    private boolean aButtonPreviouslyPressed = false;
    private boolean bButtonPreviouslyPressed = false;

    // Ramp-down state for graceful coast-to-stop
    private boolean intakeRampingDown   = false;
    private boolean flywheelRampingDown = false;
    private ElapsedTime intakeRampTimer   = new ElapsedTime();
    private ElapsedTime flywheelRampTimer = new ElapsedTime();
    private double intakeRampStartPower   = 0.0;
    private double flywheelRampStartPower = 0.0;

    // Wheel 'planting'
    final int FRAMES_TO_PLANT_WHEELS = 5;
    private int framesSinceLastMoved = 0;

    // Heading
    private double headingOffset = 0.0;
    private boolean dpadUpPrev   = false;
    private boolean dpadDownPrev = false;

    @Override
    public void runOpMode() {
        initializeHardware();

        blocker.setPosition(BLOCKER_BLOCKED_POSITION);
        lift.setPosition(LIFT_START_POSITION);
        pto.setPosition(PTO_DISENGAGED);
        leftTurret.setPosition(turretPosition);
        rightTurret.setPosition(turretPosition);

        telemetry.addLine("AndromedaDrive ready.");
        telemetry.addLine("G1 Right Trigger:  toggle intakes");
        telemetry.addLine("G1 Left Trigger:   toggle flywheels");
        telemetry.addLine("G1 A:              blocker -> launch position");
        telemetry.addLine("G1 B:              blocker -> blocked position");
        telemetry.addLine("G1 X + Y:          toggle lift (stationary only)");
        telemetry.addLine("G1 dpad_up:        intake 90% / reset heading");
        telemetry.addLine("G1 dpad_down:      intake 60%");
        telemetry.addLine("G1 RB + dpad_up:   PTO engage");
        telemetry.addLine("G1 RB + dpad_down: PTO disengage");
        telemetry.addLine("  PTO active: hold BOTH triggers = intakes backward");
        telemetry.addLine("G1 RB:             slow mode (when PTO inactive)");
        telemetry.addLine("G1 L3:             lock wheels (X)");
        telemetry.addLine("G2 Left Stick X:   turret");
        telemetry.update();

        waitForStart();

        double targetAngleFL = 0, targetAngleFR = 0, targetAngleBL = 0, targetAngleBR = 0;

        while (opModeIsActive()) {

            boolean rbHeld = gamepad1.right_bumper;

            // ============================================================
            //   PTO ENGAGE   — RB + Dpad Up   (rising edge)
            //   PTO DISENGAGE — RB + Dpad Down (rising edge)
            // ============================================================
            boolean ptoDpadUpNow   = rbHeld && gamepad1.dpad_up;
            boolean ptoDpadDownNow = rbHeld && gamepad1.dpad_down;

            if (ptoDpadUpNow && !ptoDpadUpPreviouslyPressed) {
                ptoEngaged = true;
                pto.setPosition(PTO_ENGAGED);
                // Stop intake toggle state so normal mode is clean on disengage
                intakeRunning     = false;
                intakeRampingDown = false;
            }
            if (ptoDpadDownNow && !ptoDpadDownPreviouslyPressed) {
                ptoEngaged = false;
                pto.setPosition(PTO_DISENGAGED);
            }

            ptoDpadUpPreviouslyPressed   = ptoDpadUpNow;
            ptoDpadDownPreviouslyPressed = ptoDpadDownNow;

            // ============================================================
            //   INTAKE SPEED PRESETS  (dpad_up = 90%, dpad_down = 60%)
            //   dpad_up also resets field heading — only when RB is NOT held
            // ============================================================
            boolean dpadUpNow   = gamepad1.dpad_up;
            boolean dpadDownNow = gamepad1.dpad_down;

            if (dpadUpNow && !dpadUpPrev && !rbHeld) {
                intakeSpeedTarget = INTAKE_SPEED_HIGH;
                imu.resetYaw();
                headingOffset = 0.0;
            }
            if (dpadDownNow && !dpadDownPrev && !rbHeld) {
                intakeSpeedTarget = INTAKE_SPEED_LOW;
            }
            dpadUpPrev   = dpadUpNow;
            dpadDownPrev = dpadDownNow;

            // ============================================================
            //   INTAKE MOTOR CONTROL
            //
            //   PTO ACTIVE:
            //     Both triggers held simultaneously → intakes run backward
            //     Anything else → intakes stop
            //     All other mechanisms are frozen (no trigger/button changes
            //     processed for flywheel, blocker, lift, turret)
            //
            //   PTO INACTIVE:
            //     Normal right-trigger toggle behavior
            // ============================================================
            double intakePower = 0.0;

            if (ptoEngaged) {
                boolean bothTriggersHeld = (gamepad1.left_trigger > 0.5) && (gamepad1.right_trigger > 0.5);
                intakePower = bothTriggersHeld ? -1.0 : 0.0;

                topIntake.setPower(intakePower);
                bottomIntake.setPower(intakePower);

                // Keep previous-press states in sync so normal mode resumes cleanly
                rightTriggerPreviouslyPressed = gamepad1.right_trigger > 0.5;
                leftTriggerPreviouslyPressed  = gamepad1.left_trigger  > 0.5;

                // Freeze flywheel (keep power at whatever it was — just don't update)
                // Update speed multiplier and drive as normal below

            } else {
                // --------------------------------------------------------
                //   NORMAL INTAKE TOGGLE  (Right Trigger)
                // --------------------------------------------------------
                boolean rightTriggerCurrentlyPressed = gamepad1.right_trigger > 0.5;
                if (rightTriggerCurrentlyPressed && !rightTriggerPreviouslyPressed) {
                    if (intakeRunning) {
                        intakeRampingDown    = true;
                        intakeRampStartPower = intakeSpeedTarget;
                        intakeRampTimer.reset();
                        intakeRunning = false;
                    } else {
                        intakeRampingDown = false;
                        intakeRunning     = true;
                    }
                }
                rightTriggerPreviouslyPressed = rightTriggerCurrentlyPressed;

                if (intakeRunning) {
                    intakePower = intakeSpeedTarget;
                } else if (intakeRampingDown) {
                    double fraction = intakeRampTimer.seconds() / MOTOR_COAST_RAMP_SECONDS;
                    if (fraction >= 1.0) {
                        intakeRampingDown = false;
                    } else {
                        intakePower = intakeRampStartPower * (1.0 - fraction);
                    }
                }

                topIntake.setPower(-intakePower);
                bottomIntake.setPower(-intakePower);

                // --------------------------------------------------------
                //   FLYWHEEL TOGGLE  (Left Trigger)
                // --------------------------------------------------------
                boolean leftTriggerCurrentlyPressed = gamepad1.left_trigger > 0.5;
                if (leftTriggerCurrentlyPressed && !leftTriggerPreviouslyPressed) {
                    if (flywheelRunning) {
                        double voltage = voltageSensor.getVoltage();
                        flywheelRampStartPower = (voltage > 0) ? Math.min(12.0 / voltage, 1.0) : 1.0;
                        flywheelRampingDown    = true;
                        flywheelRampTimer.reset();
                        flywheelRunning = false;
                    } else {
                        flywheelRampingDown = false;
                        flywheelRunning     = true;
                    }
                }
                leftTriggerPreviouslyPressed = leftTriggerCurrentlyPressed;

                // --------------------------------------------------------
                //   BLOCKER  (A = launch, B = blocked)
                // --------------------------------------------------------
                boolean aButtonCurrentlyPressed = gamepad1.a;
                boolean bButtonCurrentlyPressed = gamepad1.b;

                if (aButtonCurrentlyPressed && !aButtonPreviouslyPressed)
                    blocker.setPosition(BLOCKER_LAUNCH_POSITION);
                if (bButtonCurrentlyPressed && !bButtonPreviouslyPressed)
                    blocker.setPosition(BLOCKER_BLOCKED_POSITION);

                aButtonPreviouslyPressed = aButtonCurrentlyPressed;
                bButtonPreviouslyPressed = bButtonCurrentlyPressed;

                // --------------------------------------------------------
                //   LIFT SERVO  (X + Y simultaneously, stationary only)
                // --------------------------------------------------------
                boolean driverActive =
                        Math.abs(gamepad1.left_stick_x)  > DRIVE_DEADBAND ||
                                Math.abs(gamepad1.left_stick_y)  > DRIVE_DEADBAND ||
                                Math.abs(gamepad1.right_stick_x) > DRIVE_DEADBAND;

                boolean xButtonCurrentlyPressed = gamepad1.x;
                boolean yButtonCurrentlyPressed = gamepad1.y;
                boolean xPressed = xButtonCurrentlyPressed && !xButtonPreviouslyPressed;
                boolean yPressed = yButtonCurrentlyPressed && !yButtonPreviouslyPressed;

                if ((xPressed && yButtonCurrentlyPressed) || (yPressed && xButtonCurrentlyPressed)) {
                    if (!driverActive) {
                        liftEngaged = !liftEngaged;
                        lift.setPosition(liftEngaged ? LIFT_ENGAGED_POSITION : LIFT_START_POSITION);
                    } else {
                        gamepad1.rumble(0.5, 0.5, 200);
                    }
                }
                xButtonPreviouslyPressed = xButtonCurrentlyPressed;
                yButtonPreviouslyPressed = yButtonCurrentlyPressed;

                // --------------------------------------------------------
                //   TURRET  (Gamepad 2 left stick X — both servos in sync)
                // --------------------------------------------------------
                double turretStick = gamepad2.left_stick_x;
                if (Math.abs(turretStick) > TURRET_DEADBAND) {
                    turretPosition += turretStick * TURRET_SPEED;
                    turretPosition  = Math.max(0.0, Math.min(1.0, turretPosition));
                    leftTurret.setPosition(turretPosition);
                    rightTurret.setPosition(turretPosition);
                }
            }

            // ============================================================
            //   FLYWHEEL POWER (computed every loop, updated outside PTO block)
            // ============================================================
            double flywheelPower = 0.0;
            if (flywheelRunning) {
                double voltage = voltageSensor.getVoltage();
                flywheelPower = (voltage > 0) ? Math.min(12.0 / voltage, 1.0) : 1.0;
            } else if (flywheelRampingDown) {
                double fraction = flywheelRampTimer.seconds() / MOTOR_COAST_RAMP_SECONDS;
                if (fraction >= 1.0) {
                    flywheelRampingDown = false;
                } else {
                    flywheelPower = flywheelRampStartPower * (1.0 - fraction);
                }
            }
            leftFly.setPower(flywheelPower);
            rightFly.setPower(flywheelPower);

            // ============================================================
            //   CALIBRATION MODE TOGGLE  (R3) — reserved
            // ============================================================
            boolean rightStickButtonCurrentlyPressed = gamepad1.right_stick_button;
            rightStickButtonPreviouslyPressed = rightStickButtonCurrentlyPressed;

            // --- Speed Limiter (slow mode only when PTO not engaged) ---
            double speedMultiplier = MAX_SPEED_GLOBAL;
            if (rbHeld && !ptoEngaged) speedMultiplier = MAX_SPEED_SLOW_MODE;

            // ============================================================
            //   FIELD-CENTRIC SWERVE DRIVE
            // ============================================================
            double fieldY = -gamepad1.left_stick_y * speedMultiplier;
            double fieldX =  gamepad1.left_stick_x * speedMultiplier;
            double rot    =  gamepad1.right_stick_x * speedMultiplier;

            double rawHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rawPitch   = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.RADIANS);
            double rawRoll    = imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.RADIANS);
            double botHeading = wrapAngle(rawHeading - headingOffset);

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
                    Math.max(speedBackLeft,  speedBackRight)
            );
            if (maxSpeed > 1.0) {
                speedFrontLeft  /= maxSpeed;
                speedFrontRight /= maxSpeed;
                speedBackLeft   /= maxSpeed;
                speedBackRight  /= maxSpeed;
            }

            boolean driverActiveForSwerve =
                    Math.abs(gamepad1.left_stick_x)  > DRIVE_DEADBAND ||
                            Math.abs(gamepad1.left_stick_y)  > DRIVE_DEADBAND ||
                            Math.abs(gamepad1.right_stick_x) > DRIVE_DEADBAND;

            if (driverActiveForSwerve) {
                targetAngleFL = Math.atan2(B, D);
                targetAngleFR = Math.atan2(B, C);
                targetAngleBL = Math.atan2(A, D);
                targetAngleBR = Math.atan2(A, C);
                framesSinceLastMoved = 0;
            } else {
                speedFrontLeft = 0; speedFrontRight = 0;
                speedBackLeft  = 0; speedBackRight  = 0;
                framesSinceLastMoved++;
            }

            boolean lockWheels = gamepad1.left_stick_button || framesSinceLastMoved >= FRAMES_TO_PLANT_WHEELS;
            if (lockWheels) {
                targetAngleFL = -Math.PI / 4; targetAngleFR =  Math.PI / 4;
                targetAngleBL =  Math.PI / 4; targetAngleBR = -Math.PI / 4;
                speedFrontLeft = 0; speedFrontRight = 0;
                speedBackLeft  = 0; speedBackRight  = 0;
            }

            double voltage       = voltageSensor.getVoltage();
            double voltageFactor = (voltage > 0) ? Math.min(12.0 / voltage, 1.0) : 1.0;

            ModuleDebug fl = runModule(frontLeftDrive,  frontLeftSteer,  frontLeftEncoder,  FRONT_LEFT_OFFSET,  speedFrontLeft,  targetAngleFL, "FL", voltageFactor);
            ModuleDebug fr = runModule(frontRightDrive, frontRightSteer, frontRightEncoder, FRONT_RIGHT_OFFSET, speedFrontRight, targetAngleFR, "FR", voltageFactor);
            ModuleDebug bl = runModule(backLeftDrive,   backLeftSteer,   backLeftEncoder,   BACK_LEFT_OFFSET,   speedBackLeft,   targetAngleBL, "BL", voltageFactor);
            ModuleDebug br = runModule(backRightDrive,  backRightSteer,  backRightEncoder,  BACK_RIGHT_OFFSET,  speedBackRight,  targetAngleBR, "BR", voltageFactor);

            // ============================================================
            //   TELEMETRY
            // ============================================================
            telemetry.addLine("=== ANDROMEDA MECHANISMS ===");
            telemetry.addData("PTO",          ptoEngaged ? "ENGAGED (0.5)" : "DISENGAGED (0.7)");
            if (ptoEngaged) {
                boolean bothHeld = (gamepad1.left_trigger > 0.5) && (gamepad1.right_trigger > 0.5);
                telemetry.addData("PTO Intake",   bothHeld ? "BACKWARD (both triggers)" : "STOPPED");
            }
            telemetry.addData("Intake",       intakeRunning ? "RUNNING" : (intakeRampingDown ? "RAMPING DOWN" : "OFF"));
            telemetry.addData("IntakeSpeed",  "%.0f%%  (pwr %.2f)", intakeSpeedTarget * 100, intakePower);
            telemetry.addData("Flywheel",     flywheelRunning ? "RUNNING" : (flywheelRampingDown ? "RAMPING DOWN" : "OFF"));
            telemetry.addData("Blocker",      blocker.getPosition() == BLOCKER_LAUNCH_POSITION ? "LAUNCH" : "BLOCKED");
            telemetry.addData("Lift",         liftEngaged ? "ENGAGED" : "START");
            telemetry.addData("TurretPos",    "%.3f", turretPosition);

            telemetry.addLine("=== FIELD CENTRIC INPUTS ===");
            telemetry.addData("fieldX/fieldY/rot",   "%.2f  %.2f  %.2f", fieldX, fieldY, rot);
            telemetry.addData("rawHeading(rad)",     "%.3f", rawHeading);
            telemetry.addData("botHeading(rad)",     "%.3f", botHeading);
            telemetry.addData("rawPitch(rad)",       "%.3f", rawPitch);
            telemetry.addData("rawRoll(rad)",        "%.3f", rawRoll);
            telemetry.addData("robotX/robotY",       "%.2f  %.2f", robotX, robotY);

            telemetry.addLine("=== KINEMATICS ===");
            telemetry.addData("A B C D",             "%.2f  %.2f  %.2f  %.2f", A, B, C, D);
            telemetry.addData("spd FL FR BL BR",     "%.2f  %.2f  %.2f  %.2f", speedFrontLeft, speedFrontRight, speedBackLeft, speedBackRight);
            telemetry.addData("maxSpeed(norm)",      "%.2f", maxSpeed);
            telemetry.addData("driverActive",        driverActiveForSwerve);
            telemetry.addData("framesSinceMoved",    framesSinceLastMoved);
            telemetry.addData("lockWheels",          lockWheels);

            telemetry.addLine("=== MODULES (raw/adj/target/delta/servo/speed) ===");
            telemetry.addData("FL", fl.toShortString());
            telemetry.addData("FR", fr.toShortString());
            telemetry.addData("BL", bl.toShortString());
            telemetry.addData("BR", br.toShortString());

            telemetry.addLine("=== SYSTEM ===");
            telemetry.addData("Voltage", "%.2f V (factor %.3f)", voltage, voltageFactor);
            telemetry.update();
        }
    }

    // ============================================================
    //   HARDWARE INIT
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

        blocker = hardwareMap.get(Servo.class, "blocker");
        lift    = hardwareMap.get(Servo.class, "lift");
        pto     = hardwareMap.get(Servo.class, "pto");

        leftTurret  = hardwareMap.get(Servo.class, "leftTurret");
        rightTurret = hardwareMap.get(Servo.class, "rightTurret");

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

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

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
            double encoderOffset, double speed, double targetAngle, String name,
            double voltageFactor
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

        if (Math.abs(servoPower) > STEER_DEADBAND) {
            if (servoPower > 0 && servoPower < STEER_MIN_POWER)  servoPower =  STEER_MIN_POWER;
            if (servoPower < 0 && servoPower > -STEER_MIN_POWER) servoPower = -STEER_MIN_POWER;
        } else {
            servoPower = 0;
        }

        servoPower = Math.max(-1, Math.min(1, servoPower));

        double normalisedSpeed = speed * voltageFactor;
        normalisedSpeed = Math.max(-1, Math.min(1, normalisedSpeed));

        steerServo.setPower(servoPower);
        driveMotor.setPower(normalisedSpeed);

        dbg.rawAngle     = rawAngle;
        dbg.currentAngle = currentAngle;
        dbg.targetAngle  = targetAngle;
        dbg.delta        = delta;
        dbg.servoPower   = servoPower;
        dbg.drivePower   = normalisedSpeed;
        dbg.flipped      = flipped;
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