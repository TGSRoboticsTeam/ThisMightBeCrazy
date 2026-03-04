package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@Disabled
@TeleOp(name = "justFieldSwerving", group = "Swerve")
public class justFieldSwerving extends LinearOpMode {

    // --- 1. HARDWARE DECLARATIONS ---
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private CRServo frontLeftSteer, frontRightSteer, backLeftSteer, backRightSteer;
    private AnalogInput frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder;
    private IMU imu;
    private VoltageSensor voltageSensor;

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
    final double MAX_SPEED_GLOBAL = 0.8;
    final double MAX_SPEED_SLOW_MODE = 0.2;

    // --- 6. TOGGLES / STATES ---
    private boolean isCalibrationModeActive = false;
    private boolean rightStickButtonPreviouslyPressed = false;

    // Wheel 'planting'
    final int FRAMES_TO_PLANT_WHEELS = 5;
    private int framesSinceLastMoved = 0;

    // Heading debug / optional offset
    private double headingOffset = 0.0;
    private boolean dpadUpPrev = false;

    @Override


    public void runOpMode() {
        initializeHardware();

        telemetry.addLine("justSwerve_FieldCentric_Debug ready.");
        telemetry.addLine("G1 dpad_up: reset field heading (imu.resetYaw + offset=0)");
        telemetry.addLine("G1 R3: toggle calibration mode (steer/drive off)");
        telemetry.addLine("G1 RB: slow mode");
        telemetry.addLine("G1 L3: lock wheels (X)");
        telemetry.update();

        waitForStart();

        double targetAngleFL = 0, targetAngleFR = 0, targetAngleBL = 0, targetAngleBR = 0;

        while (opModeIsActive()) {

            // --- Toggle Logic for Calibration Mode (Gamepad 1) ---
            boolean rightStickButtonCurrentlyPressed = gamepad1.right_stick_button;
            if (rightStickButtonCurrentlyPressed && !rightStickButtonPreviouslyPressed) {
                isCalibrationModeActive = !isCalibrationModeActive;
            }
            rightStickButtonPreviouslyPressed = rightStickButtonCurrentlyPressed;

            // --- Speed Limiter (Gamepad 1) ---
            double speedMultiplier = MAX_SPEED_GLOBAL;
            if (gamepad1.right_bumper) speedMultiplier = MAX_SPEED_SLOW_MODE;

            // --- Field-Centric Reset (Gamepad 1 dpad_up) ---
            boolean dpadUpNow = gamepad1.dpad_up;
            if (dpadUpNow && !dpadUpPrev) {
                telemetry.addLine("Resetting field heading");
                telemetry.update();
                imu.resetYaw();
                headingOffset = 0.0;
            }
            dpadUpPrev = dpadUpNow;

            // --- CALIBRATION MODE CHECK ---
            if (isCalibrationModeActive) {
                runCalibrationMode();
                continue;
            }

            // ------ GAMEPAD INPUTS (FIELD-CENTRIC) ------ //
            // Field-frame desired motion from sticks
            double fieldY = -gamepad1.left_stick_y * speedMultiplier; // forward
            double fieldX =  gamepad1.left_stick_x * speedMultiplier; // strafe
            double rot    =  gamepad1.right_stick_x * speedMultiplier; // rotation (robot frame)

            // IMU heading (robot yaw)
            double rawHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rawPitch= imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.RADIANS);
            double rawRoll = imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.RADIANS);

            double botHeading = wrapAngle(rawHeading - headingOffset);

            // Convert field -> robot (rotate by -heading)
            // Matches your mecanum example:
            //   strafe = x*cos(-h) - y*sin(-h)
            //   drive  = x*sin(-h) + y*cos(-h)
            double robotX = fieldX * Math.cos(-botHeading) - fieldY * Math.sin(-botHeading);
            double robotY = fieldX * Math.sin(-botHeading) + fieldY * Math.cos(-botHeading);

            // If your strafe direction ends up mirrored, toggle this sign.
            // In your mecanum code you had: strafe = -strafe;
            // Keep it here as a single switch:
            boolean invertRobotX =false; // <--- MOST COMMON FIX WHEN FIELD STRAFE IS “BACKWARDS”
            if (invertRobotX) robotX = -robotX;

            // ------ SWERVE KINEMATICS (robotX/robotY/rot are robot-centric now) ------ //
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

            // Steering targets only update when moving/rotating
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

            // Lock wheels override ('X' formation) + auto-plant after idle frames
            boolean lockWheels = gamepad1.left_stick_button || framesSinceLastMoved >= FRAMES_TO_PLANT_WHEELS;
            if (lockWheels) {
                targetAngleFL = -Math.PI / 4; targetAngleFR =  Math.PI / 4;
                targetAngleBL =  Math.PI / 4; targetAngleBR = -Math.PI / 4;
                speedFrontLeft = 0; speedFrontRight = 0; speedBackLeft = 0; speedBackRight = 0;
            }

            // Apply swerve module outputs (with per-module debug telemetry)
            ModuleDebug fl = runModule(frontLeftDrive,  frontLeftSteer,  frontLeftEncoder,  FRONT_LEFT_OFFSET,  speedFrontLeft,  targetAngleFL, "FL");
            ModuleDebug fr = runModule(frontRightDrive, frontRightSteer, frontRightEncoder, FRONT_RIGHT_OFFSET, speedFrontRight, targetAngleFR, "FR");
            ModuleDebug bl = runModule(backLeftDrive,   backLeftSteer,   backLeftEncoder,   BACK_LEFT_OFFSET,   speedBackLeft,   targetAngleBL, "BL");
            ModuleDebug br = runModule(backRightDrive,  backRightSteer,  backRightEncoder,  BACK_RIGHT_OFFSET,  speedBackRight,  targetAngleBR, "BR");

            // ---------------- TELEMETRY (LOTS) ----------------
            telemetry.addLine("=== FIELD CENTRIC INPUTS ===");
            telemetry.addData("fieldX/fieldY/rot", "%.2f  %.2f  %.2f", fieldX, fieldY, rot);
            telemetry.addData("rawHeading(rad)", "%.3f", rawHeading);
            telemetry.addData("botHeading(rad)", "%.3f", botHeading);
            telemetry.addData("rawPitch(rad)", "%.3f", rawPitch);
            telemetry.addData("rawRoll(rad)", "%.3f", rawRoll);

            telemetry.addData("invertRobotX", invertRobotX);
            telemetry.addData("robotX/robotY", "%.2f  %.2f", robotX, robotY);

            telemetry.addLine("=== KINEMATICS ===");
            telemetry.addData("A B C D", "%.2f  %.2f  %.2f  %.2f", A, B, C, D);
            telemetry.addData("spd FL FR BL BR", "%.2f  %.2f  %.2f  %.2f", speedFrontLeft, speedFrontRight, speedBackLeft, speedBackRight);
            telemetry.addData("maxSpeed(norm)", "%.2f", maxSpeed);
            telemetry.addData("driverActive", driverActive);
            telemetry.addData("framesSinceMoved", framesSinceLastMoved);
            telemetry.addData("lockWheels", lockWheels);

            telemetry.addLine("=== MODULES (raw/adj/target/delta/servo/speed) ===");
            telemetry.addData("FL", fl.toShortString());
            telemetry.addData("FR", fr.toShortString());
            telemetry.addData("BL", bl.toShortString());
            telemetry.addData("BR", br.toShortString());

            telemetry.addLine("=== SYSTEM ===");
            telemetry.addData("Voltage", "%.2f V", voltageSensor.getVoltage());
            telemetry.addData("Mode", isCalibrationModeActive ? "CALIBRATION" : "DRIVE");
            telemetry.update();
        }
    }

    // ---------------- HELPER METHODS ----------------

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
        imu = hardwareMap.get(IMU.class, "imu");

        // IMPORTANT: Set this to match your control hub orientation on the robot.
        // Your earlier note: "usb pointing back and logo left"
        // That corresponds most commonly to: LogoFacingDirection.LEFT, UsbFacingDirection.BACKWARD
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );
        imu.initialize(parameters);
        imu.resetYaw();


        // Drive motor directions (keep yours)
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetMotors(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);

        // Start “field forward” as current direction
        imu.resetYaw();
        headingOffset = 0.0;
    }

    private static class ModuleDebug {
        String name;
        double rawAngle;
        double currentAngle;
        double targetAngle;
        double delta;
        double servoPower;
        double drivePower;
        boolean flipped;

        String toShortString() {
            // Keep it compact but information-dense
            return String.format(
                    "raw=%.2f adj=%.2f tgt=%.2f d=%.2f sp=%.2f drv=%.2f flip=%s",
                    rawAngle, currentAngle, targetAngle, delta, servoPower, drivePower, flipped
            );
        }
    }

    private ModuleDebug runModule(
            DcMotor driveMotor,
            CRServo steerServo,
            AnalogInput encoder,
            double encoderOffset,
            double speed,
            double targetAngle,
            String name
    ) {
        ModuleDebug dbg = new ModuleDebug();
        dbg.name = name;

        double rawAngle = getRawAngle(encoder);
        double currentAngle = wrapAngle(rawAngle - encoderOffset);

        double delta = wrapAngle(targetAngle - currentAngle);

        boolean flipped = false;
        if (Math.abs(delta) > Math.PI / 2) {
            delta = wrapAngle(delta + Math.PI);
            speed *= -1;
            flipped = true;
        }

        double servoPower = STEER_KP * delta;

        // Your original steering fix (keep)
        servoPower *= -1;

        if (Math.abs(servoPower) < STEER_DEADBAND) servoPower = 0;
        servoPower = Math.max(-1, Math.min(1, servoPower));

        steerServo.setPower(servoPower);
        driveMotor.setPower(speed);

        dbg.rawAngle = rawAngle;
        dbg.currentAngle = currentAngle;
        dbg.targetAngle = targetAngle;
        dbg.delta = delta;
        dbg.servoPower = servoPower;
        dbg.drivePower = speed;
        dbg.flipped = flipped;
        return dbg;
    }

    private double wrapAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private void resetMotors(DcMotor... motors) {
        for (DcMotor m : motors) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private double getRawAngle(AnalogInput encoder) {
        return encoder.getVoltage() / 3.3 * (2 * Math.PI);
    }

    private void runCalibrationMode() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);

        frontLeftSteer.setPower(0);
        frontRightSteer.setPower(0);
        backLeftSteer.setPower(0);
        backRightSteer.setPower(0);

        // Calibration telemetry: show raw + adjusted angles so you can re-measure offsets quickly
        double flRaw = getRawAngle(frontLeftEncoder);
        double frRaw = getRawAngle(frontRightEncoder);
        double blRaw = getRawAngle(backLeftEncoder);
        double brRaw = getRawAngle(backRightEncoder);

        telemetry.addLine("=== CALIBRATION MODE (DRIVE + STEER OFF) ===");
        telemetry.addLine("R3 to exit. Use these to confirm offsets.");
        telemetry.addData("FL raw/adj", "%.3f / %.3f", flRaw, wrapAngle(flRaw - FRONT_LEFT_OFFSET));
        telemetry.addData("FR raw/adj", "%.3f / %.3f", frRaw, wrapAngle(frRaw - FRONT_RIGHT_OFFSET));
        telemetry.addData("BL raw/adj", "%.3f / %.3f", blRaw, wrapAngle(blRaw - BACK_LEFT_OFFSET));
        telemetry.addData("BR raw/adj", "%.3f / %.3f", brRaw, wrapAngle(brRaw - BACK_RIGHT_OFFSET));

        double rawHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        telemetry.addData("IMU yaw(rad)", "%.3f", rawHeading);
        telemetry.addData("Voltage", "%.2f V", voltageSensor.getVoltage());
        telemetry.update();
    }
}
