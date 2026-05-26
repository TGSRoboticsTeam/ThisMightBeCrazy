package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * SwerveCalibration
 *
 * HOW TO USE:
 *   1. Physically rotate each swerve pod wheel so it points straight FORWARD on the robot.
 *   2. Run this OpMode.
 *   3. Press A — the current raw encoder reading for each pod is captured and displayed.
 *   4. Copy those four "OFFSET" values directly into AndromedaDrive as:
 *        FRONT_LEFT_OFFSET, FRONT_RIGHT_OFFSET, BACK_LEFT_OFFSET, BACK_RIGHT_OFFSET
 *
 * WHY THIS WORKS:
 *   The offset for each pod is simply the raw encoder reading (radians, 0–2π)
 *   when that pod is physically pointing forward. In AndromedaDrive the swerve math
 *   subtracts the offset from the raw reading to get a 0-referenced angle, so
 *   "0 = forward" for every pod. No additional math needed — paste and go.
 *
 * CONTROLS:
 *   A              — snapshot: latch and display the current raw angles as offsets
 *   B              — clear snapshot, return to live readout
 *   All motors and servos are held at zero power throughout.
 */
@Disabled
@TeleOp(name = "SwerveCalibration", group = "Swerve")
public class SwerveCalibration extends LinearOpMode {

    // --- Hardware ---
    private DcMotor  frontLeftDrive,  frontRightDrive,  backLeftDrive,  backRightDrive;
    private CRServo  frontLeftSteer,  frontRightSteer,  backLeftSteer,  backRightSteer;
    private AnalogInput frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder;

    // --- Snapshot state ---
    private boolean snapshotTaken = false;
    private double snapFL, snapFR, snapBL, snapBR;

    // --- Button debounce ---
    private boolean aWasPressed = false;
    private boolean bWasPressed = false;

    @Override
    public void runOpMode() {

        // Map hardware
        frontLeftDrive   = hardwareMap.get(DcMotor.class,    "frontLeftDrive");
        frontRightDrive  = hardwareMap.get(DcMotor.class,    "frontRightDrive");
        backLeftDrive    = hardwareMap.get(DcMotor.class,    "backLeftDrive");
        backRightDrive   = hardwareMap.get(DcMotor.class,    "backRightDrive");

        frontLeftSteer   = hardwareMap.get(CRServo.class,    "frontLeftSteer");
        frontRightSteer  = hardwareMap.get(CRServo.class,    "frontRightSteer");
        backLeftSteer    = hardwareMap.get(CRServo.class,    "backLeftSteer");
        backRightSteer   = hardwareMap.get(CRServo.class,    "backRightSteer");

        frontLeftEncoder  = hardwareMap.get(AnalogInput.class, "frontLeftEncoder");
        frontRightEncoder = hardwareMap.get(AnalogInput.class, "frontRightEncoder");
        backLeftEncoder   = hardwareMap.get(AnalogInput.class, "backLeftEncoder");
        backRightEncoder  = hardwareMap.get(AnalogInput.class, "backRightEncoder");

        // All motors/servos off for the entire OpMode
        stopAll();

        telemetry.addLine("=== SWERVE CALIBRATION ===");
        telemetry.addLine("1. Point all pods straight FORWARD on the robot.");
        telemetry.addLine("2. Press A to latch the offsets.");
        telemetry.addLine("3. Copy the OFFSET values into AndromedaDrive.");
        telemetry.addLine("   B = clear snapshot and return to live view.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            stopAll(); // ensure nothing moves

            boolean aNow = gamepad1.a;
            boolean bNow = gamepad1.b;

            // A — take snapshot
            if (aNow && !aWasPressed) {
                snapFL = rawAngle(frontLeftEncoder);
                snapFR = rawAngle(frontRightEncoder);
                snapBL = rawAngle(backLeftEncoder);
                snapBR = rawAngle(backRightEncoder);
                snapshotTaken = true;
            }

            // B — clear snapshot
            if (bNow && !bWasPressed) {
                snapshotTaken = false;
            }

            aWasPressed = aNow;
            bWasPressed = bNow;

            // Always show live readings
            double liveFL = rawAngle(frontLeftEncoder);
            double liveFR = rawAngle(frontRightEncoder);
            double liveBL = rawAngle(backLeftEncoder);
            double liveBR = rawAngle(backRightEncoder);

            telemetry.addLine("=== LIVE (radians, 0 – 2π) ===");
            telemetry.addData("FL live", "%.4f", liveFL);
            telemetry.addData("FR live", "%.4f", liveFR);
            telemetry.addData("BL live", "%.4f", liveBL);
            telemetry.addData("BR live", "%.4f", liveBR);

            telemetry.addLine("");

            if (snapshotTaken) {
                telemetry.addLine("=== SNAPSHOT — copy these into AndromedaDrive ===");
                telemetry.addData("FRONT_LEFT_OFFSET  =", "%.4f", snapFL);
                telemetry.addData("FRONT_RIGHT_OFFSET =", "%.4f", snapFR);
                telemetry.addData("BACK_LEFT_OFFSET   =", "%.4f", snapBL);
                telemetry.addData("BACK_RIGHT_OFFSET  =", "%.4f", snapBR);
                telemetry.addLine("");
                telemetry.addLine("Press B to clear and re-align.");
            } else {
                telemetry.addLine("[ Point pods FORWARD, then press A to snapshot ]");
            }

            telemetry.update();
        }
    }

    /** Zero out all drive motors and steer servos. */
    private void stopAll() {
        frontLeftDrive.setPower(0);   frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);    backRightDrive.setPower(0);
        frontLeftSteer.setPower(0);   frontRightSteer.setPower(0);
        backLeftSteer.setPower(0);    backRightSteer.setPower(0);
    }

    /** Convert analog voltage (0–3.3 V) to radians (0–2π). */
    private double rawAngle(AnalogInput encoder) {
        return encoder.getVoltage() / 3.3 * (2 * Math.PI);
    }
}
