package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Pinpoint Calibration Test OpMode
 *
 * Purpose: verify the Pinpoint odometry pods report accurate distance and heading.
 *
 * How to use:
 *   1. Place the robot on a known starting line, oriented as you would for auto.
 *   2. Press INIT, then START. The robot motors are NOT powered — this is a sensor test.
 *   3. Push the robot forward by hand a known distance (e.g. 12 or 24 inches).
 *      The X reading should match the distance traveled.
 *      Forward should increase X. If it decreases, flip X_POD_DIRECTION in justAuto.
 *   4. Push the robot sideways. The Y reading should change; X should stay roughly constant.
 *   5. Rotate the robot. Heading should change; X and Y should track the pod motion.
 *   6. Press GAMEPAD1 A to reset position and IMU to zero at any time.
 *
 * If hand-pushing 12 inches shows ~12 on telemetry: odometry is well calibrated.
 * If it shows something significantly different: the pod resolution setting may be wrong,
 * or the pod offsets may be wrong. Distance scale issues = wrong pod type.
 * Heading drift while pushing in a straight line = wrong pod offsets.
 */
@Disabled
@TeleOp(name = "pinpointTest", group = "Test")
public class pinpointTest extends LinearOpMode {

    private GoBildaPinpointDriver odo;

    // Must match values used in justAuto so behavior is consistent.
    final double X_POD_OFFSET_MM =  41.9999922;   // X pod, left of center
    final double Y_POD_OFFSET_MM = -148.3535768;  // Y pod, behind center

    final GoBildaPinpointDriver.EncoderDirection X_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    final GoBildaPinpointDriver.EncoderDirection Y_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    @Override
    public void runOpMode() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(X_POD_OFFSET_MM, Y_POD_OFFSET_MM, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(X_POD_DIRECTION, Y_POD_DIRECTION);
        odo.resetPosAndIMU();

        telemetry.addLine("Pinpoint Test initialized.");
        telemetry.addLine("Press START, then push robot by hand.");
        telemetry.addLine("Press A on gamepad1 to reset to zero.");
        telemetry.update();

        waitForStart();

        boolean aPreviouslyPressed = false;

        // Track peak values for easy reference after a push
        double maxX = 0, minX = 0, maxY = 0, minY = 0, maxHeading = 0, minHeading = 0;

        while (opModeIsActive()) {
            odo.update();

            // Reset on A press (rising edge)
            boolean aPressed = gamepad1.a;
            if (aPressed && !aPreviouslyPressed) {
                odo.resetPosAndIMU();
                maxX = 0; minX = 0;
                maxY = 0; minY = 0;
                maxHeading = 0; minHeading = 0;
            }
            aPreviouslyPressed = aPressed;

            double xIn = odo.getPosX(DistanceUnit.INCH);
            double yIn = odo.getPosY(DistanceUnit.INCH);
            double headingDeg = odo.getHeading(AngleUnit.DEGREES);

            // Track range so you can see total travel even if you push past and back
            if (xIn > maxX) maxX = xIn;
            if (xIn < minX) minX = xIn;
            if (yIn > maxY) maxY = yIn;
            if (yIn < minY) minY = yIn;
            if (headingDeg > maxHeading) maxHeading = headingDeg;
            if (headingDeg < minHeading) minHeading = headingDeg;

            telemetry.addLine("=== LIVE READINGS ===");
            telemetry.addData("X (in)", "%.3f", xIn);
            telemetry.addData("Y (in)", "%.3f", yIn);
            telemetry.addData("Heading (deg)", "%.3f", headingDeg);

            telemetry.addLine();
            telemetry.addLine("=== RANGE SINCE RESET ===");
            telemetry.addData("X range (in)", "min %.3f  max %.3f  span %.3f", minX, maxX, maxX - minX);
            telemetry.addData("Y range (in)", "min %.3f  max %.3f  span %.3f", minY, maxY, maxY - minY);
            telemetry.addData("Heading range (deg)", "min %.3f  max %.3f  span %.3f",
                    minHeading, maxHeading, maxHeading - minHeading);

            telemetry.addLine();
            telemetry.addLine("Press A to reset all values to zero.");
            telemetry.addData("Pinpoint status", odo.getDeviceStatus());
            telemetry.update();
        }
    }
}