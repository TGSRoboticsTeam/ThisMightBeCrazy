package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * PinpointLocalizer
 * ───────────────────────────────────────────────────────────────────────────
 * Wraps the goBUILDA Pinpoint odometry computer (com.qualcomm.hardware.gobilda).
 * No external SDK needed — GoBildaPinpointDriver is part of the FTC SDK.
 *
 * ── Setup checklist ──────────────────────────────────────────────────────────
 *  1. Name your Pinpoint device "pinpoint" in the Driver Hub config.
 *  2. Measure your pod offsets from robot center and set them below.
 *  3. Verify encoder directions by pushing the robot by hand (see tuning guide).
 *
 * ── Coordinate convention ────────────────────────────────────────────────────
 *   X       = forward from robot starting position (inches)
 *   Y       = left from robot starting position (inches)
 *   Heading = CCW positive, 0 = robot's initial facing direction (radians)
 */
public class PinpointLocalizer {

    // ═════════════════════════════════════════════════════════════════════════
    //  CONFIGURE THESE FOR YOUR ROBOT
    // ═════════════════════════════════════════════════════════════════════════

    /** Must match the name in your Driver Hub hardware configuration. */
    private static final String DEVICE_NAME = "odo";

    /**
     * Offset of the odometry pods from your chosen tracking point (usually robot center).
     *
     * X pod offset: how far SIDEWAYS the X (forward) pod is from center.
     *   Left of center  = positive
     *   Right of center = negative
     *
     * Y pod offset: how far FORWARD the Y (strafe) pod is from center.
     *   Forward of center  = positive
     *   Backward of center = negative
     *
     * Measure on your robot in mm and update these.
     * The defaults below are from the goBUILDA sample — replace with your values.
     */
    private static final double X_POD_OFFSET_MM = -84.0;
    private static final double Y_POD_OFFSET_MM = -168.0;

    /**
     * Flip these if the Pinpoint reads backwards during the hand-push test.
     * X pod should increase when robot moves FORWARD.
     * Y pod should increase when robot moves LEFT.
     */
    private static final GoBildaPinpointDriver.EncoderDirection X_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private static final GoBildaPinpointDriver.EncoderDirection Y_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    // ═════════════════════════════════════════════════════════════════════════
    //  HARDWARE
    // ═════════════════════════════════════════════════════════════════════════
    private final GoBildaPinpointDriver pinpoint;

    // ─────────────────────────────────────────────────────────────────────────
    public PinpointLocalizer(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, DEVICE_NAME);

        pinpoint.setOffsets(X_POD_OFFSET_MM, Y_POD_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(X_DIRECTION, Y_DIRECTION);

        // Resets position to (0,0,0) and recalibrates the IMU.
        // Call this while the robot is stationary before auto starts.
        pinpoint.resetPosAndIMU();
    }

    /**
     * Set the robot's known starting position.
     * Call this after init() if your robot doesn't start at field origin.
     *
     * @param x       field X in inches
     * @param y       field Y in inches
     * @param heading robot heading in degrees (converted to radians internally)
     */
    public void setPosition(double x, double y, double headingRadians) {
        pinpoint.setPosition(new Pose2D(
                DistanceUnit.INCH, x, y,
                AngleUnit.RADIANS, headingRadians));
    }

    /** Poll the Pinpoint for the latest odometry data. Call every loop. */
    public void update() {
        pinpoint.update();
    }

    /** Current X position in inches. */
    public double getX() {
        return pinpoint.getPosition().getX(DistanceUnit.INCH);
    }

    /** Current Y position in inches. */
    public double getY() {
        return pinpoint.getPosition().getY(DistanceUnit.INCH);
    }

    /** Current heading in radians (-π … π). */
    public double getHeading() {
        return pinpoint.getPosition().getHeading(AngleUnit.RADIANS);
    }
}
