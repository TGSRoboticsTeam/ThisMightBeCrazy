package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * PinpointLocalizer
 * ───────────────────────────────────────────────────────────────────────────
 * Thin wrapper around the goBUILDA Pinpoint odometry computer.
 *
 * The Pinpoint SDK exposes a class called GoBildaPinpointDriver.
 * Add the goBUILDA Pinpoint SDK to your project (download from goBUILDA),
 * then update the import below to match your package location.
 *
 * ── Setup checklist ──────────────────────────────────────────────────────────
 *  1. Add the Pinpoint .aar / .java files to your TeamCode module.
 *  2. Wire your 4-bar odometry pod: X pod → xOdometryPodPort,
 *                                   Y pod → yOdometryPodPort  (set below).
 *  3. Confirm the pod directions match physical forward/left.
 *  4. Set POD_RESOLUTION to your odometry pod's ticks-per-mm (goBUILDA 4-bar = 2000 t/rev,
 *     13.26 mm/rev → ~150.896 ticks/mm — the constant below is set for this pod).
 *  5. In the Driver Hub config name your Pinpoint device "pinpoint".
 *
 * ── Coordinate convention ────────────────────────────────────────────────────
 *   X  = forward from robot starting position (inches)
 *   Y  = left from robot starting position (inches)
 *   Heading = CCW positive, 0 = robot's initial facing direction (radians)
 */
public class PinpointLocalizer {

    // ── CONFIGURE THESE FOR YOUR ROBOT ───────────────────────────────────────

    /** Hardware map name of the Pinpoint device (must match Driver Hub config). */
    private static final String DEVICE_NAME = "pinpoint";

    /**
     * Offset of the odometry pod from the robot centre.
     * X pod: distance forward (+) or backward (-) from centre (mm).
     * Y pod: distance left (+) or right (-) from centre (mm).
     * Measure on your robot and update these.
     */
    private static final double X_POD_OFFSET_MM =   0.0;   // forward offset of X pod
    private static final double Y_POD_OFFSET_MM =   0.0;   // lateral offset of Y pod

    /** goBUILDA 4-bar pod resolution (ticks per mm). */
    private static final GoBildaPinpointDriver.GoBildaOdometryPods POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

    /**
     * Directions — flip if Pinpoint reads backwards on your robot.
     * FORWARD = positive when pod moves in the expected positive direction.
     * REVERSED = flip the sign.
     */
    private static final GoBildaPinpointDriver.EncoderDirection X_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private static final GoBildaPinpointDriver.EncoderDirection Y_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    // ── CONSTANTS ─────────────────────────────────────────────────────────────
    private static final double MM_PER_INCH = 25.4;

    // ── HARDWARE ─────────────────────────────────────────────────────────────
    private final GoBildaPinpointDriver pinpoint;

    // ─────────────────────────────────────────────────────────────────────────
    public PinpointLocalizer(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, DEVICE_NAME);

        pinpoint.setOffsets(X_POD_OFFSET_MM, Y_POD_OFFSET_MM);
        pinpoint.setEncoderResolution(POD_TYPE);
        pinpoint.setEncoderDirections(X_DIRECTION, Y_DIRECTION);
        pinpoint.resetPosAndIMU();
    }

    /**
     * Set the robot's known position (e.g. at auto start).
     *
     * @param x       field X in inches
     * @param y       field Y in inches
     * @param heading robot heading in radians
     */
    public void setPosition(double x, double y, double heading) {
        // Pinpoint uses mm internally; heading in radians passed directly.
        pinpoint.setPosition(new Pose2D(
                DistanceUnit.MM, x * MM_PER_INCH, y * MM_PER_INCH,
                AngleUnit.RADIANS, heading));
    }

    /** Poll the Pinpoint for the latest odometry data. Call every loop. */
    public void update() {
        pinpoint.update();
    }

    /** Current X position in inches. */
    public double getX() {
        return pinpoint.getPosX() / MM_PER_INCH;
    }

    /** Current Y position in inches. */
    public double getY() {
        return pinpoint.getPosY() / MM_PER_INCH;
    }

    /** Current heading in radians (-π … π). */
    public double getHeading() {
        return pinpoint.getHeading(AngleUnit.RADIANS);
    }

    /** Velocity in X direction (inches/sec). */
    public double getVelocityX() {
        return pinpoint.getVelX() / MM_PER_INCH;
    }

    /** Velocity in Y direction (inches/sec). */
    public double getVelocityY() {
        return pinpoint.getVelY() / MM_PER_INCH;
    }
}
