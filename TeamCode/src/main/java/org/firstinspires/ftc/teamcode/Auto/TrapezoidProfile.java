package org.firstinspires.ftc.teamcode.Auto;

/**
 * TrapezoidProfile
 * Generates a smooth trapezoidal velocity profile for a move of a given distance.
 *
 * The profile has three phases:
 *   1. Acceleration  — ramps from 0 up to maxVelocity at maxAcceleration
 *   2. Cruise        — holds maxVelocity (may be skipped for short moves)
 *   3. Deceleration  — ramps back down to minVelocity (prevents stopping dead)
 *
 * Call getVelocity(distanceTravelled) each loop to get the current target speed (0…1).
 */
public class TrapezoidProfile {

    // ── Tuning ────────────────────────────────────────────────────────────────
    /** Fraction of total distance spent accelerating (and decelerating). */
    private static final double RAMP_FRACTION = 0.25;   // 25% up, 25% down, 50% cruise

    /** Minimum output power — prevents the robot from crawling to a near-stop mid-move. */
    public static final double MIN_VELOCITY = 0.12;

    // ── Profile parameters set per-move ──────────────────────────────────────
    private final double totalDistance;
    private final double maxVelocity;
    private final double accelDistance;
    private final double decelStart;

    /**
     * @param totalDistance total inches to travel
     * @param maxVelocity   peak motor power (0…1) during cruise
     */
    public TrapezoidProfile(double totalDistance, double maxVelocity) {
        this.totalDistance = Math.max(totalDistance, 0.01); // avoid div-by-zero
        this.maxVelocity   = maxVelocity;

        // For very short moves the ramp zones overlap — cap them at half each
        double rampDist    = totalDistance * RAMP_FRACTION;
        this.accelDistance = rampDist;
        this.decelStart    = totalDistance - rampDist;
    }

    /**
     * Get the target velocity (motor power scale 0…1) at a given distance into the move.
     *
     * @param distanceTravelled inches already covered
     * @return target speed fraction (always ≥ MIN_VELOCITY while move is active)
     */
    public double getVelocity(double distanceTravelled) {
        if (distanceTravelled <= 0)                   return MIN_VELOCITY;
        if (distanceTravelled >= totalDistance)        return 0;

        double velocity;

        if (distanceTravelled < accelDistance) {
            // Ramp up
            double t = distanceTravelled / accelDistance;
            velocity = MIN_VELOCITY + t * (maxVelocity - MIN_VELOCITY);

        } else if (distanceTravelled < decelStart) {
            // Cruise
            velocity = maxVelocity;

        } else {
            // Ramp down
            double t = (distanceTravelled - decelStart) / (totalDistance - decelStart);
            velocity = maxVelocity - t * (maxVelocity - MIN_VELOCITY);
        }

        return Math.max(MIN_VELOCITY, velocity);
    }

    /** True once the profile has been completed. */
    public boolean isFinished(double distanceTravelled) {
        return distanceTravelled >= totalDistance;
    }

    public double getTotalDistance() { return totalDistance; }
}
