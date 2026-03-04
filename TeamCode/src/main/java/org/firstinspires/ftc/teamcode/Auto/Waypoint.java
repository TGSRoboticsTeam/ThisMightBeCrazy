package org.firstinspires.ftc.teamcode.Auto;

/**
 * Waypoint
 * A single point in a path with optional mechanism action.
 *
 * Usage examples:
 *
 *   // Simple move, no action
 *   new Waypoint(24, 0, 0)
 *
 *   // Move then trigger mechanism on arrival
 *   new Waypoint(24, 24, Math.PI/2, () -> lift.setTarget(800))
 *
 *   // Trigger mechanism 6 inches BEFORE arriving (while still moving)
 *   new Waypoint(48, 24, 0, ActionTiming.ON_APPROACH, 6.0, () -> claw.close())
 */
public class Waypoint {

    // ── Timing modes ─────────────────────────────────────────────────────────
    public enum ActionTiming {
        ON_ARRIVE,    // default – robot stops at point, then runs action
        ON_APPROACH   // action fires X inches before arrival, robot keeps moving
    }

    // ── Fields ───────────────────────────────────────────────────────────────
    public final double x;              // inches, field-relative
    public final double y;              // inches, field-relative
    public final double heading;        // radians, target robot heading at this point

    public final Runnable      action;
    public final ActionTiming  timing;
    public final double        approachDistance; // only used for ON_APPROACH (inches)

    // ── Constructors ─────────────────────────────────────────────────────────

    /** Plain waypoint — move only, no action. */
    public Waypoint(double x, double y, double heading) {
        this(x, y, heading, ActionTiming.ON_ARRIVE, 0, null);
    }

    /** Waypoint with an action that fires on arrival (most common). */
    public Waypoint(double x, double y, double heading, Runnable action) {
        this(x, y, heading, ActionTiming.ON_ARRIVE, 0, action);
    }

    /** Waypoint with full timing control. */
    public Waypoint(double x, double y, double heading,
                    ActionTiming timing, double approachDistance, Runnable action) {
        this.x                = x;
        this.y                = y;
        this.heading          = heading;
        this.action           = action;
        this.timing           = timing;
        this.approachDistance = approachDistance;
    }

    /** Distance in inches from this waypoint to another. */
    public double distanceTo(double tx, double ty) {
        double dx = tx - x;
        double dy = ty - y;
        return Math.hypot(dx, dy);
    }

    /** True if there is an action attached. */
    public boolean hasAction() {
        return action != null;
    }
}
