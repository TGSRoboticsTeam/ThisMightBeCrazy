package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Auto.Waypoint.ActionTiming;

/**
 * ExampleAuto
 * ───────────────────────────────────────────────────────────────────────────
 * Demonstrates the SwerveDriveAuto API with a realistic FTC scoring routine.
 *
 * Robot starts at field origin (0, 0) facing 0 radians.
 * Positive X = forward, positive Y = left, angles CCW positive.
 *
 * Replace the mechanism calls (lift, claw, etc.) with your actual mechanism
 * classes — this file is purely a usage template.
 */
@Autonomous(name = "ExampleAuto", group = "Auto")
public class ExampleAuto extends LinearOpMode {

    // ── Swap these for your actual mechanism objects ──────────────────────────
    // private Lift lift;
    // private Claw claw;

    private SwerveDriveAuto drive;

    @Override
    public void runOpMode() {

        // ── Initialise ───────────────────────────────────────────────────────
        drive = new SwerveDriveAuto(this);

        // Init with robot starting pose. Adjust if robot doesn't start at origin.
        drive.init(0, 0, 0);

        // lift = new Lift(hardwareMap);
        // claw = new Claw(hardwareMap);

        telemetry.addLine("Initialised — waiting for start");
        telemetry.update();
        waitForStart();
        if (!opModeIsActive()) return;

        // ════════════════════════════════════════════════════════════════════
        //  OPTION A — followPath()
        //  Define the entire routine as an array of waypoints.
        //  Easy to modify: just add, remove, or reorder Waypoint lines.
        // ════════════════════════════════════════════════════════════════════

        drive.followPath(new Waypoint[]{

            // 1. Drive forward 24 inches, no action
            new Waypoint(24, 0, 0),

            // 2. Strafe left 24 inches, raise lift on arrival
            new Waypoint(24, 24, 0, () -> {
                // lift.setTarget(800);
                sleep(300);   // wait for lift to reach height
            }),

            // 3. Turn 90° left while driving to next point
            new Waypoint(36, 24, Math.PI / 2),

            // 4. Start closing claw 6 inches BEFORE arriving (ON_APPROACH)
            new Waypoint(36, 36, Math.PI / 2,
                    ActionTiming.ON_APPROACH, 6.0,
                    () -> {
                        // claw.close();
                    }),

            // 5. Drive back to origin, lower lift on arrival
            new Waypoint(0, 0, 0, () -> {
                // lift.setTarget(0);
            }),
        });


        // ════════════════════════════════════════════════════════════════════
        //  OPTION B — individual driveTo() calls
        //  More explicit — good for routines with lots of mechanism logic
        //  between moves, or when you want fine control over timing.
        // ════════════════════════════════════════════════════════════════════

        /*
        // Move 1 — drive forward
        drive.driveTo(24, 0, 0);

        // Mechanism action between moves
        drive.runAction(() -> {
            // lift.setTarget(800);
            sleep(400);
        });

        // Move 2 — strafe left
        drive.driveTo(24, 24, 0);

        // Move 3 — relative move (12 inches further forward)
        drive.driveRelative(12, 0, 0);

        // Rotate in place to face 180°
        drive.rotateTo(Math.PI);

        // Move 4 — drive back to start
        drive.driveTo(0, 0, 0);
        */


        // ── End of routine ───────────────────────────────────────────────────
        drive.stopAndLock();
        telemetry.addLine("Auto complete");
        telemetry.update();
    }
}
