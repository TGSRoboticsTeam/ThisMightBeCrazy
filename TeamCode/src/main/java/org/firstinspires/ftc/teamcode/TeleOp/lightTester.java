package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp(name = "lightTester", group = "Test")
public class lightTester extends LinearOpMode {

    private Servo light;

    // ============================================================
    // SWEEP PARAMETERS  <-- adjust here
    // ============================================================
    final double SWEEP_MIN      = 0.277;  // lower bound of sweep
    final double SWEEP_MAX      = 0.722;  // upper bound of sweep
    final double SWEEP_SPEED    = 0.003;  // position units per loop tick
    // ============================================================

    @Override
    public void runOpMode() {
        light = hardwareMap.get(Servo.class, "light");

        telemetry.addData("Status", "lightTester ready");
        telemetry.addData("Range", "%.3f  <->  %.3f", SWEEP_MIN, SWEEP_MAX);
        telemetry.update();

        waitForStart();

        double position  = SWEEP_MIN;
        int    direction = +1;          // +1 = sweeping toward MAX, -1 = toward MIN
        light.setPosition(position);

        while (opModeIsActive()) {
            position += direction * SWEEP_SPEED;

            // Reverse at each end
            if (position >= SWEEP_MAX) {
                position  = SWEEP_MAX;
                direction = -1;
            } else if (position <= SWEEP_MIN) {
                position  = SWEEP_MIN;
                direction = +1;
            }

            light.setPosition(position);

            telemetry.addData("Position",  "%.4f", position);
            telemetry.addData("Direction", direction > 0 ? "-> MAX" : "<- MIN");
            telemetry.update();
        }
    }
}
