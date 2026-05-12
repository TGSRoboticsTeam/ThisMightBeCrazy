package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoSetter", group = "Swerve")
public class ServoSetter extends LinearOpMode {

    private Servo leftTurret, rightTurret;

    @Override
    public void runOpMode() {

        leftTurret  = hardwareMap.get(Servo.class, "leftTurret");
        rightTurret = hardwareMap.get(Servo.class, "rightTurret");

        telemetry.addLine("ServoSetter ready.");
        telemetry.addLine("G2 A: servos -> 0.0");
        telemetry.addLine("G2 B: servos -> 1.0");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad2.a) {
                leftTurret.setPosition(0.0);
                rightTurret.setPosition(0.0);
            } else if (gamepad2.b) {
                leftTurret.setPosition(1.0);
                rightTurret.setPosition(1.0);
            }

            telemetry.addData("leftTurret",  "%.2f", leftTurret.getPosition());
            telemetry.addData("rightTurret", "%.2f", rightTurret.getPosition());
            telemetry.update();
        }
    }
}
