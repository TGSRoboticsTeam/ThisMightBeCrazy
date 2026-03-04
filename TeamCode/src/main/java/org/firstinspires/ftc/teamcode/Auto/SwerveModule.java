package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * SwerveModule
 * Wraps one drive motor + one CRServo steer + one absolute encoder.
 * Logic is identical to justSwerve.runModule() so behaviour is consistent
 * between TeleOp and Auto.
 */
public class SwerveModule {

    // ── Hardware ────────────────────────────────────────────────────────────
    private final DcMotor   driveMotor;
    private final CRServo   steerServo;
    private final AnalogInput encoder;

    // ── Config ───────────────────────────────────────────────────────────────
    private final double encoderOffset;   // radians, from your OFFSET constants
    private final double STEER_KP      = 0.6;
    private final double STEER_DEADBAND = 0.05;

    // ── State ────────────────────────────────────────────────────────────────
    private double lastTargetAngle = 0;

    // ─────────────────────────────────────────────────────────────────────────
    public SwerveModule(DcMotor driveMotor, CRServo steerServo,
                        AnalogInput encoder, double encoderOffset) {
        this.driveMotor    = driveMotor;
        this.steerServo    = steerServo;
        this.encoder       = encoder;
        this.encoderOffset = encoderOffset;
    }

    /**
     * Set this module to a desired speed and field-relative angle.
     * Mirrors justSwerve.runModule() exactly, including the shortest-path
     * and wheel-reversal optimisation.
     *
     * @param speed       -1.0 … 1.0  (signed drive power)
     * @param targetAngle desired wheel heading in radians (-π … π)
     */
    public void set(double speed, double targetAngle) {
        double currentAngle = wrapAngle(getRawAngle() - encoderOffset);
        double delta        = wrapAngle(targetAngle - currentAngle);

        // Shortest-path optimisation: reverse drive rather than rotating >90°
        if (Math.abs(delta) > Math.PI / 2) {
            delta  = wrapAngle(delta + Math.PI);
            speed *= -1;
        }

        double servoPower = -STEER_KP * delta;   // negative matches TeleOp inversion
        if (Math.abs(servoPower) < STEER_DEADBAND) servoPower = 0;
        servoPower = Math.max(-1, Math.min(1, servoPower));

        steerServo.setPower(servoPower);
        driveMotor.setPower(speed);

        lastTargetAngle = targetAngle;
    }

    /** Stop drive motor; hold current steering angle. */
    public void stop() {
        driveMotor.setPower(0);
        // Keep servo running to hold position (set() with speed 0 handles this)
        set(0, lastTargetAngle);
    }

    /** Lock this module at a fixed heading (for X-lock). */
    public void lockAt(double angle) {
        set(0, angle);
    }

    /** Raw encoder reading → radians (0 … 2π). */
    public double getRawAngle() {
        return encoder.getVoltage() / 3.3 * (2 * Math.PI);
    }

    /** Current corrected heading in (-π … π). */
    public double getCurrentAngle() {
        return wrapAngle(getRawAngle() - encoderOffset);
    }

    // ── Utility ──────────────────────────────────────────────────────────────
    private double wrapAngle(double angle) {
        while (angle >  Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
