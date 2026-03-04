package org.firstinspires.ftc.teamcode.Auto;

/**
 * PIDController
 * Simple PID with integral windup guard and derivative filtering.
 * Used by SwerveDriveAuto for translation tracking and heading control.
 */
public class PIDController {

    private final double kP, kI, kD;

    private double integralSum   = 0;
    private double lastError     = 0;
    private double lastTimestamp = 0;

    private final double integralMax;   // windup clamp (same units as output)

    public PIDController(double kP, double kI, double kD, double integralMax) {
        this.kP          = kP;
        this.kI          = kI;
        this.kD          = kD;
        this.integralMax = integralMax;
    }

    public PIDController(double kP, double kI, double kD) {
        this(kP, kI, kD, Double.MAX_VALUE);
    }

    /**
     * Calculate PID output.
     *
     * @param error    current error (target − actual)
     * @param nowNanos System.nanoTime() — used to compute dt
     * @return         control output
     */
    public double calculate(double error, long nowNanos) {
        double dt = (lastTimestamp == 0) ? 0.02
                  : (nowNanos - lastTimestamp) / 1e9;
        lastTimestamp = nowNanos;

        // Integral with windup clamp
        integralSum += error * dt;
        integralSum  = Math.max(-integralMax, Math.min(integralMax, integralSum));

        // Derivative (change in error / dt)
        double derivative = (dt > 0) ? (error - lastError) / dt : 0;
        lastError = error;

        return kP * error + kI * integralSum + kD * derivative;
    }

    /** Reset integral and derivative state (call between moves). */
    public void reset() {
        integralSum   = 0;
        lastError     = 0;
        lastTimestamp = 0;
    }
}
