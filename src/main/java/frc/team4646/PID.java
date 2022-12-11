package frc.team4646;

/** Constants to configure a PID controller */
public class PID {
    /** Proportional gain. Applied to error. Raw output / raw error. */
    public final double P;
    /** Integral gain. Applied to error. Raw output / sum of raw error. */
    public final double I;
    /** Derivative gain. Applied to error. Raw output / (err - prevErr). */
    public final double D;
    /**
     * Feedforward scaling constant. Applied to setpoint. Converts setpoint to
     * encoder units. Raw output / velocity in ticks per 100ms on Talons.
     */
    public final double F;

    public PID() {
        this(0.0, 0.0, 0.0, 0.0);
    }

    public PID(double P, double I, double D) {
        this(P, I, D, 0.0);
    }

    public PID(double P, double I, double D, double F) {
        this.P = P;
        this.I = I;
        this.D = D;
        this.F = F;
    }
}