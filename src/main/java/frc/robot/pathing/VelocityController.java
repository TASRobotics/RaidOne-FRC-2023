package frc.robot.pathing;


public class VelocityController {
    private double kP, kV, kA, kS, feedForward, feedBack;

    /** Default constructor - sets all gains to 0 */
    public VelocityController() {
        this(0.0, 0.0, 0.0, 0.0);
    }

    /**
     * Constructs the velocity controller with desired gain
     * 
     * @param kP proportional feedback gain
     * @param kV velocity feedforward gain
     * @param kA acceleration feedforward gain
     */
    public VelocityController(double kS, double kV, double kA, double kP) {
        this.kS = kS;
        this.kP = kP;
        this.kV = kV;
        this.kA = kA;
    }

    /**
     * Sets velocity controller gain
     * 
     * @param kP proportional feedback gain
     * @param kV velocity feedforward gain
     * @param kA acceleration feedforward gain
     */
    public void setGain(double kS, double kP, double kV, double kA) {
        this.kS = kS;
        this.kP = kP;
        this.kV = kV;
        this.kA = kA;
    }

    /**
     * Updates with corrected velocity
     * 
     * @param state trajectory state
     * @param measuredVel measured velocity
     * @return updated velocity
     */
    public double update(double velMPS, double accelMPS, double measuredVel) {
        feedForward = Math.signum(velMPS) * kS + kV * velMPS + kA * accelMPS;
        feedBack = kP * (velMPS - measuredVel);
        return feedForward + feedBack;
    }

    /**
     * Updates with corrected velocity based on FF
     * 
     * @param velMPS measured velocity
     * @param accelMPS measured acceleration
     * @return updated velocity
     */
    public double updateFF(double velMPS, double accelMPS) {
        feedForward = Math.signum(velMPS) * kS + kV * velMPS + kA * accelMPS;
        return feedForward;
    }
}
