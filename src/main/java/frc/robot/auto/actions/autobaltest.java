package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.submodules.Chassis;

/**
 * similar to the command interface from WPIlib
 * all actions should utilize this interface
 */

public class autobaltest implements Action {

    double pitch = 0.0;
    double prevPitch = 0.0;
    int mode = 2;
    private Chassis chassis = Chassis.getInstance();

    public autobaltest() {}

    /**
     * returns whether the action is finished or not
     * 
     * @return if the action is finished
     */
    @Override
    public boolean isDone() {
        chassis.setPercentSpeed(0, 0);
        if (Chassis.getInstance().getPeriodicIO().pitch > -3 && Chassis.getInstance().getPeriodicIO().pitch < 3) { return true; }
        else { return false; }
        //return (Chassis.getInstance().getPeriodicIO().pitch > -3 && Chassis.getInstance().getPeriodicIO().pitch < 3);
    };

    /**
     * runs periodicly every 20? ms until isDone returns true
     */
    @Override
    public void update() {

        if (Chassis.getInstance().getPeriodicIO().pitch > 7) {
            chassis.setPercentSpeed(-0.2, -0.2);
            System.out.println("going down");
            prevPitch = pitch;
            mode = 0;
        }

        if (Chassis.getInstance().getPeriodicIO().pitch < -7) {
            chassis.setPercentSpeed(0.2, 0.2);
            System.out.println("going up");
            prevPitch = pitch;
            mode = 1;
        }

    };

    /**
     * runs once after action finishes
     */
    @Override
    public void done() {
        chassis.setPercentSpeed(0, 0);
    };

    /**
     * runs once when the action first starts
     */
    @Override
    public void initialize() {};

}
