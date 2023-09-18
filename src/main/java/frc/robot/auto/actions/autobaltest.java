package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.submodules.Chassis;

/**
 * similar to the command interface from WPIlib
 * all actions should utilize this interface
 */

public class autobaltest implements Action {

    private double pitch = 0.0;
    private double prevPitch = 0.0;
    private Timer timer;
    private double prevTime = 0.0;
    private double balTime = 0.0;
    private Chassis chassis = Chassis.getInstance();

    public autobaltest() {}

    /**
     * returns whether the action is finished or not
     * 
     * @return if the action is finished
     */
    @Override
    public boolean isDone() {
       // // if ((pitch > -3 && pitch < 3) && (Timer.getFPGATimestamp() > (prevTime + 2.0))) { return true; }
        // else { return false; }
        //return (Chassis.getInstance().getPeriodicIO().pitch > -3 && Chassis.getInstance().getPeriodicIO().pitch < 3);
        //if (timer.get() > 2.0) { return true; }
        //else { return false; }
        if ((prevTime > (timer.get() + 2.0)) && (prevPitch > -3 && prevPitch < 3)) { System.out.println("done"); return false; }
        else { return false; }
    };

    /**
     * runs periodicly every 20? ms until isDone returns true
     */
    @Override
    public void update() {

        timer = new Timer();
        timer.start();
        pitch = Chassis.getInstance().getPeriodicIO().pitch;
        System.out.println("pitch: " + pitch);
        System.out.println("time: " + timer.get());
        

        if (Chassis.getInstance().getPeriodicIO().pitch > 7) {
            chassis.setPercentSpeed(-0.05, -0.05);
            System.out.println("going down");
        }

        if (Chassis.getInstance().getPeriodicIO().pitch < -7) {
            chassis.setPercentSpeed(0.05, 0.05);
            System.out.println("going up");
        }
        
        prevPitch = pitch;

        if (pitch > -3 && pitch < 3) {
            prevTime = timer.get();
            System.out.println(timer.get());
        }

        if (prevPitch < -3 || prevPitch > 3) {
            System.out.println(timer.get());
            timer.reset();
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
    public void initialize() {
        timer.reset();
    };

}
