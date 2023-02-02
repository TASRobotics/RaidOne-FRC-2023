package frc.robot.auto.actions;

/**
 * similar to the command interface from WPIlib
 * all actions should utilize this interface
 */

public interface Action {

    /**
     * returns whether the action is finished or not
     * 
     * @return if the action is finished
     */
    boolean isDone();

    /**
     * runs periodicly every 20? ms until isDone returns true
     */
    void update();

    /**
     * runs once after action finishes
     */
    void done();

    /**
     * runs once when the action first starts
     */
    void initialize();

}
