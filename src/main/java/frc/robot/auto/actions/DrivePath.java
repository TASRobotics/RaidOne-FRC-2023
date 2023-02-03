package frc.robot.auto.actions;

import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.submodules.Chassis;
import frc.robot.submodules.Chassis.GearShift;
import frc.robot.utils.FinishConditionInterface;

/** Action for following a path. */
public class DrivePath implements Action {

    private static final Chassis chassis = Chassis.getInstance();

    private Trajectory path;
    private boolean isFirstPath;
    private FinishConditionInterface condition;

    /**
     * Constructs a DrivePath action that follows a path.
     * 
     * @param path the path to follow
     */
    public DrivePath(Trajectory path) {
        this(path, false, null);
    }

    /**
     * Constructs a DrivePath action that follows a path.
     * 
     * @param path the path to follow
     * @param isFirstPath whether this is the first path in an autonomous sequence
     */
    public DrivePath(Trajectory path, boolean isFirstPath) {
        this(path, isFirstPath, null);
    }

    /**
     * Constructs a DrivePath action that finishes when the path is finished or
     * if the provided lambda returns true.
     * 
     * @param path      path to follow
     * @param condition alternate condition to end the action
     */
    public DrivePath(Trajectory path, FinishConditionInterface condition) {
        this(path, false, condition);
    }

    /**
     * Constructs a DrivePath action that follows a path.
     * 
     * @param path the path to follow
     * @param isFirstPath whether this is the first path in an autonomous sequence
     * @param condition alternate condition to end the action
     */
    public DrivePath(Trajectory path, boolean isFirstPath, FinishConditionInterface condition) {
        this.path = path;
        this.isFirstPath = isFirstPath;
        this.condition = condition;
    }

    @Override
    public boolean isDone() {
        return chassis.isFinishedWithPath() || (condition != null && condition.passed());
    }

    @Override
    public void initialize() {
        chassis.changeShifterState(GearShift.HIGH_TORQUE);
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' started!");
        if (isFirstPath) {
            chassis.zero();
            /**
             * Set the odometry pose to the start of the trajectory.
             * Note: Should only do this on the first trajectory.
             */ 
            chassis.resetOdometry(path.getInitialPose());
        }
        // chassis.setGearShift(GearShift.LOW_TORQUE);
        chassis.setBrakeMode(true);
        chassis.setDrivePath(path);
    }

    @Override
    public void update() {
    }
    
    public void done() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' finished!");
        chassis.stop();
        // chassis.setBrakeMode(false);
    }
}