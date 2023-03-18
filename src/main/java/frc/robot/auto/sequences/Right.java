package frc.robot.auto.sequences;

import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.Constants.AutoConstants;
import frc.robot.auto.actions.DrivePath;
import frc.robot.auto.actions.LambdaAction;
import frc.robot.submodules.Chassis;
import frc.robot.auto.actions.ActionGroup;
import frc.robot.auto.actions.AutoBal;

import java.util.Arrays;

import com.pathplanner.lib.PathPlanner;

public class Right extends AutoSequence {
    private static final Trajectory path1 = PathPlanner.loadPath("Right", 1, AutoConstants.MAX_ACCEL);
    private static final Trajectory path2 = PathPlanner.loadPath("Right cone", 2, AutoConstants.MAX_ACCEL, true);

    public Right() {}

    @Override
    public void sequence() {
        addAction(
            new ActionGroup(Arrays.asList(
                //new DrivePath(path2, true),
                new DrivePath(path2, true),
                new DrivePath(path1),
                new AutoBal(),
                new LambdaAction(() -> Chassis.getInstance().smartHold())
            )
        ));
    }

    @Override
    public void onEnded() {
        System.out.println("Right ended!");
    }

    @Override
    public String getName() {
        return "Right";
    }
}
